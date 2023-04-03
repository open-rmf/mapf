/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use crate::{
    motion::{
        Trajectory, Waypoint, TimePoint, Interpolation, Motion, Timed, Duration,
        Environment, CircularProfile, DynamicCircularObstacle, BoundingBox,
        IntegrateWaypoints,
        r2::{WaypointR2 as WaypointR2, Point},
    },
    error::ThisError,
};
use smallvec::SmallVec;
use arrayvec::ArrayVec;
use std::cmp::Ordering;

type Vector2 = nalgebra::Vector2<f64>;


#[derive(Debug, Clone, Copy)]
pub enum SafeAction<Movement, WaitFor> {
    /// Move to the designated point
    Move(Movement),
    /// Wait for some condition to be met
    Wait(WaitFor),
}

impl<Movement, WaitFor> SafeAction<Movement, WaitFor> {
    pub fn movement(&self) -> Option<&Movement> {
        match self {
            Self::Move(m) => Some(m),
            _ => None,
        }
    }

    pub fn map_movement<U, F: FnOnce(Movement) -> U>(self, op: F) -> SafeAction<U, WaitFor> {
        match self {
            SafeAction::Move(m) => SafeAction::Move(op(m)),
            SafeAction::Wait(w) => SafeAction::Wait(w),
        }
    }

    pub fn wait_for(&self) -> Option<&WaitFor> {
        match self {
            Self::Wait(w) => Some(w),
            _ => None,
        }
    }
}

impl<Movement, WaitFor> From<Movement> for SafeAction<Movement, WaitFor> {
    fn from(value: Movement) -> Self {
        SafeAction::Move(value)
    }
}

// TODO(@mxgrey): Generalize this beyond WaitForObstacle, e.g. come up with a
// trait to describe waiting actions.
impl<W, M> IntegrateWaypoints<W> for SmallVec<[SafeAction<M, WaitForObstacle>; 5]>
where
    W: Timed + Clone,
    M: Into<W> + Clone,
{
    type IntegratedWaypointIter<'a> = SmallVec<[Result<W, SafeActionIntegrateWaypointError>; 5]>
    where
        W: 'a,
        M: 'a;
    type WaypointIntegrationError = SafeActionIntegrateWaypointError;
    fn integrated_waypoints<'a>(
        &'a self,
        initial_waypoint: Option<W>,
    ) -> Self::IntegratedWaypointIter<'a>
    where
        Self: 'a,
        Self::WaypointIntegrationError: 'a,
        W: 'a,
    {
        let mut initial_waypoint = match initial_waypoint {
            Some(wp) => wp,
            None => return SmallVec::from_iter([
                Err(SafeActionIntegrateWaypointError::MissingInitialWaypoint)
            ]),
        };

        let mut waypoints = SmallVec::new();
        for action in self {
            let wp: W = match action {
                SafeAction::Move(wp) => wp.clone().into(),
                SafeAction::Wait(wait) => initial_waypoint.with_time(wait.time_estimate),
            };

            waypoints.push(Ok(wp.clone()));
            initial_waypoint = wp;
        }

        waypoints
    }
}

#[derive(Debug, ThisError)]
pub enum SafeActionIntegrateWaypointError {
    #[error("An initial waypoint is needed to integrate SafeAction waypoints")]
    MissingInitialWaypoint,
}

#[derive(Clone, Copy)]
pub struct WaitForObstacle {
    /// Which obstacle needs to be waited on
    pub for_obstacle: usize,
    /// How long is it necessary to wait
    pub time_estimate: TimePoint,
}

impl std::fmt::Debug for WaitForObstacle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f
            .debug_struct("WaitForObstacle")
            .field("for_obstacle", &self.for_obstacle)
            .field("time_estimate", &self.time_estimate.as_secs_f64())
            .finish()
    }
}

pub fn compute_safe_linear_paths<Env, W>(
    from_point: WaypointR2,
    to_point: WaypointR2,
    in_environment: &Env,
) -> SmallVec<[SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 5]>; 3]>
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let profile = in_environment.agent_profile();
    let delta_t = (to_point.time - from_point.time).as_secs_f64();
    assert!(delta_t >= 0.0);
    let bb = BoundingBox::for_line(profile, &from_point, &to_point);
    if delta_t < 1e-8 {
        // This very small time interval suggests that the agent is not moving
        // significantly. Just check if the proposed path is safe or not.
        if is_safe_segment((&from_point, &to_point), Some(bb), in_environment) {
            return SmallVec::from_iter([
                SmallVec::from_iter([
                    SafeAction::Move(to_point)
                ])
            ]);
        } else {
            // The segment is not safe and it's too close to being a singularity
            // to make it safe, so we'll just return that there are no safe paths
            return SmallVec::new();
        }
    }

    let ranked_hints = compute_safe_linear_path_wait_hints(
        (&from_point, &to_point), Some(bb), in_environment
    );

    compute_safe_arrival_times(to_point, in_environment)
        .into_iter()
        .filter_map(|arrival_time|
            compute_safe_arrival_path(
                from_point,
                to_point,
                arrival_time,
                &ranked_hints,
                in_environment,
            )
        ).collect()
}

pub type RankedHints = SmallVec<[RankedHint; 16]>;

pub fn compute_safe_linear_path_wait_hints<Env, W>(
    (from_point, to_point): (&WaypointR2, &WaypointR2),
    bounding_box: Option<BoundingBox>,
    in_environment: &Env,
) -> RankedHints
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let bb = bounding_box.unwrap_or_else(
        || BoundingBox::for_line(
            in_environment.agent_profile(), from_point, to_point
        )
    );

    let wait_hints = compute_wait_hints(
        (&from_point, &to_point), &bb, in_environment
    );

    let dx = to_point.position - from_point.position;
    let dist = dx.norm();
    assert!(dist >= 1e-8);
    let u = dx / dist;
    let dt = (to_point.time - from_point.time).as_secs_f64();
    let speed = dist / dt;

    let mut ranked_hints: SmallVec<[RankedHint; 16]> = SmallVec::new();
    for hint in wait_hints {
        let reach = (hint.at_point - from_point.position).dot(&u);
        let wait_t = (hint.until - from_point.time).as_secs_f64();
        let contour = if speed > 1e-8 {
            Duration::from_secs_f64(wait_t - reach / speed)
        } else {
            Duration::from_secs_f64(wait_t)
        };

        ranked_hints.push(RankedHint { contour, reach, hint });
    }

    ranked_hints.sort_by(|a, b| {
        match a.contour.cmp(&b.contour) {
            Ordering::Equal => {
                if a.reach <= b.reach {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            }
            other => other,
        }
    });

    ranked_hints
}

// TODO(@mxgrey): Consider making the number of entries configurable.
pub type SafeArrivalTimes = SmallVec<[TimePoint; 5]>;

#[inline]
pub fn compute_safe_arrival_times<Env, W>(
    for_point: WaypointR2,
    in_environment: &Env,
) -> SafeArrivalTimes
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let profile = in_environment.agent_profile();
    let mut safe_arrival_times = SafeArrivalTimes::new();

    // First find every safe arrival time above from_point.time
    let mut candidate_time = for_point.time;
    loop {
        let mut any_pushed = true;
        while any_pushed {
            any_pushed = false;
            for obs in in_environment.obstacles() {
                let obs_traj = match obs.trajectory() {
                    Some(r) => r,
                    None => continue,
                };

                let critical_distance_squared = profile.critical_distance_squared_for(obs.profile());
                let adjustment = adjust_candidate_time(
                    for_point,
                    candidate_time,
                    obs_traj,
                    critical_distance_squared,
                );

                if !adjustment.pushed {
                    continue;
                }

                if let Some(new_candidate_time) = adjustment.result {
                    candidate_time = new_candidate_time;
                    any_pushed = true;
                } else {
                    // This obstacle is violated and no end could be
                    // found for the violation, so there will never be
                    // another safe arrival time.
                    return safe_arrival_times;
                }
            }
        }

        // We found a new safe arrival time
        safe_arrival_times.push(candidate_time);

        // Look for the next soonest candidate time
        let mut next_candidate_time = None;
        for obs in in_environment.obstacles() {
            let min_distance_squared = profile.critical_distance_squared_for(obs.profile());

            let obs_traj = match obs.trajectory() {
                Some(r) => r,
                None => continue,
            };

            if let Some(check) = find_next_candidate_time(
                candidate_time,
                for_point,
                obs_traj,
                min_distance_squared,
            ) {
                if let Some(next_candidate_time) = &mut next_candidate_time {
                    if check < *next_candidate_time {
                        *next_candidate_time = check;
                    }
                } else {
                    next_candidate_time = Some(check);
                }
            }
        }

        candidate_time = match next_candidate_time {
            Some(t) => t,
            // There are no more candidate times to consider
            None => break,
        };
    }

    safe_arrival_times
}

#[inline]
pub fn compute_safe_arrival_path<Env, W>(
    from_point: WaypointR2,
    to_point: WaypointR2,
    arrival_time: TimePoint,
    ranked_hints: &RankedHints,
    in_environment: &Env,
) -> Option<SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 5]>>
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let dx = to_point.position - from_point.position;
    let dist = dx.norm();
    assert!(dist >= 1e-8);
    let dt = (to_point.time - from_point.time).as_secs_f64();
    let speed = dist / dt;

    let can_arrive_safely = |from_p: WaypointR2| -> bool {
        let t_arrival = (to_point.position - from_p.position).norm() / speed;
        let arrival_wp = WaypointR2::new(
            from_p.time + Duration::from_secs_f64(t_arrival),
            to_point.position.x,
            to_point.position.y,
        );

        if !is_safe_segment((&from_p, &arrival_wp), None, in_environment) {
            return false;
        }

        if arrival_wp.time < arrival_time {
            let final_wp = arrival_wp.with_time(arrival_time);
            return is_safe_segment((&arrival_wp, &final_wp), None, in_environment);
        }

        true
    };

    let make_parent_arrival = |child_wp: WaypointR2, parent_wp: WaypointR2| -> WaypointR2 {
        let hint_p = parent_wp.position;
        let dt_hint_arrival = (hint_p - child_wp.position).norm() / speed;
        let t_hint_arrival = child_wp.time + Duration::from_secs_f64(dt_hint_arrival);
        parent_wp.with_time(t_hint_arrival)
    };

    // First test if we can just go straight to the goal
    if can_arrive_safely(from_point) {
        // We can just go straight from the start point to the goal point and
        // safely wait at the goal until the arrival time, so let's just do
        // that.
        return Some(SmallVec::from_iter([SafeAction::Move(to_point)]));
    }

    if ranked_hints.is_empty() {
        // It will be impossible to find any path if the direct path is not
        // valid and no hints exist. The presence of obstacles combined with the
        // absence of hints implies that the obstacles are permanent and there
        // will never be a way around them.
        return None;
    }

    let mut search: SmallVec<[ArrivalPathSearchElement; 16]> = SmallVec::new();
    {
        let goal_contour = {
            let goal_reach = (to_point.position - from_point.position).norm();
            let goal_dt = arrival_time - from_point.time;
            if speed > 1e-8 {
                Duration::from_secs_f64(goal_dt.as_secs_f64() - goal_reach / speed)
            } else {
                goal_dt
            }
        };
        let mut found_contour: Option<Duration> = None;
        for (i, hint) in ranked_hints.iter().enumerate() {
            if hint.contour < goal_contour {
                continue;
            }

            if let Some(found_contour) = found_contour {
                if found_contour < hint.contour {
                    // Don't bother searching anymore, we're past the lowest
                    // acceptable contour.
                    break;
                }

                // Swap the previous initial search point with this new one
                search.clear();
                search.push(ArrivalPathSearchElement::new(i));
            } else {
                found_contour = Some(hint.contour);
                search.push(ArrivalPathSearchElement::new(i));
            }
        }
    }

    while let Some(element) = search.pop() {
        let ranking = match ranked_hints.get(element.index) {
            Some(h) => h,
            None => continue,
        };
        let hint_wp: WaypointR2 = ranking.hint.into();

        if !element.tested_arrival_to_parent {
            // Test if we can safely reach the parent point from the hint point.
            let parent_wp_end = match search.last() {
                Some(parent) => ranked_hints.get(parent.index).unwrap().hint.into(),
                None => {
                    let dt = (to_point.position - hint_wp.position).norm() / speed;
                    to_point.with_time(hint_wp.time + Duration::from_secs_f64(dt))
                }
            };

            let parent_wp_start = make_parent_arrival(hint_wp, parent_wp_end);
            let safe_hint_arrival = is_safe_segment(
                (&hint_wp, &parent_wp_start), None, in_environment
            ) && is_safe_segment(
                (&parent_wp_start, &parent_wp_end), None, in_environment
            );

            if !safe_hint_arrival {
                // We can't safely reach the parent from this hint. We need to
                // cull this branch, or if there's no parent then we need to
                // increment by one to try out the next best final arrival hint.
                if search.is_empty() {
                    // Climb higher if possible
                    if element.index + 1 < ranked_hints.len() {
                        search.push(ArrivalPathSearchElement::new(element.index + 1));
                    }
                }
                continue;
            }
        }
        let element = element.tested();

        if let Some(child) = element.child {
            search.push(element.decrement_child());

            let child_reach = ranked_hints.get(child).unwrap().reach;
            if child_reach <= ranking.reach {
                // If this child has a lesser or equal reach to the parent then
                // we can add it to the search queue to consider it.
                search.push(ArrivalPathSearchElement::new(child));
            }
        } else {
            // Test if the start waypoint can arrive at this hint. If it can
            // then we have found our path.
            let hint_arrival_wp = make_parent_arrival(from_point, hint_wp);
            let safe_hint_arrival = is_safe_segment(
                (&from_point, &hint_arrival_wp), None, in_environment
            ) && is_safe_segment(
                (&hint_arrival_wp, &hint_wp), None, in_environment
            );
            if safe_hint_arrival {
                // We have found the desired path
                search.push(element);
                search.reverse();
                let mut path: SmallVec<[SafeAction<_, _>; 5]> = SmallVec::new();
                let mut previous_wp = from_point;
                for element in &search {
                    let hint = ranked_hints.get(element.index).unwrap().hint;
                    let hint_wp_end = hint.into();
                    let hint_wp_start = make_parent_arrival(previous_wp, hint_wp_end);
                    path.push(SafeAction::Move(hint_wp_start));
                    path.push(SafeAction::Wait(WaitForObstacle {
                        for_obstacle: hint.for_obstacle,
                        time_estimate: hint.until,
                    }));

                    previous_wp = hint_wp_end;
                }

                let final_wp = make_parent_arrival(previous_wp, to_point);
                path.push(SafeAction::Move(final_wp));
                return Some(path);
            }
        };
    }

    None
}

#[derive(Debug)]
pub struct RankedHint {
    contour: Duration,
    reach: f64,
    hint: WaitHint,
}

#[derive(Debug)]
struct ArrivalPathSearchElement {
    index: usize,
    child: Option<usize>,
    tested_arrival_to_parent: bool,
}

impl ArrivalPathSearchElement {
    fn new(index: usize) -> Self {
        Self {
            index,
            child: if index > 0 { Some(index-1) } else { None },
            tested_arrival_to_parent: false,
        }
    }

    fn decrement_child(mut self) -> Self {
        self.child = if let Some(child) = self.child.filter(|c| *c > 0) {
            Some(child - 1)
        } else {
            None
        };
        self
    }

    fn tested(mut self) -> Self {
        self.tested_arrival_to_parent = true;
        self
    }
}

#[inline]
fn compute_t_range(
    (wp0_a, wp1_a): (&WaypointR2, &WaypointR2),
    (wp0_b, wp1_b): (&WaypointR2, &WaypointR2),
) -> (TimePoint, TimePoint) {
    (wp0_a.time.max(wp0_b.time), wp1_a.time.min(wp1_b.time))
}

#[inline]
fn compute_p0_v(
    (wp0, wp1): (&WaypointR2, &WaypointR2),
    t_range: &(TimePoint, TimePoint),
) -> (Vector2, Vector2) {
    let dp = wp1.position - wp0.position;
    let dt = (wp1.time - wp0.time).as_secs_f64();
    let p0 = wp0.interpolate(&wp1).compute_position(&t_range.0).unwrap();
    (p0.coords, dp / dt)
}

#[inline]
fn compute_dp0_dv(
    line_a: (&WaypointR2, &WaypointR2),
    line_b: (&WaypointR2, &WaypointR2),
    t_range: &(TimePoint, TimePoint),
) -> (Vector2, Vector2) {
    let (p0_a, v_a) = compute_p0_v(line_a, t_range);
    let (p0_b, v_b) = compute_p0_v(line_b, t_range);
    let dp0 = p0_b - p0_a;
    let dv = v_b - v_a;

    (dp0, dv)
}

#[derive(Clone, Copy)]
struct Proximity {
    enter: Option<TimePoint>,
    exit: Option<TimePoint>,
}

impl Proximity {
    fn none() -> Self {
        Self { enter: None, exit: None }
    }
}

impl std::fmt::Debug for Proximity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f
            .debug_struct("Proximity")
            .field("enter", &self.enter.as_ref().map(|t| t.as_secs_f64()))
            .field("exit", &self.exit.as_ref().map(|t| t.as_secs_f64()))
            .finish()
    }
}

#[inline]
fn detect_proximity(
    proximity_dist_squared: f64,
    line_a: (&WaypointR2, &WaypointR2),
    line_b: (&WaypointR2, &WaypointR2),
) -> Proximity {
    let t_range = compute_t_range(line_a, line_b);
    if (t_range.0 - t_range.1).as_secs_f64().abs() < 1e-4 {
        // One or both of the segments is approximately a single point, so just
        // check whether the lines are within proximity at either time point.
        for t in [t_range.0, t_range.1] {
            let p_a = line_a.0.interpolate(&line_a.1).compute_position(&t);
            let p_b = line_b.0.interpolate(&line_b.1).compute_position(&t);
            if let (Ok(p_a), Ok(p_b)) = (p_a, p_b) {
                // TODO(@mxgrey): What if either is Err? Is that even possible?
                // If what would it mean if that happens?
                let dp = p_a - p_b;
                if dp.dot(&dp) <= proximity_dist_squared {
                    return Proximity {
                        enter: Some(t),
                        exit: None,
                    };
                }
            }
        }

        return Proximity::none();
    }

    let (dp0, dv) = compute_dp0_dv(line_a, line_b, &t_range);
    let a = dv.dot(&dv);
    let b = 2.0 * dv.dot(&dp0);
    let c = dp0.dot(&dp0) - proximity_dist_squared;

    if a.abs() < 1e-8 {
        // There is virtually no relative motion
        assert!(b.abs() < 1e-4);
        if c <= 0.0 {
            // The objects are within the proximity limit
            return Proximity {
                enter: Some(t_range.0),
                // The objects are not moving relative to each other so they
                // cannot exit
                exit: None,
            };
        } else {
            // The objects are not within proximity
            return Proximity::none();
        }
    }

    let radicand = b.powi(2) - 4.0 * a * c;
    if radicand < 0.0 {
        // The objects are moving relative to each other, but they will never come
        // close enough to each other.
        return Proximity { enter: None, exit: None };
    }

    let sqrt_radicand = radicand.sqrt();
    let dt = (t_range.1 - t_range.0).as_secs_f64();
    let t_m = (-b - sqrt_radicand) / (2.0 * a);
    let t_p = (-b + sqrt_radicand) / (2.0 * a);
    assert!(t_m <= t_p);

    if t_p < 0.0 {
        // The intersection happens before the time range
        return Proximity::none();
    }

    if dt < t_m {
        // The intersection happens after the time range
        return Proximity::none();
    }

    return Proximity {
        enter: Some(t_range.0 + Duration::from_secs_f64(f64::max(0.0, t_m))),
        exit: if t_p <= dt {
            Some(t_range.0 + Duration::from_secs_f64(t_p))
        } else {
            None
        }
    };
}

struct CandidateAdjustment {
    pushed: bool,
    result: Option<TimePoint>,
}

#[inline]
fn adjust_candidate_time<W>(
    for_point: WaypointR2,
    candidate_time: TimePoint,
    obs_traj: &Trajectory<W>,
    acceptable_distance_squared: f64,
) -> CandidateAdjustment
where
    W: Into<WaypointR2> + Waypoint,
{
    // Has this obstacle pushed the candidate time?
    let mut this_obs_violated = false;
    let mut new_candidate_time: Option<TimePoint> = None;
    for [wp0, wp1] in obs_traj.iter_from(candidate_time).pairs() {
        if candidate_time < *wp0.time() && !this_obs_violated {
            break;
        }

        let wp0: WaypointR2 = wp0.into();
        let wp1: WaypointR2 = wp1.into();
        let proximity = detect_proximity(
            acceptable_distance_squared,
            (
                &for_point.with_time(wp0.time),
                &for_point.with_time(wp1.time),
            ),
            (&wp0, &wp1),
        );

        if this_obs_violated {
            // Look for a time where the footprints are no longer
            // overlapping
            let exit = match proximity.exit {
                Some(exit) => exit,
                None => continue,
            };

            // If everything is working correctly, it should not be
            // possible for this exit time to be less than the
            // candidate time (but we add some buffer to accommodate
            // floating point error).
            assert!(exit + time_point::Duration::new(10000) >= candidate_time);
            new_candidate_time = Some(exit.max(candidate_time));
            break;
        } else {
            let enter = match proximity.enter {
                Some(enter) => enter,
                None => continue,
            };

            if enter > candidate_time {
                // The violation happens after the candidate time
                // so we can safely ignore it. It is also not
                // possible for any more trajectory segments to
                // violate the target at the candidate time.
                break;
            }

            this_obs_violated = true;
            let exit = match proximity.exit {
                Some(exit) => exit,
                None => continue,
            };

            if exit > candidate_time {
                // The violation is both entered and exited within
                // this trajectory segment.
                new_candidate_time = Some(exit);
                break;
            }

            if exit <= candidate_time {
                // The violation was relieved before the candidate
                // time so it is not a relevant violation.
                this_obs_violated = false;
            }
        }
    }

    if this_obs_violated && new_candidate_time.is_none() {
        if let Some(tf) = obs_traj.finish_time() {
            if candidate_time < tf {
                new_candidate_time = Some(tf);
            }
        }
    }

    CandidateAdjustment {
        pushed: this_obs_violated,
        result: new_candidate_time,
    }
}

#[inline]
fn find_next_candidate_time<W>(
    previous_candidate_time: TimePoint,
    for_point: WaypointR2,
    obs_traj: &Trajectory<W>,
    acceptable_distance_squared: f64,
) -> Option<TimePoint>
where
    W: Into<WaypointR2> + Waypoint,
{
    for [wp0, wp1] in obs_traj.iter_from(previous_candidate_time).pairs() {
        let wp0: WaypointR2 = wp0.into();
        let wp1: WaypointR2 = wp1.into();
        let proximity = detect_proximity(
            acceptable_distance_squared,
            (
                &for_point.with_time(wp0.time),
                &for_point.with_time(wp1.time),
            ),
            (&wp0, &wp1),
        );

        if let Some(exit) = proximity.exit {
            if previous_candidate_time < exit {
                return Some(exit);
            }
        }
    }

    if let Some(tf) = obs_traj.finish_time() {
        if previous_candidate_time < tf {
            // Check if the trajectory ends within the critical distance. If it
            // does then we should use its vanishing time as a candidate time.
            let wpf: WaypointR2 = obs_traj.finish_motion().clone().into();
            let dq = wpf.position - for_point.position;
            if dq.dot(&dq) <= acceptable_distance_squared {
                return Some(tf);
            }
        }
    }

    None
}

#[inline]
pub fn is_safe_segment<Env, W>(
    line_a: (&WaypointR2, &WaypointR2),
    bb: Option<BoundingBox>,
    in_environment: &Env,
) -> bool
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let profile = in_environment.agent_profile();
    let bb = match bb {
        Some(bb) => bb,
        None => BoundingBox::for_line(profile, line_a.0, line_a.1),
    };

    for obs in in_environment.obstacles() {
        let conflict_distance_squared = profile.conflict_distance_squared_for(obs.profile());
        let obs_traj = match obs.trajectory() {
            Some(r) => r,
            None => continue,
        };

        if !bb.overlaps(obs.bounding_box().cloned()) {
            continue;
        }

        for [wp0_b, wp1_b] in obs_traj.iter_range(line_a.0.time, line_a.1.time).pairs() {
            if line_a.1.time < *wp0_b.time() {
                // The trajectories are no longer overlapping in time so there
                // is no longer a risk.
                return true;
            }

            let wp0_b: WaypointR2 = wp0_b.into();
            let wp1_b: WaypointR2 = wp1_b.into();
            let line_b = (&wp0_b, &wp1_b);

            if !bb.overlaps(Some(
                BoundingBox::for_line(obs.profile(), &wp0_b, &wp1_b)
                .inflated_by(1e-3)
            )) {
                continue;
            }

            let in_time_range = |t: &TimePoint| -> bool {
                line_a.0.time < *t && *t < line_a.1.time
                && line_b.0.time < *t && *t < line_b.1.time
            };

            let proximity = detect_proximity(
                conflict_distance_squared,
                line_a,
                line_b,
            );

            if let Some(t) = proximity.enter.filter(in_time_range) {
                let t_range = compute_t_range(line_a, line_b);
                let t = (t - t_range.0).as_secs_f64();
                let (dp0, dv) = compute_dp0_dv(line_a, line_b, &t_range);
                let a = dv.dot(&dv);
                let b = 2.0 * dv.dot(&dp0);
                let deriv = 2.0*a * t + b;
                // Allow for a little floating point error. When the derivative
                // is very very close to zero (which is a perfectly acceptable
                // value), floating point calculation errors can cause its
                // calculated value to dip into the negative.
                //
                // TODO(@mxgrey): Consider if there are more robust ways (not
                // sensitive to floating point error) to determine whether the
                // segment is safe.
                if deriv < -1e-6 {
                    // The distance between the agents is reducing while they
                    // are already within an unsafe proximity, so we will call
                    // this situation unsafe.
                    return false;
                }
            }
        }
    }

    true
}

#[inline]
fn compute_wait_hints<Env, W>(
    (wp0, wp1): (&WaypointR2, &WaypointR2),
    bb: &BoundingBox,
    in_environment: &Env,
) -> SmallVec<[WaitHint; 16]>
where
    W: Into<WaypointR2> + Waypoint,
    Env: Environment<CircularProfile, DynamicCircularObstacle<W>>,
{
    let profile = in_environment.agent_profile();
    let q = wp0.position;
    let r = wp1.position;
    let u = match (r -q).try_normalize(1e-8) {
        Some(u) => u,
        // The agent is hardly moving so don't bother looking for where it needs
        // to wait.
        None => return SmallVec::new(),
    };
    let agent_dt = (wp1.time - wp0.time).as_secs_f64();
    let agent_v = (r - q) / agent_dt;
    let agent_speed = agent_v.norm();
    let s_max = (r - q).norm();

    let mut wait_hints = SmallVec::new();
    for (for_obstacle, obs) in in_environment.obstacles().into_iter().enumerate() {
        if !bb.overlaps(obs.bounding_box().cloned()) {
            continue;
        }

        let obs_traj = match obs.trajectory() {
            Some(t) => t,
            None => continue,
        };

        let min_distance = profile.critical_distance_for(obs.profile());
        let min_distance_squared = min_distance.powi(2);
        for [obs_wp0, obs_wp1] in obs_traj.iter().pairs() {
            let obs_wp0: WaypointR2 = obs_wp0.into();
            let obs_wp1: WaypointR2 = obs_wp1.into();
            if !bb.overlaps(Some(
                BoundingBox::for_line(obs.profile(), &obs_wp0, &obs_wp1)
                .inflated_by(1e-3)
            )) {
                continue;
            }

            let obs_q = obs_wp0.position;
            let obs_r = obs_wp1.position;
            let obs_dt = (obs_wp1.time - obs_wp0.time).as_secs_f64();
            let obs_t0 = obs_wp0.time;
            let obstruction_time_range = compute_quadratic_wait_hints(
                q, r, u,
                obs_q, obs_r, obs_dt,
                min_distance_squared,
            );

            if let Some((_, t_end)) = obstruction_time_range {
                // There should be a valid unit vector from the start to the end
                // of the obstacle line segment if it was possible to calculate
                // an obstruction time range.
                let obs_u = (obs_r - obs_q).try_normalize(1e-8).unwrap();

                // Find the times that the main trajectory is within proximity
                // of the line so that we can compute the waiting location.
                let wait_point_time = compute_quadratic_wait_hints(
                    obs_q, obs_r, obs_u,
                    q, r, agent_dt,
                    min_distance_squared,
                );

                if let Some((t_wait_at, _)) = wait_point_time {
                    if 0.0 <= t_wait_at {
                        let until = obs_t0 + Duration::from_secs_f64(t_end);
                        let arrival = wp0.time + Duration::from_secs_f64(t_wait_at);
                        if arrival <= until {
                            let at_point = q + agent_v * t_wait_at;
                            wait_hints.push(WaitHint { at_point, until, for_obstacle });
                        }
                    }
                }
            }

            let obs_v = (obs_r - obs_q) / obs_dt;
            let aligned_speed = obs_v.dot(&u);
            if aligned_speed >= 0.0 && wp0.time <= obs_wp1.time {
                // The obstacle is either stationary or moving in the same
                // direction as the agent. We should create hints based on
                // following the obstacle from behind.
                if agent_speed <= aligned_speed {
                    // The obstacle moves faster than the agent along the
                    // agent's path, so there is no need to do any waiting to
                    // produce a valid action. Follower checkpoints will be
                    // added in post-processing.
                } else if aligned_speed > 1e-2 {
                    // The obstacle is slower than the agent speed. We should
                    // use a sawtooth pattern, moving forward until we've caught
                    // up to the obstacle, then waiting to give the obstacle
                    // time to get ahead.
                    let (dq, mut t) = if obs_wp0.time < wp0.time {
                        let t_offset = (wp0.time - obs_wp0.time).as_secs_f64();
                        let dq = (obs_q + obs_v * t_offset) - q;
                        (dq, wp0.time.as_secs_f64())
                    } else {
                        let dq = obs_q - q;
                        (dq, obs_wp0.time.as_secs_f64())
                    };

                    if dq.dot(&dq) >= min_distance_squared {
                        let follow_distance = profile.follow_distance_for(obs.profile());
                        let v_rel = agent_speed - aligned_speed;
                        let obs_s0 = dq.dot(&u);
                        let t0 = wp0.time.as_secs_f64();
                        let obs_t0 = obs_t0.as_secs_f64();
                        if obs_s0 >= 0.0 {
                            let mut s = if obs_s0 <= follow_distance {
                                // Wait at the start until the obstacle has
                                // reached the follow distance
                                t += (follow_distance - obs_s0) / aligned_speed;
                                wait_hints.push(WaitHint {
                                    at_point: wp0.position,
                                    until: TimePoint::from_secs_f64(t),
                                    for_obstacle,
                                });
                                0.0
                            } else if obs_s0 - agent_speed * (t - t0) <= follow_distance {
                                let wait_dt = (obs_s0 - follow_distance) / agent_speed;
                                let s = if wait_dt >= 0.0 {
                                    // Move ahead just enough to come within the follow distance
                                    // of the obstacle's initial position.
                                    agent_speed * wait_dt
                                } else {
                                    // Wait at the start until the obstacle reaches
                                    // the follow distance
                                    t += (follow_distance - obs_s0) / aligned_speed;
                                    0.0
                                };
                                wait_hints.push(WaitHint {
                                    at_point: q + s*u,
                                    until: TimePoint::from_secs_f64(t),
                                    for_obstacle,
                                });
                                s
                            } else {
                                // Move ahead to the start position because the
                                // obstacle will not be a blocker
                                agent_speed * (t - t0)
                            };

                            let wait_interval = follow_distance / aligned_speed;
                            loop {
                                let obs_s = obs_s0 + aligned_speed * (t - obs_t0);
                                let ds = obs_s - s;
                                let delta_t = (ds - min_distance) / v_rel;
                                s += agent_speed * delta_t;
                                t += delta_t + wait_interval;
                                let reached_end = s >= s_max;
                                if reached_end {
                                    // A hint that would put us beyond the reach
                                    // limit is not a helpful one. Let's cap it
                                    // at the point where it reaches s_max.
                                    let t_backtrack = (s - s_max) / agent_speed;
                                    t -= t_backtrack;
                                    s = s_max;
                                }

                                let tp = TimePoint::from_secs_f64(t);
                                wait_hints.push(WaitHint {
                                    at_point: q + s*u,
                                    until: tp.min(obs_wp1.time),
                                    for_obstacle,
                                });

                                if tp >= obs_wp1.time || reached_end {
                                    // We don't need to follow any longer once
                                    // the obstacle vanishes.
                                    break;
                                }
                            }
                        }
                    }
                } else {
                    // The obstacle is nearly at a stand-still along the agent's
                    // path. We should approach its closest waypoint and then
                    // wait for it to vanish.
                    let mut times: ArrayVec<f64, 2> = ArrayVec::new();
                    let mut both_in_range = true;
                    for obs_q in [obs_q, obs_r] {
                        if let Some((t, _)) = compute_stationary_proximity(
                            obs_q, q, agent_v, min_distance_squared,
                        ) {
                            if t >= 0.0 {
                                // This point will be an obstruction
                                times.push(t);
                            }
                        } else {
                            // This point is not an obstruction, which means
                            // if the overall path is an obstruction then the
                            // earlier compute_quadratic_wait_hints search will
                            // find the right hint. We do not need to derive a
                            // hint from this branch.
                            both_in_range = false;
                        }
                    }

                    if both_in_range {
                        let t_wait_at = times.iter().min_by(
                            |a, b| a.partial_cmp(b).unwrap_or(Ordering::Less)
                        );
                        if let Some(t_wait_at) = t_wait_at {
                            let at_point = q + agent_v * *t_wait_at;
                            wait_hints.push(WaitHint {
                                at_point,
                                for_obstacle,
                                // Wait until this segment vanishes
                                until: obs_wp1.time,
                            });
                        }
                    }
                }
            }
        }
    }

    wait_hints
}

// TODO(@mxgrey): Split this into two functions because the first and second
// tuple elements are always used for different things.
#[inline]
fn compute_quadratic_wait_hints(
    q: Point,
    r: Point,
    u: Vector2,
    p0: Point,
    pf: Point,
    dt: f64,
    min_distance_squared: f64,
) -> Option<(f64, f64)> {
    let v = (pf - p0)/dt;
    let alpha = v - v.dot(&u) * u;
    let beta = p0 - (p0 - q).dot(&u) * u - q;
    let a = alpha.dot(&alpha);
    let b = 2.0 * alpha.dot(&beta);
    let c = beta.dot(&beta) - min_distance_squared;

    if a.abs() < 1e-8 {
        // There is virtually no motion for the obstacle, or the
        // obstacle is running almost parallel to the path which means
        // we cannot calculate a helpful waiting hint for this path.
        return None;
    }

    let radicand = b.powi(2) - 4.0 * a * c;
    if radicand < 0.0 {
        // The obstacle will never be close enough to the path to matter
        return None;
    }

    let t_begin = (-b - f64::sqrt(radicand)) / (2.0 * a);
    if dt < t_begin {
        // The collision would begin after the relevant range of the
        // obstacle
        return None;
    }

    // This segment for the obstacle stops being relevant beyond dt, so we'll
    // cap t_end at dt to prevent excessive waiting.
    let mut t_end = f64::min((-b + f64::sqrt(radicand)) / (2.0 * a), dt);
    if t_end < 0.0 {
        // The collision would end before the relevant range of the
        // obstacle
        return None;
    }

    // t_non_perpendicular is the time when the smallest vector between the
    // moving object and the reference line is no longer perpendicular to the
    // reference line and instead is the vector between the moving object and
    // the endpoint of the reference line.
    /*
     *               pf
     *              /
     *             /
     *            p(t_non_perpindicular)
     *           /|
     *          /||
     *  q------/--r
     *      ||/
     *      |/
     *      p0
     *
     */
    let t_non_perpendicular = (r - p0).dot(&u) / (v.dot(&u));
    // Checking past the perpendicular is only relevant when the object is
    // moving in the same direction as the reference line
    let same_direction = v.dot(&u) > 0.0;
    if t_non_perpendicular < t_end && same_direction {
        // Check if the proximity exit time relative to the endpoint is between
        // t_perpindicular and t_end. If so, that exit time should become the
        // new t_end.
        if let Some((_, t_leave_endpoint)) = compute_stationary_proximity(
            r, p0, v, min_distance_squared
        ) {
            if t_leave_endpoint < t_end {
                t_end = t_leave_endpoint;
            }
        }
    }

    Some((t_begin, t_end))
}

#[inline]
fn compute_stationary_proximity(
    r: Point,
    p0: Point,
    v: Vector2,
    min_distance_squared: f64,
) -> Option<(f64, f64)> {
    let p0_r = p0 - r;
    let a = v.dot(&v);
    let b = 2.0 * v.dot(&p0_r);
    let c = p0_r.dot(&p0_r) - min_distance_squared;

    if a.abs() >= 1e-8 {
        let radicand = b.powi(2) - 4.0 * a * c;
        if radicand >= 0.0 {
            let sqrt_radicand = f64::sqrt(radicand);
            let t_m = (-b - sqrt_radicand) / (2.0 * a);
            let t_p = (-b + sqrt_radicand) / (2.0 * a);
            return Some((t_m, t_p));
        }
    }

    None
}

#[derive(Clone, Copy)]
struct WaitHint {
    at_point: Point,
    until: TimePoint,
    for_obstacle: usize,
}

impl std::fmt::Debug for WaitHint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f
            .debug_struct("WaitHint")
            .field("at_point", &self.at_point)
            .field("until", &self.until.as_secs_f64())
            .field("for_obstacle", &self.for_obstacle)
            .finish()
    }
}

impl From<WaitHint> for WaypointR2 {
    fn from(value: WaitHint) -> Self {
        WaypointR2::new(value.until, value.at_point.x, value.at_point.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{DynamicEnvironment, CircularProfile};
    use approx::assert_relative_eq;

    fn add_to_env(
        in_environment: &mut DynamicEnvironment<WaypointR2>,
        profile: CircularProfile,
        (t0, x0, y0): (f64, f64, f64),
        (t1, x1, y1): (f64, f64, f64),
    ) {
        in_environment.obstacles.push(
            DynamicCircularObstacle::new(profile)
            .with_trajectory(Some(Trajectory::from_iter(
                [
                    WaypointR2::new(TimePoint::from_secs_f64(t0), x0, y0),
                    WaypointR2::new(TimePoint::from_secs_f64(t1), x1, y1),
                ]
            ).unwrap()))
        );
    }

    #[test]
    fn test_head_to_head_cross_over() {
        for footprint_radius in [
            0.001, 0.01, 0.1, 0.25, 0.5, 1.0, 1.1, 1.5,
        ] {
            let profile = CircularProfile::new(footprint_radius, 0.5, 1.0).unwrap();

            let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
            let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

            let y_obs = 1.0 + 2.0 * footprint_radius;
            let mut in_environment = DynamicEnvironment::new(profile);
            add_to_env(
                &mut in_environment, profile,
                (0.0, 10.0, -y_obs),
                (10.0, 0.0, y_obs),
            );

            let paths = compute_safe_linear_paths(from_point, to_point, &in_environment);
            assert_eq!(1, paths.len());

            let path = paths.first().unwrap();
            let wait_p = *path.first().unwrap().movement().unwrap();
            let line_a = (&from_point, &wait_p);
            assert!(is_safe_segment(line_a, None, &in_environment));

            let t_wait_until = path[1].wait_for().unwrap().time_estimate;
            let obs_traj = in_environment.obstacles.first().unwrap().trajectory().unwrap();
            let obs_p = obs_traj
                .initial_motion()
                .interpolate(obs_traj.finish_motion())
                .compute_position(&t_wait_until)
                .unwrap();

            assert_relative_eq!(
                (wait_p.position - obs_p).norm(),
                profile.critical_distance_for(&profile),
                max_relative=0.1,
            );
        }
    }

    #[test]
    fn test_follow_mid_vanish() {
        let footprint_radius = 0.5;
        let profile = CircularProfile::new(footprint_radius, 0.5, 1.0).unwrap();

        let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
        let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

        let mut in_environment = DynamicEnvironment::new(profile);

        let x_obs = 0.5 + 2.0*footprint_radius;
        add_to_env(
            &mut in_environment, profile,
            (0.0, x_obs, -1.0),
            (5.0, 2.5, 0.0)
        );

        let line_a = (&from_point, &to_point);
        assert!(!is_safe_segment(line_a, None, &in_environment));

        let paths = compute_safe_linear_paths(from_point, to_point, &in_environment);
        assert_eq!(1, paths.len());

        let path = paths.first().unwrap();

        // The agent should not wait longer than it takes for the obstacle to vanish
        let tf = path.last().unwrap().movement().unwrap().time;
        assert!(tf.as_secs_f64() < 15.0);
    }

    #[test]
    fn test_follow_cross_over() {
        let footprint_radius = 0.5;
        let profile = CircularProfile::new(footprint_radius, 0.5, 1.0).unwrap();

        let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
        let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

        let mut in_environment = DynamicEnvironment::new(profile);

        let x_obs = 0.5 + 2.0*footprint_radius;
        add_to_env(
            &mut in_environment, profile,
            (0.0, x_obs, -1.0),
            (40.0, 20.0, 0.0)
        );

        let line_a = (&from_point, &to_point);
        assert!(!is_safe_segment(line_a, None, &in_environment));

        let paths = compute_safe_linear_paths(from_point, to_point, &in_environment);
        assert!(paths.len() >= 1);
    }

    #[test]
    fn test_temporary_blockers() {
        let footprint_radius = 0.5;
        let profile = CircularProfile::new(footprint_radius, 0.5, 1.0).unwrap();

        for t0 in [0.0, -22.3, 3.5, -1015.7, 476.2] {
            let dt = 10.0;
            let tf = t0 + dt;
            for [x0, y0] in [
                [10.0, 0.0],
                [10.0, 10.0],
                [0.0, 10.0],
                [-10.0, 10.0],
                [-10.0, 0.0],
                [-10.0, -10.0],
                [0.0, -10.0],
                [10.0, -10.0],
            ] {
                let from_point = WaypointR2::new(TimePoint::from_secs_f64(t0), x0, y0);
                let to_point = WaypointR2::new(TimePoint::from_secs_f64(tf), 0.0, 0.0);
                let p0 = from_point.position;
                let p1 = to_point.position;
                let v = (p1 - p0).norm() / dt;

                let mut in_environment = DynamicEnvironment::new(profile);

                let obs_a = p0 + (p1 - p0) * 0.3;
                let obs_b = p0 + (p1 - p0) * 0.6;

                let t0_a = t0 + 0.3 * dt - 3.0;
                let tf_a = t0 + 0.3 * dt + 5.0;

                let t0_b = tf_a + 0.3 * dt - 3.0;
                let tf_b = tf_a + 0.3 * dt + 10.0;

                let t0_c = tf_a + 0.2 * dt;
                let tf_c = t0 + 100.0 * dt;

                add_to_env(
                    &mut in_environment, profile,
                    (t0_a, obs_a.x, obs_a.y),
                    (tf_a, obs_a.x, obs_a.y),
                );

                add_to_env(
                    &mut in_environment, profile,
                    (t0_b, obs_b.x, obs_b.y),
                    (tf_b, obs_b.x, obs_b.y),
                );

                add_to_env(
                    &mut in_environment, profile,
                    (t0_c, obs_a.x, obs_a.y),
                    (tf_c, obs_a.x, obs_a.y),
                );

                let line_a = (&from_point, &to_point);
                assert!(!is_safe_segment(line_a, None, &in_environment));

                let paths = compute_safe_linear_paths(from_point, to_point, &in_environment);
                assert_eq!(1, paths.len());

                let path = paths.first().unwrap();
                let wp_f = path.last().unwrap().movement().unwrap();
                assert_relative_eq!((wp_f.position - to_point.position).norm(), 0.0, epsilon=1e-3);
                assert_relative_eq!(wp_f.time.as_secs_f64(), tf_b + 0.4 * dt + 2.0*footprint_radius/v, epsilon=1e-3);
            }
        }
    }

    #[test]
    fn test_cycling_endpoint_blocker() {
        let profile = CircularProfile::new(0.25, 0.5, 1.0,).unwrap();

        for t0 in [0.0, -413.1, 17.6, -782.994, 4230.0] {
            let dt = 5.0;
            let tf = t0 + dt;
            for [x0, y0] in [
                [10.0, 0.0],
                [10.0, 10.0],
                [0.0, 10.0],
                [-10.0, 10.0],
                [-10.0, 0.0],
                [-10.0, -10.0],
                [0.0, -10.0],
                [10.0, -10.0],
            ] {
                let from_point = WaypointR2::new_f64(t0, x0, y0);
                let to_point = WaypointR2::new_f64(tf, 0.0, 0.0);

                let mut obs_wps = Vec::new();
                let n_cycles = 10;
                for cycle in 0..n_cycles {
                    for [t, x, y] in [
                        [0.0, 0.0, 0.0],
                        [5.0, 0.0, 5.0],
                        [10.0, 5.0, 5.0],
                        [15.0, 5.0, 0.0],
                    ] {
                        obs_wps.push(WaypointR2::new_f64(
                            20.0*(cycle as f64) + t + t0, x, y
                        ));
                    }
                }
                obs_wps.push(WaypointR2::new_f64(
                    20.0*(n_cycles as f64) + t0, 0.0, 0.0
                ));
                // Pause a moment on the destination for the very last waypoint
                obs_wps.push(WaypointR2::new_f64(
                    20.0*(n_cycles as f64) + 10.0 + t0, 0.0, 0.0
                ));

                let obs_traj = Trajectory::from_iter(obs_wps).unwrap();
                let mut in_environment = DynamicEnvironment::new(profile);
                in_environment.obstacles.push(
                    DynamicCircularObstacle::new(profile)
                    .with_trajectory(Some(obs_traj.clone()))
                );

                let paths = compute_safe_linear_paths(
                    from_point, to_point, &in_environment
                );
                assert_eq!(11, paths.len());

                for i in 0..n_cycles {
                    let tf = paths[i].last().unwrap().movement().unwrap().time.as_secs_f64();
                    let i = i as f64;
                    assert!(20.0*i + t0 < tf);
                    assert!(tf < 20.0*(i+1.0) + t0);
                }

                let tf = paths.last().unwrap().last().unwrap().movement().unwrap().time.as_secs_f64();
                let obs_tf = 20.0*(n_cycles as f64) + 10.0 + t0;
                assert!(obs_tf < tf);
                assert!(tf < obs_tf + 1.0);
            }
        }
    }
}
