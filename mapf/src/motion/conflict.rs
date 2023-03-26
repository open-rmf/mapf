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
        r2::{Waypoint as WaypointR2, Point},
    },
};
use smallvec::SmallVec;
use arrayvec::ArrayVec;
use std::cmp::Ordering;

type Vector2 = nalgebra::Vector2<f64>;

#[derive(Debug, Clone, Copy)]
pub struct Profile {
    /// Radius that encompasses the physical footprint of the robot
    footprint_radius: f64,
    /// Distance that the agent should try to keep its footprint away from the
    /// location where the footprint of an obstacle will collide with it. When
    /// two agents in contention have different `safety_distance` values, the
    /// larger value will be used by both.
    safety_distance: f64,

    /// When an agent is following an obstacle or another agent (the dot
    /// product of their velocities is positive), the agent's movements should
    /// be broken into segments of this size
    follow_distance: f64,
}

impl Profile {

    pub fn new(
        footprint_radius: f64,
        safety_distance: f64,
        follow_distance: f64,
    ) -> Result<Self, ()> {
        if footprint_radius < 0.0 || safety_distance < 0.0 || follow_distance < 0.0 {
            return Err(());
        }

        Ok(Self { footprint_radius, safety_distance, follow_distance })
    }

    pub fn with_footprint_radius(mut self, footprint_radius: f64) -> Result<Self, ()> {
        if footprint_radius < 0.0 {
            return Err(());
        }
        self.footprint_radius = footprint_radius;
        Ok(self)
    }

    pub fn with_safety_distance(mut self, safety_distance: f64) -> Result<Self, ()> {
        if safety_distance < 0.0 {
            return Err(());
        }
        self.safety_distance = safety_distance;
        Ok(self)
    }

    pub fn with_follow_distance(mut self, follow_distance: f64) -> Result<Self, ()> {
        if follow_distance < 0.0 {
            return Err(());
        }
        self.follow_distance = follow_distance;
        Ok(self)
    }

    /// The critical distance is the distance between two traffic participants
    /// where they must not approach each other any closer. Use this value when
    /// calculating an acceptable stopping location for an agent.
    ///
    /// See also [`Profile::conflict_distance_for`]
    pub fn critical_distance_for(&self, other: &Profile) -> f64 {
        let d = self.footprint_radius + other.footprint_radius;
        f64::max(d, 1e-3)
    }

    /// When two traffic participants are at or within this distance, then we
    /// will consider them to be in-conflict if they move any closer towards
    /// each other.
    ///
    /// See also [`Profile::critical_distance_for`]
    pub fn conflict_distance_for(&self, other: &Profile) -> f64 {
        let d = self.footprint_radius + other.footprint_radius;
        f64::max(d - 1e-3, 0.0)
    }

    pub fn critical_distance_squared_for(&self, other: &Profile) -> f64 {
        self.critical_distance_for(other).powi(2)
    }

    pub fn conflict_distance_squared_for(&self, other: &Profile) -> f64 {
        self.conflict_distance_for(other).powi(2)
    }

    pub fn safety_distance_for(&self, other: &Profile) -> f64 {
        f64::max(self.safety_distance, other.safety_distance)
    }

    pub fn follow_distance_for(&self, other: &Profile) -> f64 {
        f64::max(self.follow_distance, other.follow_distance)
    }
}

pub struct DynamicEnvironment<W: Waypoint> {
    pub obstacles: Vec<DynamicObstacle<W>>,
}

impl<W: Waypoint> DynamicEnvironment<W> {
    pub fn new() -> Self {
        Self { obstacles: Vec::new() }
    }
}

impl<W: Waypoint> Default for DynamicEnvironment<W> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct DynamicObstacle<W: Waypoint> {
    profile: Profile,
    trajectory: Option<Trajectory<W>>,
    bounding_box: Option<BoundingBox>,
}

impl<W: Waypoint + Into<WaypointR2>> DynamicObstacle<W> {
    pub fn new(profile: Profile) -> Self {
        Self {
            profile,
            trajectory: None,
            bounding_box: None,
        }
    }

    pub fn with_trajectory(self, trajectory: Option<Trajectory<W>>) -> Self {
        Self {
            bounding_box: trajectory.as_ref().map(
                |t| BoundingBox::for_trajectory(&self.profile, t)
            ),
            profile: self.profile,
            trajectory,
        }
    }

    pub fn profile(&self) -> &Profile {
        &self.profile
    }

    pub fn set_profile(&mut self, profile: Profile) {
        self.profile = profile;
        self.bounding_box = self.trajectory.as_ref().map(
            |t| BoundingBox::for_trajectory(&self.profile, t)
        );
    }

    pub fn trajectory(&self) -> Option<&Trajectory<W>> {
        self.trajectory.as_ref()
    }

    pub fn set_trajectory(&mut self, trajectory: Option<Trajectory<W>>) {
        self.trajectory = trajectory;
        self.bounding_box = self.trajectory.as_ref().map(
            |t| BoundingBox::for_trajectory(&self.profile, t)
        );
    }
}

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

    pub fn wait_for(&self) -> Option<&WaitFor> {
        match self {
            Self::Wait(w) => Some(w),
            _ => None,
        }
    }
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

pub fn compute_safe_linear_paths<W>(
    profile: &Profile,
    from_point: WaypointR2,
    to_point: WaypointR2,
    in_environment: &DynamicEnvironment<W>,
) -> SmallVec<[SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 3]>; 3]>
where
    W: Into<WaypointR2> + Waypoint,
{
    let delta_t = (to_point.time - from_point.time).as_secs_f64();
    assert!(delta_t >= 0.0);
    let bb = BoundingBox::for_line(profile, &from_point, &to_point);
    if delta_t < 1e-8 {
        // This very small time interval suggests that the agent is not moving
        // significantly. Just check if the proposed path is safe or not.
        if is_safe_segment(profile, (&from_point, &to_point), Some(bb), in_environment) {
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

    let wait_hints = compute_wait_hints(
        profile,
        (&from_point, &to_point),
        &bb,
        in_environment
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
        let contour = if reach > 1e-6 {
            Duration::from_secs_f64(wait_t - speed / reach)
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

    dbg!(&ranked_hints);
    dbg!(ranked_hints.len());

    dbg!();
    dbg!(compute_safe_arrival_times(profile, to_point, in_environment))
        .into_iter()
        .filter_map(|arrival_time|
            compute_safe_arrival_path(
                profile,
                from_point,
                to_point,
                arrival_time,
                &ranked_hints,
                in_environment,
            )
        ).collect()
}

pub fn compute_safe_arrival_times<W>(
    profile: &Profile,
    for_point: WaypointR2,
    in_environment: &DynamicEnvironment<W>,
) -> SmallVec<[TimePoint; 3]>
where
    W: Into<WaypointR2> + Waypoint,
{
    let mut safe_arrival_times = SmallVec::<[TimePoint; 3]>::new();

    // First find every safe arrival time above from_point.time
    let mut candidate_time = for_point.time;
    loop {
        dbg!(candidate_time);
        let mut any_pushed = true;
        while any_pushed {
            dbg!(candidate_time);
            any_pushed = false;
            for obs in &in_environment.obstacles {
                let obs_traj = match &obs.trajectory {
                    Some(r) => r,
                    None => continue,
                };

                let critical_distance_squared = profile.critical_distance_squared_for(&obs.profile);
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
                    candidate_time = dbg!(new_candidate_time);
                    any_pushed = true;
                } else {
                    // This obstacle is violated and no end could be
                    // found for the violation, so there will never be
                    // another safe arrival time.
                    dbg!();
                    return safe_arrival_times;
                }
            }
        }

        // We found a new safe arrival time
        safe_arrival_times.push(candidate_time);

        // Look for the next soonest candidate time
        let mut next_candidate_time = None;
        for obs in &in_environment.obstacles {
            let min_distance_squared = profile.critical_distance_squared_for(&obs.profile);

            let obs_traj = match &obs.trajectory {
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
                    if dbg!(check < *next_candidate_time) {
                        *next_candidate_time = check;
                    }
                } else {
                    dbg!();
                    next_candidate_time = Some(check);
                }
            }
        }

        candidate_time = match next_candidate_time {
            Some(t) => dbg!(t),
            // There are no more candidate times to consider
            None => { dbg!(); break },
        };
    }

    safe_arrival_times
}

#[inline]
fn compute_safe_arrival_path<W>(
    profile: &Profile,
    from_point: WaypointR2,
    to_point: WaypointR2,
    arrival_time: TimePoint,
    ranked_hints: &SmallVec<[RankedHint; 16]>,
    in_environment: &DynamicEnvironment<W>,
) -> Option<SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 3]>>
where
    W: Into<WaypointR2> + Waypoint,
{
    dbg!(arrival_time);
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

        if !is_safe_segment(profile, (&from_p, &arrival_wp), None, in_environment) {
            return false;
        }

        if arrival_wp.time < arrival_time {
            let final_wp = arrival_wp.with_time(arrival_time);
            return is_safe_segment(profile, (&arrival_wp, &final_wp), None, in_environment);
        }

        true
    };

    let make_hint_arrival = |previous_wp: WaypointR2, hint_wp_end: WaypointR2| -> WaypointR2 {
        let hint_p = hint_wp_end.position;
        let dt_hint_arrival = (hint_p - previous_wp.position).norm() / speed;
        let t_hint_arrival = previous_wp.time + Duration::from_secs_f64(dt_hint_arrival);
        hint_wp_end.with_time(t_hint_arrival)
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

    let mut search: SmallVec<[(usize, Option<usize>); 16]> = SmallVec::new();
    search.push((0, None));
    while let Some((consider, next)) = dbg!(search.pop()) {
        dbg!((arrival_time.as_secs_f64(), ranked_hints.len(), &search));
        let mut consider_next = if let Some(next) = next {
            next + 1
        } else {
            // Test if we can safely reach the point being considered from the
            // previous point.
            let previous_wp = match dbg!(search.last()) {
                Some((previous_index, _)) => {
                    ranked_hints.get(*previous_index).unwrap().hint.into()
                }
                None => from_point,
            };

            dbg!((consider, ranked_hints.len()));
            println!("Testing if arrival is safe {:?}", ranked_hints.get(consider).unwrap().hint);
            let hint_wp_end: WaypointR2 = ranked_hints.get(consider).unwrap().hint.into();
            let hint_wp_start = make_hint_arrival(previous_wp, hint_wp_end);
            dbg!((hint_wp_start, hint_wp_end));
            if hint_wp_end.time < hint_wp_start.time {
                // If the end time of the hint is earlier than we would arrive
                // then it's not actually a helpful hint.
                if consider < ranked_hints.len() {
                    search.push(dbg!((consider+1, None)));
                }
                continue;
            }

            dbg!((previous_wp, hint_wp_start));
            let safe_0 = dbg!(is_safe_segment(
                profile, (&previous_wp, &hint_wp_start), None, in_environment
            ));
            dbg!();
            let safe_1 = dbg!(is_safe_segment(
                profile, (&hint_wp_start, &hint_wp_end), None, in_environment
            ));
            let safe_hint_arrival = safe_0 && safe_1;

            dbg!(safe_hint_arrival);
            if !safe_hint_arrival {
                // We cannot safely arrive at this hint from the previous
                // waypoint. We will prune this part of the search.
                if consider < ranked_hints.len() {
                    search.push(dbg!((consider+1, None)));
                }
                continue;
            }

            if can_arrive_safely(hint_wp_end) {
                dbg!();
                // We can safely arrive at the goal from this hint waypoint, so
                // we no longer need to search for a path.

                // Put the final waypoint back into the search vector to
                // simplify the construction of the path.
                search.push((consider, None));

                let mut path: SmallVec<[SafeAction<_, _>; 3]> = SmallVec::new();
                let mut previous_wp = from_point;
                for (hint_id, _) in &search {
                    let hint = ranked_hints.get(*hint_id).unwrap().hint;
                    let hint_wp_end = hint.into();
                    let hint_wp_start = make_hint_arrival(previous_wp, hint_wp_end);
                    path.push(SafeAction::Move(hint_wp_start));
                    path.push(SafeAction::Wait(WaitForObstacle {
                        for_obstacle: hint.for_obstacle,
                        time_estimate: hint.until,
                    }));

                    previous_wp = hint_wp_end;
                }

                dbg!(previous_wp);
                let final_wp = make_hint_arrival(previous_wp, to_point);
                path.push(SafeAction::Move(final_wp));

                return Some(path);
            }

            consider + 1
        };

        let pushed_child = 'pushed_child: {
            while let (Some(prev_hint), Some(next_hint)) = (
                ranked_hints.get(consider), ranked_hints.get(consider_next)
            ) {
                dbg!((prev_hint, next_hint));
                dbg!((prev_hint.reach, next_hint.reach));
                if prev_hint.reach <= next_hint.reach {
                    // We need to always be moving towards hints that reach further
                    // towards the goal. We do not support backtracking.
                    search.push((consider, Some(consider_next)));
                    search.push((consider_next, None));
                    break 'pushed_child true;
                }

                consider_next += 1;
            }

            false
        };

        if !pushed_child && ((consider + 1) < ranked_hints.len()) {
            dbg!();
            search.push((consider+1, None));
        }
    }

    None
}

#[derive(Debug)]
struct RankedHint {
    contour: Duration,
    reach: f64,
    hint: WaitHint,
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
    dbg!((line_a, line_b));
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
                dbg!();
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
    dbg!((a, b, c));

    if a.abs() < 1e-8 {
        // dbg!();
        // There is virtually no relative motion
        assert!(b.abs() < 1e-4);
        if c <= 0.0 {
            // The objects are within the proximity limit
            // dbg!();
            return Proximity {
                enter: Some(t_range.0),
                // The objects are not moving relative to each other so they
                // cannot exit
                exit: None,
            };
        } else {
            // The objects are not within proximity
            dbg!();
            return Proximity::none();
        }
    }

    let radicand = b.powi(2) - 4.0 * a * c;
    if radicand < 0.0 {
        // The objects are moving relative to each other, but they will never come
        // close enough to each other.
        dbg!(radicand);
        return Proximity { enter: None, exit: None };
    }

    let sqrt_radicand = radicand.sqrt();
    let dt = (t_range.1 - t_range.0).as_secs_f64();
    let t_m = (-b - sqrt_radicand) / (2.0 * a);
    let t_p = (-b + sqrt_radicand) / (2.0 * a);
    dbg!((t_m, t_p));
    assert!(t_m <= t_p);

    if t_p < 0.0 {
        // dbg!(t_p);
        // The intersection happens before the time range
        return Proximity::none();
    }

    if dt < t_m {
        // dbg!(t_m);
        // The intersection happens after the time range
        return Proximity::none();
    }

    // dbg!();
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
    dbg!(candidate_time.as_secs_f64());
    let mut this_obs_violated = false;
    let mut new_candidate_time: Option<TimePoint> = None;
    for [wp0, wp1] in obs_traj.iter_from(candidate_time).pairs() {
        dbg!((candidate_time, wp0.time(), wp1.time()));
        if candidate_time < *wp0.time() && !this_obs_violated {
            dbg!();
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

        dbg!(&proximity);

        if this_obs_violated {
            // Look for a time where the footprints are no longer
            // overlapping
            let exit = match proximity.exit {
                Some(exit) => exit,
                None => {dbg!(); continue }
            };

            // If everything is working correctly, it should not be
            // possible for this exit time to be less than the
            // candidate time (but we add some buffer to accommodate
            // floating point error).
            assert!(exit + time_point::Duration::new(10000) >= candidate_time);
            new_candidate_time = dbg!(Some(exit.max(candidate_time)));
            break;
        } else {
            let enter = match proximity.enter {
                Some(enter) => enter,
                None => { dbg!(); continue; }
            };

            dbg!(enter.as_secs_f64());
            if enter > candidate_time {
                // The violation happens after the candidate time
                // so we can safely ignore it. It is also not
                // possible for any more trajectory segments to
                // violate the target at the candidate time.
                dbg!();
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
                new_candidate_time = dbg!(Some(exit));
                break;
            }

            if exit <= candidate_time {
                // The violation was relieved before the candidate
                // time so it is not a relevant violation.
                dbg!();
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
    dbg!();
    for [wp0, wp1] in obs_traj.iter_from(previous_candidate_time).pairs() {
        dbg!();
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

        if let Some(exit) = dbg!(proximity.exit) {
            dbg!((previous_candidate_time, exit));
            if previous_candidate_time < exit {
                dbg!();
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
fn is_safe_segment<W>(
    profile: &Profile,
    line_a: (&WaypointR2, &WaypointR2),
    bb: Option<BoundingBox>,
    in_environment: &DynamicEnvironment<W>,
) -> bool
where
    W: Into<WaypointR2> + Waypoint,
{
    let bb = match bb {
        Some(bb) => bb,
        None => BoundingBox::for_line(profile, line_a.0, line_a.1),
    };
    dbg!(line_a);

    for obs in &in_environment.obstacles {
        let conflict_distance_squared = profile.conflict_distance_squared_for(&obs.profile);
        // dbg!(min_distance_squared);
        let obs_traj = match &obs.trajectory {
            Some(r) => r,
            None => continue,
        };

        // dbg!();
        if !bb.overlaps(obs.bounding_box) {
            dbg!();
            continue;
        }

        // dbg!((line_a.0.time, line_a.1.time));
        for [wp0_b, wp1_b] in obs_traj.iter_range(line_a.0.time, line_a.1.time).pairs() {
            dbg!((&wp0_b, &wp1_b));
            if line_a.1.time < *wp0_b.time() {
                dbg!();
                // The trajectories are no longer overlapping in time so there
                // is no longer a risk.
                return true;
            }

            let wp0_b: WaypointR2 = wp0_b.into();
            let wp1_b: WaypointR2 = wp1_b.into();
            let line_b = (&wp0_b, &wp1_b);
            dbg!(wp0_b, wp1_b);

            if !bb.overlaps(Some(
                BoundingBox::for_line(&obs.profile, &wp0_b, &wp1_b)
                .inflated_by(1e-3)
            )) {
                continue;
            }

            let in_time_range = |t: &TimePoint| -> bool {
                line_a.0.time < *t && *t < line_a.1.time
                && line_b.0.time < *t && *t < line_b.1.time
            };

            dbg!();
            let proximity = detect_proximity(
                conflict_distance_squared,
                line_a,
                line_b,
            );
            dbg!();

            dbg!(proximity);

            if let Some(t) = proximity.enter.filter(in_time_range) {
                let t_range = compute_t_range(line_a, line_b);
                let t = (t - t_range.0).as_secs_f64();
                let (dp0, dv) = compute_dp0_dv(line_a, line_b, &t_range);
                let a = dv.dot(&dv);
                let b = 2.0 * dv.dot(&dp0);
                let deriv = 2.0*a * t + b;
                dbg!(deriv);
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
fn compute_wait_hints<W>(
    profile: &Profile,
    (wp0, wp1): (&WaypointR2, &WaypointR2),
    bb: &BoundingBox,
    in_environment: &DynamicEnvironment<W>,
) -> SmallVec<[WaitHint; 16]>
where
    W: Into<WaypointR2> + Waypoint,
{
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
    for (for_obstacle, obs) in in_environment.obstacles.iter().enumerate() {
        if !bb.overlaps(obs.bounding_box) {
            continue;
        }

        let obs_traj = match &obs.trajectory {
            Some(t) => t,
            None => continue,
        };

        let min_distance = profile.critical_distance_for(&obs.profile);
        let min_distance_squared = min_distance.powi(2);
        // dbg!(min_distance_squared);
        for [obs_wp0, obs_wp1] in obs_traj.iter().pairs() {
            let obs_wp0: WaypointR2 = obs_wp0.into();
            let obs_wp1: WaypointR2 = obs_wp1.into();
            if !bb.overlaps(Some(
                BoundingBox::for_line(&obs.profile, &obs_wp0, &obs_wp1)
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
                dbg!(t_end);
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
                    if t_wait_at >= 0.0 {
                        let at_point = q + agent_v * t_wait_at;
                        wait_hints.push(WaitHint {
                            at_point,
                            for_obstacle,
                            until: obs_t0 + Duration::from_secs_f64(t_end),
                        });
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
                        (dq, 0.0)
                    } else {
                        let dq = obs_q - q;
                        let t = (obs_wp0.time - wp0.time).as_secs_f64();
                        (dq, t)
                    };

                    if dq.dot(&dq) >= min_distance_squared {
                        let follow_distance = profile.follow_distance_for(&obs.profile);
                        let v_rel = agent_speed - aligned_speed;
                        let obs_s0 = dq.dot(&u);
                        if obs_s0 >= 0.0 {
                            let mut s = if obs_s0 <= min_distance + follow_distance {
                                // Wait at the start until the obstacle has
                                // reached the follow distance
                                t += (min_distance + follow_distance - obs_s0) / aligned_speed;
                                wait_hints.push(dbg!(WaitHint {
                                    at_point: wp0.position,
                                    until: wp0.time + Duration::from_secs_f64(t),
                                    for_obstacle,
                                }));
                                0.0
                            } else if obs_s0 - agent_speed * t <= min_distance + follow_distance {
                                let wait_t = (obs_s0 - min_distance - follow_distance) / agent_speed;
                                let s = if wait_t >= 0.0 {
                                    // Move ahead just enough to come within the follow distance
                                    // of the obstacle's initial position.
                                    agent_speed * wait_t
                                } else {
                                    // Wait at the start until the obstacle reaches
                                    // the follow distance
                                    t += (min_distance + follow_distance - obs_s0) / aligned_speed;
                                    0.0
                                };
                                wait_hints.push(dbg!(WaitHint {
                                    at_point: q + s*u,
                                    until: wp0.time + Duration::from_secs_f64(t),
                                    for_obstacle,
                                }));
                                s
                            } else {
                                // Move ahead to the start position because the
                                // obstacle will not be a blocker
                                agent_speed * t
                            };

                            let wait_interval = (min_distance + follow_distance) / aligned_speed;
                            while s < s_max {
                                let obs_s = obs_s0 + aligned_speed * t;
                                let ds = obs_s - s;
                                let delta_t = (ds - min_distance) / v_rel;
                                s += agent_speed * delta_t;
                                t += delta_t + wait_interval;
                                let t_abs = wp0.time + Duration::from_secs_f64(t);
                                wait_hints.push(dbg!(WaitHint {
                                    at_point: q + s*u,
                                    until: t_abs.min(obs_wp1.time),
                                    for_obstacle,
                                }));

                                if t_abs >= obs_wp1.time {
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
                    dbg!((obs_q, obs_r));
                    let mut times: ArrayVec<f64, 2> = ArrayVec::new();
                    let mut both_in_range = true;
                    for obs_q in [obs_q, obs_r] {
                        if let Some((t, _)) = dbg!(compute_stationary_proximity(
                            obs_q, q, agent_v, min_distance_squared,
                        )) {
                            dbg!(t);
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
                            wait_hints.push(dbg!(WaitHint {
                                at_point,
                                for_obstacle,
                                // Wait until this segment vanishes
                                until: obs_wp1.time,
                            }));
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

#[derive(Debug, Clone, Copy)]
struct BoundingBox {
    min: Vector2,
    max: Vector2,
}

impl BoundingBox {
    fn overlaps(&self, other: Option<BoundingBox>) -> bool {
        let other = match other {
            Some(b) => b,
            None => return false,
        };
        if other.max.x < self.min.x {
            return false;
        }
        if other.max.y < self.min.y {
            return false;
        }
        if self.max.x < other.min.x {
            return false;
        }
        if self.max.y < other.min.y {
            return false;
        }
        return true;
    }

    fn for_point(p: Point) -> Self {
        Self {
            min: p.coords,
            max: p.coords,
        }
    }

    fn for_line(profile: &Profile, wp0: &WaypointR2, wp1: &WaypointR2) -> Self {
        Self::for_point(wp0.position)
        .incorporating(wp1.position)
        .inflated_by(profile.footprint_radius)
    }

    fn for_trajectory<W>(profile: &Profile, trajectory: &Trajectory<W>) -> Self
    where
        W: Into<WaypointR2> + Waypoint,
    {
        let initial_bb = BoundingBox::for_point(
            trajectory.initial_motion().clone().into().position
        );

        trajectory
        .iter()
        .fold(initial_bb, |b: Self, p| b.incorporating(p.into().position))
        .inflated_by(profile.footprint_radius)
    }

    fn incorporating(self, p: Point) -> Self {
        Self {
            min: Vector2::new(
                f64::min(self.min.x, p.x),
                f64::min(self.min.y, p.y),
            ),
            max: Vector2::new(
                f64::max(self.max.x, p.x),
                f64::max(self.max.y, p.y),
            ),
        }
    }

    fn inflated_by(self, r: f64) -> Self {
        Self {
            min: self.min - Vector2::from_element(r),
            max: self.max + Vector2::from_element(r),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn add_to_env(
        in_environment: &mut DynamicEnvironment<WaypointR2>,
        profile: Profile,
        (t0, x0, y0): (f64, f64, f64),
        (t1, x1, y1): (f64, f64, f64),
    ) {
        in_environment.obstacles.push(
            DynamicObstacle::new(profile)
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
            let profile = Profile {
                footprint_radius,
                safety_distance: 0.5,
                follow_distance: 1.0,
            };

            let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
            let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

            let y_obs = 1.0 + 2.0 * footprint_radius;
            let mut in_environment = DynamicEnvironment::new();
            add_to_env(
                &mut in_environment, profile,
                (0.0, 10.0, -y_obs),
                (10.0, 0.0, y_obs),
            );

            let paths = compute_safe_linear_paths(&profile, from_point, to_point, &in_environment);
            assert!(paths.len() == 1);

            let path = paths.first().unwrap();
            let wait_p = *path.first().unwrap().movement().unwrap();
            let line_a = (&from_point, &wait_p);
            assert!(is_safe_segment(&profile, line_a, None, &in_environment));

            let t_wait_until = path[1].wait_for().unwrap().time_estimate;
            let obs_traj = in_environment.obstacles.first().unwrap().trajectory.as_ref().unwrap();
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
        let profile = Profile {
            footprint_radius,
            safety_distance: 0.5,
            follow_distance: 1.0,
        };

        let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
        let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

        let mut in_environment = DynamicEnvironment::new();

        let x_obs = 0.5 + 2.0*footprint_radius;
        add_to_env(
            &mut in_environment, profile,
            (0.0, x_obs, -1.0),
            (5.0, 2.5, 0.0)
        );

        let line_a = (&from_point, &to_point);
        assert!(!is_safe_segment(&profile, line_a, None, &in_environment));

        let paths = compute_safe_linear_paths(&profile, from_point, to_point, &in_environment);
        assert!(paths.len() == 1);

        let path = paths.first().unwrap();

        // The agent should not wait longer than it takes for the obstacle to vanish
        let tf = path.last().unwrap().movement().unwrap().time;
        assert!(tf.as_secs_f64() < 15.0);
    }

    #[test]
    fn test_follow_cross_over() {
        let footprint_radius = 0.5;
        let profile = Profile {
            footprint_radius,
            safety_distance: 0.5,
            follow_distance: 1.0,
        };

        let from_point = WaypointR2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0);
        let to_point = WaypointR2::new(TimePoint::from_secs_f64(10.0), 10.0, 0.0);

        let mut in_environment = DynamicEnvironment::new();

        let x_obs = 0.5 + 2.0*footprint_radius;
        add_to_env(
            &mut in_environment, profile,
            (0.0, x_obs, -1.0),
            (40.0, 20.0, 0.0)
        );

        let line_a = (&from_point, &to_point);
        assert!(!is_safe_segment(&profile, line_a, None, &in_environment));

        let paths = compute_safe_linear_paths(&profile, from_point, to_point, &in_environment);
        assert!(paths.len() >= 1);

        let path = paths.first().unwrap();
        println!("{path:?}");

        let t_wait_until = path[1].wait_for().unwrap().time_estimate;
        let obs_traj = in_environment.obstacles.first().unwrap().trajectory.as_ref().unwrap();
        let obs_p = obs_traj
            .initial_motion()
            .interpolate(obs_traj.finish_motion())
            .compute_position(&t_wait_until)
            .unwrap();

        dbg!((obs_p - to_point.position).norm());
    }

    #[test]
    fn test_temporary_blockers() {
        let footprint_radius = 0.5;
        let profile = Profile {
            footprint_radius,
            safety_distance: 0.5,
            follow_distance: 1.0,
        };

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
                dbg!((t0, x0, y0));
                let from_point = WaypointR2::new(TimePoint::from_secs_f64(t0), x0, y0);
                let to_point = WaypointR2::new(TimePoint::from_secs_f64(tf), 0.0, 0.0);
                let p0 = from_point.position;
                let p1 = to_point.position;
                let v = (p1 - p0).norm() / dt;

                let mut in_environment = DynamicEnvironment::new();

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
                assert!(!is_safe_segment(&profile, line_a, None, &in_environment));

                let paths = compute_safe_linear_paths(&profile, from_point, to_point, &in_environment);
                println!("paths: {paths:?}");
                assert!(paths.len() == 1);

                let path = paths.first().unwrap();
                let wp_f = path.last().unwrap().movement().unwrap();
                assert_relative_eq!((wp_f.position - to_point.position).norm(), 0.0, epsilon=1e-3);
                assert_relative_eq!(wp_f.time.as_secs_f64(), tf_b + 0.4 * dt + 2.0*footprint_radius/v, epsilon=1e-3);
            }
        }
    }

    #[test]
    fn test_cycling_endpoint_blocker() {
        let profile = Profile {
            footprint_radius: 0.25,
            safety_distance: 0.5,
            follow_distance: 1.0,
        };

        for t0 in [0.0, /*-43.1, 17.6, -782.994, 4230.0*/] {
            let dt = 5.0;
            let tf = t0 + dt;
            for [x0, y0] in [
                // [10.0, 0.0],
                // [10.0, 10.0],
                // [0.0, 10.0],
                // [-10.0, 10.0],
                [-10.0, 0.0],
                // [-10.0, -10.0],
                // [0.0, -10.0],
                // [10.0, -10.0],
            ] {
                dbg!((t0, x0, y0));
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
                            dbg!(20.0*(dbg!(cycle) as f64) + t), x, y
                        ));
                    }
                }
                obs_wps.push(WaypointR2::new_f64(
                    20.0*(n_cycles as f64), 0.0, 0.0
                ));
                // Pause a moment on the destination for the very last waypoint
                obs_wps.push(WaypointR2::new_f64(
                    20.0*(n_cycles as f64) + 10.0, 0.0, 0.0
                ));

                let obs_traj = Trajectory::from_iter(obs_wps).unwrap();
                let mut in_environment = DynamicEnvironment::new();
                in_environment.obstacles.push(
                    DynamicObstacle::new(profile)
                    .with_trajectory(Some(obs_traj.clone()))
                );

                let paths = compute_safe_linear_paths(
                    &profile, from_point, to_point, &in_environment
                );
                dbg!(&paths);
                dbg!(paths.len());
            }
        }
    }
}
