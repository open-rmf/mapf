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
        r2::Waypoint as WaypointR2,
    },
};
use smallvec::SmallVec;

type Vector2 = nalgebra::Vector2<f64>;

pub struct Profile {
    /// Radius that encompasses the physical footprint of the robot
    footprint_radius: f64,
    /// Distance that the agent should try to keep its footprint away from the
    /// location where the footprint of an obstacle will collide with it. When
    /// two agents in contention have different `safety_distance` values, the
    /// larger value will be used by both.
    safety_distance: f64,

    // TODO(@mxgrey): Think about how to use this
    // /// When an agent is following an obstacle or another agent (the dot
    // /// product of their velocities is positive), the agent's movements should
    // /// be broken into segments of this size
    // follow_distance: f64,
}

impl Profile {
    pub fn union(&self, other: &Profile) -> Profile {
        Profile {
            footprint_radius: self.footprint_radius + other.footprint_radius,
            safety_distance: self.safety_distance.max(other.safety_distance),
            // follow_distance: f64::max(self.follow_distance, other.follow_distance),
        }
    }
}

pub struct DynamicEnvironment<W: Waypoint> {
    obstacles: Vec<DynamicObstacle<W>>,
}

pub struct DynamicObstacle<W: Waypoint> {
    profile: Profile,
    trajectory: Option<Trajectory<W>>,
    // TODO(@mxgrey): Include a bounding box for the trajectory to allow
    // broad phase collision detection
}

pub enum SafeAction<Movement, WaitFor> {
    /// Move to the designated point
    Move(Movement),
    /// Wait for some condition to be met
    Wait(WaitFor),
}

pub struct WaitForObstacle {
    /// Which obstacle needs to be waited on
    for_obstacle: usize,
    /// How long is it necessary to wait
    time_estimate: TimePoint,
}

pub fn compute_safe_linear_paths<W>(
    profile: &Profile,
    from_point: WaypointR2,
    to_point: WaypointR2,
    in_environment: &DynamicEnvironment<W>,
) -> SmallVec<[SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 3]>; 1]>
where
    W: Into<WaypointR2> + Waypoint,
{
    let delta_t = (to_point.time - from_point.time).as_secs_f64();
    assert!(delta_t >= 0.0);
    if delta_t < 1e-8 {
        // This very small time interval suggests that the agent is not moving
        // significantly. Just check if the proposed path is safe or not.
        if is_safe_segment(profile, (&from_point, &to_point), in_environment) {
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

    compute_safe_arrival_times(profile, to_point, in_environment)
        .into_iter()
        .filter_map(|arrival_time|
            compute_safe_arrival_path(
                from_point, to_point, arrival_time, in_environment
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
        let mut any_pushed = true;
        while any_pushed {
            any_pushed = false;
            for obs in &in_environment.obstacles {
                let obs_traj = match &obs.trajectory {
                    Some(r) => r,
                    None => continue,
                };

                let acceptable_distance_squared = (
                    profile.footprint_radius + obs.profile.footprint_radius
                ).powi(2);
                let adjustment = adjust_candidate_time(
                    for_point,
                    candidate_time,
                    obs_traj,
                    acceptable_distance_squared
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
        for obs in &in_environment.obstacles {
            let acceptable_distance_squared = (profile.footprint_radius + obs.profile.footprint_radius).powi(2);

            let obs_traj = match &obs.trajectory {
                Some(r) => r,
                None => continue,
            };

            if let Some(check) = find_next_candidate_time(
                candidate_time,
                for_point,
                obs_traj,
                acceptable_distance_squared,
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
fn compute_safe_arrival_path<W>(
    from_point: WaypointR2,
    to_point: WaypointR2,
    arrival_time: TimePoint,
    in_environment: &DynamicEnvironment<W>,
) -> Option<SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 3]>>
where
    W: Into<WaypointR2> + Waypoint,
{

    None
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

struct Proximity {
    enter: Option<TimePoint>,
    exit: Option<TimePoint>,
}

#[inline]
fn detect_proximity(
    proximity_dist_squared: f64,
    line_a: (&WaypointR2, &WaypointR2),
    line_b: (&WaypointR2, &WaypointR2),
) -> Proximity {
    let t_range = compute_t_range(line_a, line_b);
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
            return Proximity {
                enter: None,
                exit: None,
            }
        }
    }

    let radicand = b.powi(2) - 4.0 * a * c;
    if radicand >= 0.0 {
        let sqrt_radicand = radicand.sqrt();
        let dt = (t_range.1 - t_range.0).as_secs_f64();
        let t_m = (-b - sqrt_radicand) / (2.0 * a);
        let t_p = (-b + sqrt_radicand) / (2.0 * a);
        assert!(t_m <= t_p);

        if t_p < 0.0 {
            // The intersection happens before the time range
            return Proximity {
                enter: None,
                exit: None,
            };
        }

        if dt < t_m {
            // The intersection happens after the time range
            return Proximity {
                enter: None,
                exit: None,
            }
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

    // The objects are moving relative to each other, but they will never come
    // close enough to each other.
    Proximity { enter: None, exit: None }
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
    for (wp0, wp1) in obs_traj.iter_from(candidate_time).pairs() {
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

            if exit < candidate_time {
                // The violation was relieved before the candidate
                // time so it is not a relevant violation.
                this_obs_violated = false;
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
    for (wp0, wp1) in obs_traj.iter_from(previous_candidate_time).pairs() {
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

    None
}

#[inline]
fn is_safe_segment<W>(
    profile: &Profile,
    line_a: (&WaypointR2, &WaypointR2),
    in_environment: &DynamicEnvironment<W>,
) -> bool
where
    W: Into<WaypointR2> + Waypoint,
{
    for obs in &in_environment.obstacles {
        let acceptable_distance_squared = (profile.footprint_radius + obs.profile.footprint_radius).powi(2);

        let obs_traj = match &obs.trajectory {
            Some(r) => r,
            None => continue,
        };

        for (wp0_b, wp1_b) in obs_traj.iter_from(line_a.0.time).pairs() {
            if line_a.1.time < *wp0_b.time() {
                // The trajectories are no longer overlapping in time so there
                // is no longer a risk.
                return true;
            }

            let wp0_b: WaypointR2 = wp0_b.into();
            let wp1_b: WaypointR2 = wp1_b.into();
            let line_b = (&wp0_b, &wp1_b);

            let proximity = detect_proximity(
                acceptable_distance_squared,
                line_a,
                line_b,
            );

            if let Some(t) = proximity.enter {
                let t_range = compute_t_range(line_a, line_b);
                let t = (t - t_range.0).as_secs_f64();
                let (dp0, dv) = compute_dp0_dv(line_a, line_b, &t_range);
                let a = dv.dot(&dv);
                let b = 2.0 * dv.dot(&dp0);
                let deriv = 2.0*a * t + 2.0*b;
                if deriv < 0.0 {
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
