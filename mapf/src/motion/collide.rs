/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
    motion::{Trajectory, Interpolation, Motion, TimePoint, Duration, r2, se2},
    expander::{AimlessConstraint, TargetedConstraint},
    node::Agent,
    error::NoError,
};
use nalgebra::Vector2;
use std::sync::Arc;

fn compute_t_range(
    wp0_a: &r2::timed_position::Waypoint,
    wp1_a: &r2::timed_position::Waypoint,
    wp0_b: &r2::timed_position::Waypoint,
    wp1_b: &r2::timed_position::Waypoint,
) -> (TimePoint, TimePoint) {
    (wp0_a.time.max(wp0_b.time), wp1_a.time.min(wp1_b.time))
}

fn compute_p0_v(
    wp0: &r2::timed_position::Waypoint,
    wp1: &r2::timed_position::Waypoint,
    t_range: &(TimePoint, TimePoint),
) -> (Vector2<f64>, Vector2<f64>) {
    let dp = wp1.position - wp0.position;
    let dt = (wp1.time - wp0.time).as_secs_f64();
    let p0 = wp0.interpolate(wp1).compute_position(&t_range.0).unwrap();
    (p0.coords, dp/dt)
}

fn time_within_range(
    t: f64,
    t_range: &(TimePoint, TimePoint),
) -> Option<TimePoint> {
    let dt = (t_range.1 - t_range.0).as_secs_f64();
    if 0.0 <= t && t <= dt {
        return Some(t_range.0 + Duration::from_secs_f64(t));
    }

    return None;
}

fn detect_proximity(
    dist_squared: f64,
    mut iter_a: impl Iterator<Item=r2::timed_position::Waypoint>,
    mut iter_b: impl Iterator<Item=r2::timed_position::Waypoint>,
) -> Option<TimePoint> {
    let mut wp0_a_opt = iter_a.next();
    let mut wp1_a_opt = iter_a.next();
    let mut wp0_b_opt = iter_b.next();
    let mut wp1_b_opt = iter_b.next();

    while let (Some(wp0_a), Some(wp1_a), Some(wp0_b), Some(wp1_b)) = (wp0_a_opt, wp1_a_opt, wp0_b_opt, wp1_b_opt) {
        if wp1_a.time < wp0_b.time {
            wp0_a_opt = Some(wp1_a);
            wp1_a_opt = iter_a.next();
            continue;
        }

        if wp1_b.time < wp0_a.time {
            wp0_b_opt = Some(wp1_b);
            wp1_b_opt = iter_b.next();
            continue;
        }

        let t_range = compute_t_range(&wp0_a, &wp1_a, &wp0_b, &wp1_b);
        let (p0_a, v_a) = compute_p0_v(&wp0_a, &wp1_a, &t_range);
        let (p0_b, v_b) = compute_p0_v(&wp0_b, &wp1_b, &t_range);
        let dp0 = p0_b - p0_a;
        let dv = v_b - v_a;

        let a = dv.dot(&dv);
        let b = 2.0*dv.dot(&dp0);
        let c = dp0.dot(&dp0) - dist_squared;

        if a.abs() < 1e-8 {
            // The two motions have almost identical velocities or nearly
            // orthogonal velocities
            if b.abs() < 1e-8 {
                // There is no relative motion
                if c <= 0.0 {
                    return Some(wp0_a.time.max(wp0_b.time));
                }
            } else {
                let t = -c/b;
                if let Some(t) = time_within_range(t, &t_range) {
                    return Some(t);
                }
            }
        } else {
            let radicand = b.powi(2) - 4.0*a*c;
            if radicand >= 0.0 {
                let sqrt_radicand = radicand.sqrt();
                let t_m = (-b - sqrt_radicand)/(2.0*a);
                if let Some(t_m) = time_within_range(t_m, &t_range) {
                    return Some(t_m);
                }

                let t_p = (-b + sqrt_radicand)/(2.0*a);
                if let Some(t_p) = time_within_range(t_p, &t_range) {
                    return Some(t_p);
                }
            }
        }

        let tp0 = wp0_a.time.min(wp0_b.time);
        let t0 = tp0.as_secs_f64();
        if a*t0.powi(2) + b*t0 + c <= 0.0 {
            return Some(tp0);
        }

        let mut advance_a = false;
        let mut advance_b = false;
        if wp1_a.time < wp1_b.time {
            advance_a = true;
        } else if wp1_b.time < wp1_a.time {
            advance_b = true;
        } else {
            (advance_a, advance_b) = (true, true);
        }

        if advance_a {
            wp0_a_opt = Some(wp1_a);
            wp1_a_opt = iter_a.next();
        }

        if advance_b {
            wp0_b_opt = Some(wp1_b);
            wp1_b_opt = iter_b.next();
        }
    }

    return None;
}

pub fn detect_collision_circles_r2(
    radius_a: f64,
    trajectory_a: &r2::LinearTrajectory,
    radius_b: f64,
    trajectory_b: &r2::LinearTrajectory,
) -> Option<TimePoint> {
    detect_proximity(
        (radius_a + radius_b).powi(2),
        trajectory_a.iter().map(|w| w.0),
        trajectory_b.iter().map(|w| w.0),
    )
}

pub fn detect_collision_circles_se2(
    radius_a: f64,
    trajectory_a: &se2::LinearTrajectory,
    radius_b: f64,
    trajectory_b: &se2::LinearTrajectory,
) -> Option<TimePoint> {
    detect_proximity(
        (radius_a + radius_b).powi(2),
        trajectory_a.iter().map(|w| w.0.into()),
        trajectory_b.iter().map(|w| w.0.into()),
    )
}

pub struct CircleCollisionConstraint {
    pub obstacles: Vec<(f64, se2::LinearTrajectory)>,
    pub agent_radius: f64,
}

impl<N> AimlessConstraint<N> for CircleCollisionConstraint
where N: Agent<se2::timed_position::Waypoint, Trajectory<se2::timed_position::Waypoint>>
{
    type ConstraintError = NoError;
    fn constrain(&self, node: Arc<N>) -> Result<Option<std::sync::Arc<N>>, Self::ConstraintError> {
        if let Some(trajectory) = node.action() {
            for (r_obs, t_obs) in &self.obstacles {
                if detect_collision_circles_se2(
                    self.agent_radius,
                    trajectory,
                    *r_obs,
                    t_obs
                ).is_some() {
                    return Ok(None);
                }
            }
        }

        return Ok(Some(node));
    }
}

impl<N, G> TargetedConstraint<N, G> for CircleCollisionConstraint
where N: Agent<se2::timed_position::Waypoint, Trajectory<se2::timed_position::Waypoint>>
{
    type ConstraintError = NoError;
    fn constrain(&self, node: Arc<N>, _: &G) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        AimlessConstraint::constrain(self, node)
    }
}
