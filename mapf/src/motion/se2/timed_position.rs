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

use super::{Position, Vector, Velocity};
use crate::{
    motion::{
        self, timed, InterpError, Interpolation, TimePoint, IntegrateWaypoints,
    },
    error::NoError,
};
use arrayvec::ArrayVec;

#[derive(Clone, Copy, PartialEq)]
pub struct WaypointSE2 {
    pub time: TimePoint,
    pub position: Position,
}

impl timed::Timed for WaypointSE2 {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }

    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl WaypointSE2 {
    pub fn new(time: TimePoint, x: f64, y: f64, yaw: f64) -> Self {
        return WaypointSE2 {
            time,
            position: Position::new(Vector::new(x, y), yaw),
        };
    }

    pub fn new_f64(time: f64, x: f64, y: f64, yaw: f64) -> Self {
        return WaypointSE2 {
            time: TimePoint::from_secs_f64(time),
            position: Position::new(Vector::new(x, y), yaw),
        }
    }
}

impl std::fmt::Debug for WaypointSE2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f
            .debug_struct("WaypointSE2")
            .field("time", &self.time.as_secs_f64())
            .field("position", &self.position)
            .finish()
    }
}

impl motion::Waypoint for WaypointSE2 {
    type Position = Position;
    type Velocity = Velocity;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    initial_wp: WaypointSE2,
    final_wp: WaypointSE2,
}

impl Motion {
    pub fn in_time_range(&self, time: &TimePoint) -> Result<(), InterpError> {
        if time.nanos_since_zero < self.initial_wp.time.nanos_since_zero {
            return Err(InterpError::OutOfBounds {
                range: [self.initial_wp.time, self.final_wp.time],
                request: *time,
            });
        }

        if self.final_wp.time.nanos_since_zero < time.nanos_since_zero {
            return Err(InterpError::OutOfBounds {
                range: [self.initial_wp.time, self.final_wp.time],
                request: *time,
            });
        }

        return Ok(());
    }
}

impl motion::Motion<Position, Velocity> for Motion {
    fn compute_position(&self, time: &TimePoint) -> Result<Position, InterpError> {
        self.in_time_range(time)?;
        let delta_t = (*time - self.initial_wp.time).as_secs_f64();
        let t_range = (self.final_wp.time - self.initial_wp.time).as_secs_f64();
        return Ok(self
            .initial_wp
            .position
            .lerp_slerp(&self.final_wp.position, delta_t / t_range));
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<Velocity, InterpError> {
        self.in_time_range(time)?;

        // TODO(@mxgrey): Since velocity is taken to be constant across the whole
        // range, this could be precomputed once and saved inside the Motion object.
        let t_range = (self.final_wp.time - self.initial_wp.time).as_secs_f64();
        let p0 = &self.initial_wp.position.translation.vector;
        let p1 = &self.final_wp.position.translation.vector;
        let linear_v = (p1 - p0) / t_range;

        let r0 = &self.initial_wp.position.rotation;
        let r1 = &self.final_wp.position.rotation;
        let angular_v = (r1 / r0).angle() / t_range;
        return Ok(Velocity {
            translational: linear_v,
            rotational: angular_v,
        });
    }
}

impl Interpolation<Position, Velocity> for WaypointSE2 {
    type Motion = Motion;

    fn interpolate(&self, up_to: &Self) -> Self::Motion {
        return Self::Motion {
            initial_wp: self.clone(),
            final_wp: up_to.clone(),
        };
    }
}

impl<W, const N: usize> IntegrateWaypoints<W> for ArrayVec<WaypointSE2, N>
where
    WaypointSE2: Into<W>,
{
    type IntegratedWaypointIter<'a> = ArrayVec<Result<W, NoError>, N>
    where
        W: 'a;

    type WaypointIntegrationError = NoError;
    fn integrated_waypoints<'a>(
        &'a self,
        _initial_waypoint: Option<W>,
    ) -> Self::IntegratedWaypointIter<'a>
    where
        Self: 'a,
        Self::WaypointIntegrationError: 'a,
        W: 'a
    {
        // TODO(@mxgrey): Should it be an error if _initial_waypoint is None?
        // We do not store the initial waypoint for the trajectory in the action
        // so it would be wrong to initiate a trajectory with an action. However
        // we also don't need that initial waypoint information to produce the
        // correct waypoints, so it's unclear whether that needs to be an error.
        self
        .into_iter()
        .map(|w| Ok(w.clone().into()))
        .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use motion::Motion;

    #[test]
    fn test_interpolation() {
        let t0 = time_point::TimePoint::new(0);
        let t1 = t0 + time_point::Duration::from_secs_f64(2.0);
        let wp0 = WaypointSE2::new(t0, 1.0, 5.0, 10f64.to_radians());
        let wp1 = WaypointSE2::new(t1, 1.0, 10.0, -20f64.to_radians());

        let motion = wp0.interpolate(&wp1);
        let t = (t1 - t0) / 2f64 + t0;
        let p = motion.compute_position(&t).ok().unwrap();
        assert_relative_eq!(p.translation.vector[0], 1f64, max_relative = 0.001);
        assert_relative_eq!(p.translation.vector[1], 7.5f64, max_relative = 0.001);
        assert_relative_eq!(p.rotation.angle(), -5f64.to_radians(), max_relative = 0.001);

        let v = motion.compute_velocity(&t).ok().unwrap();
        assert_relative_eq!(v.translational[0], 0f64, max_relative = 0.001);
        assert_relative_eq!(v.translational[1], 5.0 / 2.0, max_relative = 0.001);
        assert_relative_eq!(
            v.rotational,
            -30f64.to_radians() / 2.0,
            max_relative = 0.001
        );
    }
}
