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

use super::{MaybePositioned, Position, Positioned, Velocity};
use crate::{
    error::NoError,
    motion::{
        self,
        se2::{MaybeOriented, WaypointSE2},
        IntegrateWaypoints, InterpError, Interpolation, MaybeTimed, TimePoint, Timed,
    },
};
use arrayvec::ArrayVec;

#[derive(Clone, Copy, PartialEq)]
pub struct WaypointR2 {
    pub time: TimePoint,
    pub position: Position,
}

impl std::fmt::Debug for WaypointR2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WaypointR2")
            .field("time", &self.time.as_secs_f64())
            .field("position", &self.position)
            .finish()
    }
}

impl WaypointR2 {
    pub fn new(time: TimePoint, x: f64, y: f64) -> Self {
        WaypointR2 {
            time,
            position: Position::new(x, y),
        }
    }

    pub fn new_f64(time: f64, x: f64, y: f64) -> Self {
        WaypointR2 {
            time: TimePoint::from_secs_f64(time),
            position: Position::new(x, y),
        }
    }

    pub fn with_yaw(self, yaw: f64) -> WaypointSE2 {
        WaypointSE2::new(self.time, self.position.x, self.position.y, yaw)
    }
}

impl Timed for WaypointR2 {
    fn time(&self) -> TimePoint {
        return self.time;
    }

    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl MaybeTimed for WaypointR2 {
    fn maybe_time(&self) -> Option<TimePoint> {
        Some(self.time)
    }
}

impl Positioned for WaypointR2 {
    fn point(&self) -> super::Point {
        self.position.point()
    }
}

impl MaybePositioned for WaypointR2 {
    fn maybe_point(&self) -> Option<super::Point> {
        Some(self.point())
    }
}

impl MaybeOriented for WaypointR2 {
    fn maybe_oriented(&self) -> Option<motion::se2::Orientation> {
        None
    }
}

impl From<WaypointSE2> for WaypointR2 {
    fn from(value: WaypointSE2) -> Self {
        Self::new(
            value.time,
            value.position.translation.x,
            value.position.translation.y,
        )
    }
}

impl motion::Waypoint for WaypointR2 {
    type Position = Position;
    type Velocity = Velocity;
    fn position(&self) -> Self::Position {
        self.position
    }

    fn zero_velocity() -> Self::Velocity {
        Velocity::zeros()
    }
}

pub struct Motion {
    initial_wp: WaypointR2,
    final_wp: WaypointR2,
    v: Velocity,
}

impl Motion {
    fn new(initial_wp: WaypointR2, final_wp: WaypointR2) -> Self {
        let dx = final_wp.position - initial_wp.position;
        let dt = (final_wp.time - initial_wp.time).as_secs_f64();
        Self {
            initial_wp,
            final_wp,
            v: dx / dt,
        }
    }

    pub fn in_range(&self, time: &TimePoint) -> Result<(), InterpError> {
        if *time < self.initial_wp.time {
            return Err(InterpError::OutOfBounds {
                range: [self.initial_wp.time, self.final_wp.time],
                request: *time,
            });
        }

        if self.final_wp.time < *time {
            return Err(InterpError::OutOfBounds {
                range: [self.initial_wp.time, self.final_wp.time],
                request: *time,
            });
        }

        return Ok(());
    }
}

impl crate::motion::Motion<Position, Velocity> for Motion {
    fn compute_position(&self, time: &TimePoint) -> Result<Position, InterpError> {
        self.in_range(time)?;
        let delta_t = (*time - self.initial_wp.time).as_secs_f64();
        Ok(self.initial_wp.position + self.v * delta_t)
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<Velocity, InterpError> {
        self.in_range(time)?;
        Ok(self.v)
    }
}

impl Interpolation<Position, Velocity> for WaypointR2 {
    type Motion = Motion;

    fn interpolate(&self, up_to: &Self) -> Self::Motion {
        return Self::Motion::new(*self, *up_to);
    }
}

impl<W, const N: usize> IntegrateWaypoints<W> for ArrayVec<WaypointR2, N>
where
    WaypointR2: Into<W>,
{
    type IntegratedWaypointIter<'a>
        = ArrayVec<Result<W, NoError>, N>
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
        W: 'a,
    {
        // TODO(@mxgrey): Should it be an error if _initial_waypoint is None?
        // We do not store the initial waypoint for the trajectory in the action
        // so it would be wrong to initiate a trajectory with an action. However
        // we also don't need that initial waypoint information to produce the
        // correct waypoints, so it's unclear whether that needs to be an error.
        self.into_iter().map(|w| Ok(w.clone().into())).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{Duration, Motion};
    use approx::assert_relative_eq;

    #[test]
    fn test_interpolation() {
        let t0 = TimePoint::new(0);
        let t1 = t0 + Duration::from_secs_f64(2.0);
        let wp0 = WaypointR2::new(t0, 1.0, 5.0);
        let wp1 = WaypointR2::new(t1, 1.0, 10.0);

        let motion = wp0.interpolate(&wp1);
        let t = (t1 - t0) / 2_f64 + t0;
        let p = motion.compute_position(&t).ok().unwrap();
        assert_relative_eq!(p[0], 1_f64, max_relative = 0.001);
        assert_relative_eq!(p[1], 7.5_f64, max_relative = 0.001);

        let v = motion.compute_velocity(&t).ok().unwrap();
        assert_relative_eq!(v[0], 0_f64, max_relative = 0.001);
        assert_relative_eq!(v[1], 5.0 / 2.0, max_relative = 0.001);
    }
}
