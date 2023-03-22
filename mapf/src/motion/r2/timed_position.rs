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

use super::{Position, Velocity};
use crate::motion::{
    self, timed, InterpError, Interpolation, TimePoint,
    se2::Waypoint as WaypointSE2,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub time: TimePoint,
    pub position: Position,
}

impl Waypoint {
    pub fn new(time: TimePoint, x: f64, y: f64) -> Self {
        return Waypoint {
            time,
            position: Position::new(x, y),
        };
    }
}

impl timed::Timed for Waypoint {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }

    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl From<WaypointSE2> for Waypoint {
    fn from(value: WaypointSE2) -> Self {
        Self::new(
            value.time,
            value.position.translation.x,
            value.position.translation.y,
        )
    }
}

impl motion::Waypoint for Waypoint {
    type Position = Position;
    type Velocity = Velocity;
}

pub struct Motion {
    initial_wp: Waypoint,
    final_wp: Waypoint,
    v: Velocity,
}

impl Motion {
    fn new(initial_wp: Waypoint, final_wp: Waypoint) -> Self {
        let dx = final_wp.position - initial_wp.position;
        let dt = (final_wp.time - initial_wp.time).as_secs_f64();
        Self {
            initial_wp,
            final_wp,
            v: dx / dt,
        }
    }

    pub fn in_range(&self, time: &TimePoint) -> Result<(), InterpError> {
        if time.nanos_since_zero < self.initial_wp.time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
        }

        if self.final_wp.time.nanos_since_zero < time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
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

impl Interpolation<Position, Velocity> for Waypoint {
    type Motion = Motion;

    fn interpolate(&self, up_to: &Self) -> Self::Motion {
        return Self::Motion::new(*self, *up_to);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use crate::motion::{Motion, Duration};

    #[test]
    fn test_interpolation() {
        let t0 = TimePoint::new(0);
        let t1 = t0 + Duration::from_secs_f64(2.0);
        let wp0 = Waypoint::new(t0, 1.0, 5.0);
        let wp1 = Waypoint::new(t1, 1.0, 10.0);

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
