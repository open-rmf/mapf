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
use crate::{
    motion::{self, se2, timed, InterpError, Interpolation},
    domain::{Extrapolator, Reversible},
    error::NoError,
};
use arrayvec::ArrayVec;
use time_point::{Duration, TimePoint};
use thiserror::Error as ThisError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub time: TimePoint,
    pub position: Position,
}

impl timed::Timed for Waypoint {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }

    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl Waypoint {
    pub fn new(time: TimePoint, x: f64, y: f64) -> Self {
        return Waypoint {
            time,
            position: Position::new(x, y),
        };
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineFollow {
    speed: f64,
    direction: f64,
    distance_threshold: f64,
}

impl LineFollow {
    pub fn new(speed: f64) -> Result<Self, ()> {
        if speed <= 0.0 {
            return Err(());
        }

        Ok(LineFollow {
            speed,
            direction: 1.0,
            distance_threshold: motion::DEFAULT_TRANSLATIONAL_THRESHOLD,
        })
    }

    pub fn set_speed(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.speed = value;
        Ok(())
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    fn extrapolate_impl(
        &self,
        from_waypoint: &Waypoint,
        to_target: &Position,
        speed_limit: Option<f64>,
    ) -> Result<Option<(ArrayVec<Waypoint, 1>, Waypoint)>, LineFollowError> {
        let speed = if let Some(limit) = speed_limit {
            if limit <= 0.0 {
                return Err(LineFollowError::InvalidSpeedLimit(limit));
            }
            self.speed.min(limit)
        } else {
            self.speed
        };

        let dx = (to_target - from_waypoint.position).norm();
        if dx <= self.distance_threshold {
            // The target is close enough to the start point that we treat it
            // as though the agent is already there.
            let wp = Waypoint::new(from_waypoint.time, to_target.x, to_target.y);
            return Ok(Some((ArrayVec::new(), wp)));
        }

        let t = Duration::from_secs_f64(self.direction * dx / speed) + from_waypoint.time;
        let wp = Waypoint::new(t, to_target.x, to_target.y);
        let extrap = ArrayVec::from_iter([wp]);
        Ok(Some((extrap, wp)))
    }
}

impl Extrapolator<Waypoint, Position, ()> for LineFollow {
    type Extrapolation = ArrayVec<Waypoint, 1>;
    type ExtrapolationError = LineFollowError;
    fn extrapolate(
        &self,
        from_state: &Waypoint,
        to_target: &Position,
        _: &(),
    ) -> Result<Option<(Self::Extrapolation, Waypoint)>, Self::ExtrapolationError> {
        self.extrapolate_impl(from_state, to_target, None)
    }
}

/// A trait for properties that can specify speed limits.
pub trait SpeedLimiter {
    fn speed_limit(&self) -> Option<f64>;
}

impl<T: SpeedLimiter> Extrapolator<Waypoint, Position, T> for LineFollow {
    type Extrapolation = ArrayVec<Waypoint, 1>;
    type ExtrapolationError = LineFollowError;
    fn extrapolate(
        &self,
        from_state: &Waypoint,
        to_target: &Position,
        with_guidance: &T,
    ) -> Result<Option<(ArrayVec<Waypoint, 1>, Waypoint)>, Self::ExtrapolationError> {
        self.extrapolate_impl(from_state, to_target, with_guidance.speed_limit())
    }
}

#[derive(Debug, ThisError)]
pub enum LineFollowError {
    #[error("LineFollow::extrapolate was provided with an invalid speed limit (must be >0.0): {0}")]
    InvalidSpeedLimit(f64),
}

impl Reversible for LineFollow {
    type Reverse = LineFollow;
    type ReversalError = NoError;

    fn reverse(&self) -> Result<Self::Reverse, NoError> {
        Ok(Self {
            speed: self.speed,
            direction: -1.0 * self.direction,
            distance_threshold: self.distance_threshold,
        })
    }
}

// impl From<&se2::timed_position::DifferentialDriveLineFollow> for LineFollow {
//     fn from(other: &se2::timed_position::DifferentialDriveLineFollow) -> Self {
//         LineFollow::new(other.translational_speed())
//             .expect("corrupt speed in DifferentialDriveLineFollow")
//     }
// }

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use motion::Motion;

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

    #[test]
    fn test_extrapolation() {
        let t0 = TimePoint::from_secs_f64(3.0);
        let wp0 = Waypoint::new(t0, 1.0, -3.0);
        let movement = LineFollow::new(2.0).expect("Failed to make LineFollow");
        let p_target = Position::new(1.0, 3.0);
        let (waypoints, end) = movement
            .extrapolate(&wp0, &p_target, &())
            .expect("Failed to extrapolate")
            .expect("Missing extrapolation result");
        assert_eq!(waypoints.len(), 1);
        assert_relative_eq!(
            waypoints.last().unwrap().time.as_secs_f64(),
            (t0 + Duration::from_secs_f64(6.0 / 2.0)).as_secs_f64()
        );

        assert_relative_eq!(waypoints.last().unwrap().position[0], p_target[0]);

        assert_relative_eq!(waypoints.last().unwrap().position[1], p_target[1]);

        let trajectory = motion::r2::LinearTrajectory::from_iter(
            [wp0].into_iter().chain(waypoints.iter().map(|wp| *wp)),
        )
        .expect("Failed to create trajectory");
        assert_eq!(trajectory.len(), 2);
    }
}
