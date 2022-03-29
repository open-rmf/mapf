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

use nalgebra::geometry::Isometry2;
use nalgebra::{Vector2, Vector3};
use time_point::TimePoint;
use simba::simd::SimdRealField;
use crate::motion::{timed, Interpolation, InterpError};

#[derive(Clone, Debug)]
pub struct Waypoint<Precision> {
    pub time: TimePoint,
    pub position: Isometry2<Precision>,
}

pub struct Motion<Precision> {
    initial_wp: Waypoint<Precision>,
    final_wp: Waypoint<Precision>,
}

impl<P> timed::Timed for Waypoint<P> {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }
    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl<P: Copy + Clone + SimdRealField> crate::motion::Motion<Isometry2<P>, Vector3<P>> for Motion<P> {
    fn compute_position(&self, time: &TimePoint) -> Result<Isometry2<P>, InterpError> {
        if time.nanos_since_zero < self.initial_wp.time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
        }

        if self.final_wp.time.nanos_since_zero < time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
        }

        let delta_t = (self.final_wp.time - self.initial_wp.time).as_secs_f64();
        let delta_p = (self.final_wp.position.translation.vector - self.initial_wp.position.translation.vector);
        // self.final_wp.position.try_lerp_slerp()
        return Ok(self.initial_wp.position.clone());
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<Vector3<P>, InterpError> {
        return Err(InterpError::Indeterminate);
    }
}

impl<P: Copy + Clone + SimdRealField> Interpolation<Isometry2<P>, Vector3<P>> for Waypoint<P> {
    type Motion = Motion<P>;

    fn interpolate(&self, up_to: &Self) -> Self::Motion {
        return Self::Motion{
            initial_wp: self.clone(),
            final_wp: up_to.clone()
        }
    }
}

pub type WaypointF32 = Waypoint<f32>;
pub type WaypointF64 = Waypoint<f64>;
