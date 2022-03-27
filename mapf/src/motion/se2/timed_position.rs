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
use time_point::TimePoint;
use crate::motion::timed;

pub struct Waypoint<Precision> {
    time: TimePoint,
    pub position: Isometry2<Precision>,
}

impl<P> timed::Timed for Waypoint<P> {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }
}

impl<P> timed::private::SetTime for Waypoint<P> {
    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

pub type Waypoint_f32 = Waypoint<f32>;
pub type Waypoint_f64 = Waypoint<f64>;
