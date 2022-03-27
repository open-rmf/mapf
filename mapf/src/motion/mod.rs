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

pub mod timed;

pub mod se2;

pub mod trajectory;
use trajectory::Trajectory;

use time_point::TimePoint;

pub trait Motion<Position, Velocity> {
    fn compute_position(&self, time: &TimePoint) -> Position;
    fn compute_velocity(&self, time: &TimePoint) -> Velocity;
}

pub trait Interpolation<Position, Velocity> {
    type Motion: Motion<Position, Velocity>;

    fn interpolate(&self, up_to: &Self) -> Self::Motion;
}
