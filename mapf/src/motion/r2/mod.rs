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

pub type Position = nalgebra::geometry::Point2<f64>;
pub type Point = Position;
pub type Velocity = nalgebra::Vector2<f64>;

pub mod timed_position;
pub use timed_position::*;

pub type LinearTrajectory = super::Trajectory<Waypoint>;

pub mod space;
pub use space::*;

pub mod direct_travel;
pub use direct_travel::*;

pub mod positioned;
pub use positioned::*;
