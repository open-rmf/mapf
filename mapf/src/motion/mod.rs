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
pub mod r2;

pub mod waypoint;
pub use waypoint::Waypoint;

pub mod trajectory;
pub use trajectory::Trajectory;

pub mod extrapolator;
pub use extrapolator::Extrapolator;

pub mod graph_search;

pub mod reach;


pub mod collide;

use time_point::{TimePoint, Duration};

/// The default translational threshold is 1mm
pub const DEFAULT_TRANSLATIONAL_THRESHOLD: f64 = 0.001;

/// The default rotational threshold is 1-degree.
pub const DEFAULT_ROTATIONAL_THRESHOLD: f64 = 1.0f64 * std::f64::consts::PI / 180.0;

// TODO(MXG): Should each implementation of Interpolation be allowed to specify
// its own error types?
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterpError {
    /// The requested time is outside the time range of the motion
    OutOfBounds,

    /// The requested interpolation does not have a unique solution
    Indeterminate,
}

pub trait Motion<Position, Velocity> {

    /// Compute the position of this motion at a specific time. If the requested
    /// time is outside the bounds of the motion, then this will return an Err.
    fn compute_position(&self, time: &TimePoint) -> Result<Position, InterpError>;

    /// Compute the velocity of this motion at a specific time. If the requested
    /// time is outside the bounds of the motion, then this will return an Err.
    fn compute_velocity(&self, time: &TimePoint) -> Result<Velocity, InterpError>;
}

pub trait Interpolation<Position, Velocity> {
    type Motion: Motion<Position, Velocity>;

    /// Interpolate the motion from this waypoint to another one.
    fn interpolate(&self, up_to: &Self) -> Self::Motion;
}
