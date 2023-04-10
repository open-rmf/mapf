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

pub mod environment;
pub use environment::*;

pub mod timed;
pub use timed::*;

pub mod r2;
pub mod se2;

pub mod waypoint;
pub use waypoint::*;

pub mod trajectory;
pub use trajectory::{FindWaypoint, Trajectory};

pub mod travel_effort_cost;
pub use travel_effort_cost::*;

pub mod travel_time_cost;
pub use travel_time_cost::*;

pub mod safe_interval;
pub use safe_interval::*;

pub mod speed_limit;
pub use speed_limit::*;

pub mod conflict;
pub use conflict::*;

pub use time_point::{Duration, TimePoint};

use crate::error::ThisError;

/// The default translational threshold is 1mm
pub const DEFAULT_TRANSLATIONAL_THRESHOLD: f64 = 0.001;

/// The default rotational threshold is 1-degree.
pub const DEFAULT_ROTATIONAL_THRESHOLD: f64 = 1.0f64 * std::f64::consts::PI / 180.0;

// TODO(@mxgrey): Should each implementation of Interpolation be allowed to specify
// its own error types?
#[derive(Clone, Copy, Debug, PartialEq, Eq, ThisError)]
pub enum InterpError {
    #[error("The requested time [{request:?}] is outside the time range of the motion {range:?}")]
    OutOfBounds {
        range: [TimePoint; 2],
        request: TimePoint,
    },
    #[error("The requested interpolation does not have a unique solution")]
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
