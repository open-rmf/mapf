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
pub use trajectory::Trajectory;

use time_point::{TimePoint, Duration};

/// The default translational threshold is 1mm
pub const DEFAULT_TRANSLATIONAL_THRESHOLD: f64 = 0.001;

/// The default rotational threshold is 1-degree.
pub const DEFAULT_ROTATIONAL_THRESHOLD: f64 = 1.0f64 * std::f64::consts::PI / 180.0;

pub trait Waypoint:
    timed::Timed
    + Interpolation<Self::Position, Self::Velocity>
    + Clone
    + std::fmt::Debug
{
    /// What type of spatial position does the waypoint have
    type Position;

    /// How does the waypoint represent the time derivative of its position
    type Velocity;
}

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtrapError {
    /// The requested extrapolation is impossible. This usually means that your
    /// Movement parameters are flawed or there is something wrong with the
    /// requested position.
    Impossible,

    /// There is no unique solution for how to extrapolate to the target.
    Indeterminate
}

// TODO(MXG): Do we really need extrapolation and Movement to be directly coupled
// to the type of Waypoint, or should we decouple them so that users can customize
// the way movement and extrapolation are implemented?
pub trait Extrapolation<W: Waypoint, Target> {

    /// Extrapolate a new waypoint from this one given a target waypoint and
    /// a movement description.
    ///
    /// If the current waypoint is already at the target position, then an empty
    /// Vec<W> will be returned.
    fn extrapolate(&self,
        from_waypoint: &W,
        to_target: &Target,
    ) -> Result<Vec<W>, ExtrapError>;
}
