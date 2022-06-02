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

use super::Waypoint;

/// Can extrapolate the motion from a waypoint towards a target.
pub trait Extrapolator<W: Waypoint, Target> {
    /// The type of error that can happen while extrapolating.
    type Error: std::fmt::Debug;

    /// An iterator that produces a sequence of waypoints representing a motion
    /// through time.
    type Extrapolation<'a>: IntoIterator<Item=W> where Self: 'a;

    /// Extrapolate a new waypoint from this one given a target waypoint and
    /// a movement description.
    ///
    /// If the current waypoint is already at the target position, then an empty
    /// iterator will be returned.
    fn extrapolate<'a>(
        &'a self,
        from_waypoint: &W,
        to_target: &Target,
    ) -> Result<Self::Extrapolation<'a>, Self::Error>;
}

/// Trait to indicate that the extrapolation can be reversed and provide
/// extrapolation in the reverse direction of time.
pub trait Reversible<W: Waypoint, Target> {
    /// The type of error that can happen while reversing.
    type Error: std::fmt::Debug;

    /// The Reverse of this extrapolation.
    type Reverse: Extrapolator<W, Target>;

    /// Get the reverse of this extrapolation.
    fn reverse(&self) -> Result<Self::Reverse, Self::Error>;
}
