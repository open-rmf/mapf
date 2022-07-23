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

use crate::{
    motion::{Waypoint, Trajectory},
    error::Error,
};

/// Can extrapolate the motion from a waypoint towards a target.
pub trait Extrapolator<W: Waypoint, Target> {
    /// The type of error that can happen while extrapolating.
    type Error: Error;

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

    /// Extrapolate a trajectory from a waypoint to the given target.
    ///
    /// If an extrapolation error happens, that will be forwarded to the result.
    ///
    /// If no motion is needed to move from the start waypoint to the target,
    /// then Ok(None) will be returned instead of Ok(Trajectory<W>).
    ///
    /// Note that this function performs heap allocation whereas extrapolate()
    /// typically does not (but that depends on the underlying implementation
    /// of the Extrapolator).
    fn make_trajectory(
        &self,
        from_waypoint: W,
        to_target: &Target,
    ) -> Result<Option<Trajectory<W>>, Self::Error> {
        let motion = self.extrapolate(&from_waypoint, to_target)?;
        Ok(Trajectory::from_iter(
            [from_waypoint].into_iter().chain(motion.into_iter())
        ).ok())
    }
}

/// Trait to indicate that the extrapolation can be reversed and provide
/// extrapolation in the reverse direction of time.
pub trait Reversible<W: Waypoint, Target> {
    /// The type of error that can happen while reversing.
    type Error: Error;

    /// The Reverse of this extrapolation.
    type Reverse: Extrapolator<W, Target>;

    /// Get the reverse of this extrapolation.
    fn reverse(&self) -> Result<Self::Reverse, Self::Error>;
}
