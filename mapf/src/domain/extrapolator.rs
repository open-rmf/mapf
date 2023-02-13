/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

use crate::error::NoError;

/// Create an action that extrapolates from an initial state to a target.
///
/// See also: [`Connectable`]
pub trait Extrapolator<State, Target, Guidance> {
    /// What kind of action is produced during extrapolation
    type Extrapolation;

    /// What kind of error can happen during extrapolation
    type ExtrapolationError;

    /// Extrapolate an action from a state to a target with the provided guidance.
    /// Return the action that has been extrapolated as well as the final state
    /// once the target is reached. If the target cannot be reached, return None.
    ///
    /// * `from_state` - The initial state to extrapolate from.
    /// * `to_target` - The target to extrapolate towards. This can be a different
    /// type than the `State`. For example, if `State` is a position in SE2, then
    /// the `Target` could be a position in R2.
    /// * `with_guidance` - Parameters to describe how the extrapolation should
    /// be performed. This can include constraints like speed limits.
    fn extrapolate(
        &self,
        from_state: &State,
        to_target: &Target,
        with_guidance: &Guidance,
    ) -> Result<Option<(Self::Extrapolation, State)>, Self::ExtrapolationError>;
}

pub struct NoExtrapolation<E>(std::marker::PhantomData<E>);
impl<State, Target, Guidance, E> Extrapolator<State, Target, Guidance> for NoExtrapolation<E> {
    type Extrapolation = E;
    type ExtrapolationError = NoError;
    fn extrapolate(
        &self,
        _: &State,
        _: &Target,
        _: &Guidance,
    ) -> Result<Option<(Self::Extrapolation, State)>, Self::ExtrapolationError> {
        Ok(None)
    }
}

