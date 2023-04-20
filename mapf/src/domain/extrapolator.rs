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
/// See also:
/// * [`IncrementalExtrapolator`]
/// * [`crate::domain::ConflictAvoider`]
/// * [`crate::domain::Connectable`]
pub trait Extrapolator<State, Target, Guidance, Key> {
    /// What kind of action is produced during extrapolation
    type Extrapolation;

    /// What kind of error can happen during extrapolation
    type ExtrapolationError;

    /// Iterator for all the extrapolations provided by this extrapolator. Each
    /// extrapolation in the iterator is an alternative way to move the agent
    /// from the start state to the target. Each alternative extrapolation
    /// should be considered independently of each other.
    ///
    /// If the target cannot be reached this will return an empty iterator. If
    /// more than one extrapolation is provided, the meaning of the ordering is
    /// implementation-defined (if the ordering has any meaning at all).
    type ExtrapolationIter<'a>: IntoIterator<Item = Result<(Self::Extrapolation, State), Self::ExtrapolationError>>
        + 'a
    where
        Self: 'a,
        Self::Extrapolation: 'a,
        Self::ExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    /// Extrapolate an action from a state to a target with the provided guidance.
    /// For each alternative action that the agent can use to reach the target,
    /// return the action that has been extrapolated as well as the final state
    /// once the target is reached. If the target cannot be reached, return an
    /// empty iterator.
    ///
    /// ### Arguments
    ///
    /// * `from_state` - The initial state to extrapolate from.
    /// * `to_target` - The target to extrapolate towards. This can be a different
    /// type than the `State`. For example, if `State` is a position in SE2, then
    /// the `Target` could be a position in R2.
    /// * `with_guidance` - Parameters to describe how the extrapolation should
    /// be performed. This can include constraints like speed limits.
    /// * `for_keys` - The keys for the start vertex and target vertex, if applicable.
    fn extrapolate<'a>(
        &'a self,
        from_state: &State,
        to_target: &Target,
        with_guidance: &Guidance,
        for_keys: (Option<&Key>, Option<&Key>),
    ) -> Self::ExtrapolationIter<'a>
    where
        Self: 'a,
        Self::Extrapolation: 'a,
        Self::ExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;
}

/// Incrementally extrapolate an action from a state to a target with the
/// provided guidance.
///
/// This is similar to [`Extrapolator`] except the action from the initial
/// state to the target can be broken down into a sequence of incremental
/// actions and states. In [`Extrapolator`] the action must bring the state
/// all the way to the target, but that is not a requirement for
/// `IncrementalExtrapolator`.
///
/// This can be useful for domain+algorithm combinations that benefit from
/// more granularity in the actions taken by the agent. For example, a
/// SE(2) agent using Dijkstra's algorithm can cover more states in its
/// search cache by breaking down rotations and translations into separate
/// search nodes. Single-use algorithms like A* do not benefit from covering
/// more states in its search, and should therefore prefer [`Extrapolator`].
///
/// See also:
/// * [`Extrapolator`]
/// * [`crate::domain::ConflictAvoider`]
/// * [`crate::domain::Connectable`]
pub trait IncrementalExtrapolator<State, Target, Guidance, Key> {
    /// What kind of action is produced during extrapolation
    type IncrementalExtrapolation;

    /// What kind of error can happen during extrapolation
    type IncrementalExtrapolationError;

    /// Iterator for all the extrapolations provided by this extrapolator. Each
    /// extrapolation in the iterator is an alternative way to move the agent
    /// from the start state to the target. Each alternative extrapolation
    /// should be considered independently of each other.
    ///
    /// If the target cannot be reached this will return an empty iterator. If more
    /// than one extrapolation is provided, the meaning of the ordering is
    /// implementation-defined (if the ordering has any meaning at all).
    type IncrementalExtrapolationIter<'a>: IntoIterator<
            Item = Result<
                (Self::IncrementalExtrapolation, State, ExtrapolationProgress),
                Self::IncrementalExtrapolationError,
            >,
        > + 'a
    where
        Self: 'a,
        Self::IncrementalExtrapolation: 'a,
        Self::IncrementalExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    /// Extrapolate an action from a state towards a target with the provided
    /// guidance. For each alternative action that the agent can use to move
    /// towards the target, return the action that has been extrapolated as well
    /// as the state of the agent at the end of the action. If there is no way
    /// to progress towards the target, return an empty iterator.
    ///
    /// * `from_state` - The initial state to extrapolate from.
    /// * `to_target` - The target to extrapolate towards. This can be a different
    /// type than the `State`. For example, if `State` is a position in SE2, then
    /// the `Target` could be a position in R2.
    /// * `with_guidance` - Parameters to describe how the extrapolation should
    /// be performed. This can include constraints like speed limits.
    fn incremental_extrapolate<'a>(
        &'a self,
        from_state: &State,
        to_target: &Target,
        with_guidance: &Guidance,
        for_keys: (Option<&Key>, Option<&Key>),
    ) -> Self::IncrementalExtrapolationIter<'a>
    where
        Self: 'a,
        Self::IncrementalExtrapolation: 'a,
        Self::IncrementalExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;
}

#[derive(Debug, Clone, Copy)]
pub enum ExtrapolationProgress {
    Incomplete,
    Arrived,
}

impl ExtrapolationProgress {
    pub fn incomplete(&self) -> bool {
        matches!(self, ExtrapolationProgress::Incomplete)
    }

    pub fn arrived(&self) -> bool {
        matches!(self, ExtrapolationProgress::Arrived)
    }
}

pub struct NoExtrapolation<E>(std::marker::PhantomData<E>);
impl<State, Target, Guidance, Key, E> Extrapolator<State, Target, Guidance, Key>
    for NoExtrapolation<E>
{
    type Extrapolation = E;
    type ExtrapolationError = NoError;
    type ExtrapolationIter<'a> = [Result<(E, State), NoError>; 0]
    where
        E: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    fn extrapolate<'a>(
        &'a self,
        _: &State,
        _: &Target,
        _: &Guidance,
        _: (Option<&Key>, Option<&Key>),
    ) -> [Result<(E, State), NoError>; 0]
    where
        E: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
    {
        []
    }
}

impl<State, Target, Guidance, Key, E> IncrementalExtrapolator<State, Target, Guidance, Key>
    for NoExtrapolation<E>
{
    type IncrementalExtrapolation = E;
    type IncrementalExtrapolationError = NoError;
    type IncrementalExtrapolationIter<'a> = [Result<(E, State, ExtrapolationProgress), NoError>; 0]
    where
        E: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    fn incremental_extrapolate<'a>(
        &'a self,
        _: &State,
        _: &Target,
        _: &Guidance,
        _: (Option<&Key>, Option<&Key>),
    ) -> [Result<(E, State, ExtrapolationProgress), NoError>; 0]
    where
        E: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
    {
        []
    }
}
