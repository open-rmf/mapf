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
use std::borrow::Borrow;

/// The `Satisfiable` trait allows search algorithms to recognize when a state
/// has reached a goal.
///
/// See also [`ArrivalKeyring`].
pub trait Satisfiable<State, Goal> {
    type SatisfactionError;

    fn is_satisfied(
        &self,
        by_state: &State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError>;
}

impl<State, Goal> Satisfiable<State, Goal> for ()
where
    State: Borrow<Goal>,
    Goal: PartialEq,
{
    type SatisfactionError = NoError;
    fn is_satisfied(
        &self,
        by_state: &State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError> {
        Ok(*by_state.borrow() == *for_goal)
    }
}

/// As an alternative (or complement) to [`Satisfiable`], `ArrivalKeyring`
/// provides a set of keys, each of which indicates a goal has been reached.
/// This can be used by algorithms to pursue goal states more directly.
pub trait ArrivalKeyring<Key, Goal> {
    type ArrivalKeyError;

    type ArrivalKeys<'a>: IntoIterator<Item=Result<Key, Self::ArrivalKeyError>> + 'a
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
        Key: 'a,
        Goal: 'a;

    fn get_arrival_keys<'a>(
        &'a self,
        goal: &'a Goal,
    ) -> Self::ArrivalKeys<'a>
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
        Key: 'a,
        Goal: 'a;
}

impl<Key, Goal> ArrivalKeyring<Key, Goal> for ()
where
    Goal: Into<Key> + Clone,
{
    type ArrivalKeyError = NoError;
    type ArrivalKeys<'a> = [Result<Key, NoError>; 1]
    where
        Key: 'a,
        Goal: 'a;

    fn get_arrival_keys<'a>(
        &'a self,
        goal: &'a Goal,
    ) -> Self::ArrivalKeys<'a>
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
        Key: 'a,
        Goal: 'a
    {
        [Ok(goal.clone().into())]
    }
}
