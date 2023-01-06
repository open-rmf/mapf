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

/// The ActionMap trait describes a domain property that can modify the
/// actions produced or used by activities. This can be used to apply
/// constraints or transformations to activities within a domain.
pub trait ActionMap<State, FromAction> {
    /// What kind of action is produced after mapping this activity
    type ToAction;

    /// What kind of error can happen if a bad state or action is provided
    type Error;

    /// Conrete type for the returned container of actions
    type ToActions<'a>: IntoIterator<Item = Result<Self::ToAction, Self::Error>>
    where
        Self: 'a,
        Self::ToAction: 'a,
        Self::Error: 'a,
        State: 'a,
        FromAction: 'a;

    /// Implement this function to modify a set of actions.
    fn map_actions<'a>(
        &'a self,
        from_state: &'a State,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a;
}

/// This struct implements ActionMap for any action that implements Into<ToAction>
pub struct ActionInto<ToAction> {
    _ignore: std::marker::PhantomData<ToAction>,
}

impl<ToAction> ActionInto<ToAction> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}

impl<S, FromAction, ToAction> ActionMap<S, FromAction> for ActionInto<ToAction>
where
    FromAction: Into<ToAction>
{
    type ToAction = ToAction;
    type Error = NoError;
    type ToActions<'a> = std::option::IntoIter<Result<ToAction, NoError>>
    where
        ToAction: 'a,
        S: 'a,
        FromAction: 'a;

    fn map_actions<'a>(
        &'a self,
        _: &'a S,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a,
    {
        Some(Ok(from_action.into())).into_iter()
    }
}

/// This struct implements ActionMap for any action that implements Into<Option<ToAction>>
pub struct ActionMaybeInto<ToAction> {
    _ignore: std::marker::PhantomData<ToAction>
}

impl<ToAction> ActionMaybeInto<ToAction> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}

impl<S, FromAction, ToAction> ActionMap<S, FromAction> for ActionMaybeInto<ToAction>
where
    FromAction: Into<Option<ToAction>>,
{
    type ToAction = ToAction;
    type Error = NoError;
    type ToActions<'a> = std::option::IntoIter<Result<ToAction, NoError>>
    where
        ToAction: 'a,
        S: 'a,
        FromAction: 'a;

    fn map_actions<'a>(
        &'a self,
        _: &'a S,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a,
    {
        Ok(from_action.into()).transpose().into_iter()
    }
}

/// NoActionMap is a trait that provides a no-op implementation of an ActionMap
/// that can be "implemented" by structs that need to have an ActionMap trait
/// but don't need to actually need to map any actions.
pub trait NoActionMap {}
impl<T: NoActionMap, State, Action> ActionMap<State, Action> for T {
    type Error = NoError;
    type ToAction = Action;
    type ToActions<'a> = Option<Result<Action, NoError>>
    where
        T: 'a,
        State: 'a,
        Action: 'a;

    fn map_actions<'a>(
        &'a self,
        _: &'a State,
        from_action: Action,
    ) -> Self::ToActions<'a>
    where
        Action: 'a,
    {
        Some(Ok(from_action))
    }
}
