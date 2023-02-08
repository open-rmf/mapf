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

/// The ActionMap trait describes a domain property that can map actions from
/// one domain to another.
pub trait ActionMap<State, FromAction> {
    /// What kind of action is produced after mapping this activity
    type ToAction;

    /// What kind of error can happen if a bad state or action is provided
    type ActionMapError;

    /// Conrete type for the returned container of actions
    type ToActions<'a>: IntoIterator<Item = Result<Self::ToAction, Self::ActionMapError>>
    where
        Self: 'a,
        Self::ToAction: 'a,
        Self::ActionMapError: 'a,
        State: 'a,
        FromAction: 'a;

    /// Implement this function to modify a set of actions.
    fn map_action<'a>(
        &'a self,
        from_state: State,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a,
        State: 'a;
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
    type ActionMapError = NoError;
    type ToActions<'a> = std::option::IntoIter<Result<ToAction, NoError>>
    where
        ToAction: 'a,
        S: 'a,
        FromAction: 'a;

    fn map_action<'a>(
        &'a self,
        _: S,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a,
        S: 'a,
    {
        Some(Ok(from_action.into())).into_iter()
    }
}

/// This struct implements ActionMap for any action that implements Into<Option<ToAction>>
pub struct MaybeActionInto<ToAction> {
    _ignore: std::marker::PhantomData<ToAction>
}

impl<ToAction> MaybeActionInto<ToAction> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}

impl<S, FromAction, ToAction> ActionMap<S, FromAction> for MaybeActionInto<ToAction>
where
    FromAction: Into<Option<ToAction>>,
{
    type ToAction = ToAction;
    type ActionMapError = NoError;
    type ToActions<'a> = std::option::IntoIter<Result<ToAction, NoError>>
    where
        ToAction: 'a,
        S: 'a,
        FromAction: 'a;

    fn map_action<'a>(
        &'a self,
        _: S,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a,
        S: 'a,
    {
        Ok(from_action.into()).transpose().into_iter()
    }
}

/// NoActionMap is a struct that provides a no-op implementation of ActionMap.
/// Used by DomainMap when an ActionMap is not needed.
pub struct NoActionMap;
impl<State, Action> ActionMap<State, Action> for NoActionMap {
    type ActionMapError = NoError;
    type ToAction = Action;
    type ToActions<'a> = Option<Result<Action, NoError>>
    where
        State: 'a,
        Action: 'a;

    fn map_action<'a>(
        &'a self,
        _: State,
        from_action: Action,
    ) -> Self::ToActions<'a>
    where
        Action: 'a,
        State: 'a,
    {
        Some(Ok(from_action))
    }
}
