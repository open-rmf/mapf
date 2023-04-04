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

use super::*;
use crate::error::NoError;

/// Create actions that connect a state to a target. May produce multiple
/// actions, each of which represents a valid connection. If no actions are
/// produced, then a connection was not possible.
///
/// See also: [`Extrapolator`]
pub trait Connectable<State, Action, Target> {
    /// What kind of error can happen during connection
    type ConnectionError;

    type Connections<'a>: IntoIterator<Item = Result<(Action, State), Self::ConnectionError>> + 'a
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Target: 'a;

    fn connect<'a>(&'a self, from_state: State, to_target: &'a Target) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Target: 'a;
}

// Allow an empty tuple to implement the Connectable trait by never attempting
// any connection.
impl<State, Action, Target> Connectable<State, Action, Target> for () {
    type ConnectionError = NoError;
    type Connections<'a> = [Result<(Action, State), NoError>; 0]
    where
        Action: 'a,
        State: 'a,
        Target: 'a;

    fn connect<'a>(&'a self, _: State, _: &'a Target) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Target: 'a,
    {
        []
    }
}

impl<Base, Prop, State, Action, Target> Connectable<State, Action, Target> for Chained<Base, Prop>
where
    Base: Connectable<State, Action, Target>,
    Base::ConnectionError: Into<Prop::ConnectionError>,
    Prop: Connectable<State, Action, Target>,
    State: Clone,
{
    type ConnectionError = Prop::ConnectionError;
    type Connections<'a> = impl IntoIterator<Item = Result<(Action, State), Self::ConnectionError>> + 'a
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Target: 'a;

    fn connect<'a>(&'a self, from_state: State, to_target: &'a Target) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Target: 'a,
    {
        self.base
            .connect(from_state.clone(), to_target)
            .into_iter()
            .map(|r| {
                r.map(|(action, state)| (action.into(), state))
                    .map_err(Into::into)
            })
            .chain(self.prop.connect(from_state, to_target).into_iter())
    }
}
