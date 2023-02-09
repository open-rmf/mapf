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

/// Create actions that connect a state to a target. May produce multiple
/// actions, each of which represents a valid connection. If no actions are
/// produced, then a connection was not possible.
///
/// See also: [`Extrapolator`]
pub trait Connectable<State, Target> {
    /// What kind of action is produced during connection
    type Connection;

    /// What kind of error can happen during connection
    type ConnectionError;

    type Connections<'a>: IntoIterator<Item = Result<(Self::Connection, State), Self::ConnectionError>>
    where
        Self: 'a,
        Self::Connection: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Target: 'a;

    fn connect<'a>(
        &'a self,
        from_state: State,
        to_target: &'a Target,
    ) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::Connection: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Target: 'a;
}
