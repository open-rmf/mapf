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

use std::hash::Hash;

/// Reachable means the domain may be able to quickly find an action or series
/// of actions to connect two states. This can be useful for doing faster
/// searches, or it can be used by bidirectional searches to bridge between
/// forward and reverse search efforts.
//
// TODO(MXG): Should this be split out into two traits instead of one?
pub trait Reachable<State> {
    /// The reachability key determines when two states may be worth attempting
    /// a connection. If the key values are equal then the states may be
    /// connectible. An ideal implementation would never allow connectible
    /// states to have unequal keys.
    type ReachabilityKey: Hash + Eq;

    /// What kind of error can happen while calculating the reachability key.
    type ReachabilityError;

    /// Calculate the reachability key for the state.
    fn reachability_key(
        &self,
        for_state: &State,
    ) -> Result<Option<Self::ReachabilityKey>, Self::ReachabilityError>;

    /// Type of action used during connection
    type ConnectAction;

    /// The actions used to create a connection.
    type Connection<'a>: IntoIterator<Item=Self::ConnectAction>
    where
        Self: 'a,
        Self::ConnectAction: 'a,
        State: 'a;

    /// What kind of error can happen while attempting a connection
    type ConnectionError;

    /// Attempt to form a connection between the two given states.
    fn attempt_connection<'a>(
        &'a self,
        from_state: &'a State,
        to_state: &'a State,
    ) -> Result<Option<Self::Connection<'a>>, Self::ConnectionError>;
}
