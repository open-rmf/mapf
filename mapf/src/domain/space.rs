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

use crate::domain::{Keyed, Keyring};
use std::borrow::Borrow;

/// The `Space` trait describes the data structures of a state space
pub trait Space {
    /// The domain state that is searched over
    type State;

    /// The portion of the search state that describes the physical aspects of
    /// the state
    type Waypoint;

    /// The return type that provides access to waypoint data.
    type WaypointRef<'a>: Borrow<Self::Waypoint> + 'a
    where
        Self: 'a,
        Self::Waypoint: 'a;

    /// Extract a waypoint for the given state
    fn waypoint<'a>(&'a self, state: &'a Self::State) -> Self::WaypointRef<'a>;
}

/// [`KeyedSpace`] is a trait for describing spaces whose states can be
/// assigned a unique key.
pub trait KeyedSpace<K>: Space + Keyed<Key=K> + Keyring<Self::State> {
    fn make_keyed_state(
        &self,
        key: K,
        waypoint: Self::Waypoint,
    ) -> Self::State;
}
