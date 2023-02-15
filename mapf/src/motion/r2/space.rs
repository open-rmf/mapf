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

use crate::domain::{
    keyed::{Key, Keyed, Keyring},
    space::{Space, KeyedSpace},
};
use super::*;

#[derive(Debug, Clone, Copy)]
pub struct DiscreteSpaceTimeR2<Key>(std::marker::PhantomData<Key>);
impl<Key> DiscreteSpaceTimeR2<Key> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<Key> Default for DiscreteSpaceTimeR2<Key> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Key> Space for DiscreteSpaceTimeR2<Key> {
    type State = StateR2<Key>;
    type Waypoint = timed_position::Waypoint;

    type WaypointRef<'a> = &'a Self::Waypoint where Key: 'a;
    fn waypoint<'a>(&'a self, state: &'a Self::State) -> &'a Self::Waypoint {
        &state.waypoint
    }
}

impl<K: Key> Keyed for DiscreteSpaceTimeR2<K> {
    type Key = K;
}

impl<K: Key + Clone> Keyring<StateR2<K>> for DiscreteSpaceTimeR2<K> {
    fn key_for(&self, state: &StateR2<K>) -> Self::Key {
        state.key.clone()
    }
}

impl<K: Key + Clone> KeyedSpace<K> for DiscreteSpaceTimeR2<K> {
    fn make_keyed_state(
        &self,
        key: K,
        waypoint: Self::Waypoint,
    ) -> Self::State {
        StateR2 { key, waypoint }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StateR2<K> {
    pub key: K,
    pub waypoint: timed_position::Waypoint,
}
