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

use crate::{
    domain::{
        Key, Keyed, Keyring, SelfKey, Space, KeyedSpace,
        Initializable,
    },
    motion::{Timed, TimePoint},
    graph::Graph,
    error::ThisError,
};
use super::{*, timed_position::Waypoint};
use std::{sync::Arc, borrow::Borrow};

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

impl<K> Timed for StateR2<K> {
    fn set_time(&mut self, new_time: TimePoint) {
        self.waypoint.set_time(new_time);
    }

    fn time(&self) -> &TimePoint {
        self.waypoint.time()
    }
}

impl<K: Key + Clone> Keyed for StateR2<K> {
    type Key = K;
}

impl<K: Key + Clone> SelfKey for StateR2<K> {
    fn key(&self) -> Self::Key {
        self.key.clone()
    }
}

pub struct StartR2<K> {
    pub time: TimePoint,
    pub key: K,
}

impl<K> From<(TimePoint, K)> for StartR2<K> {
    fn from((time, key): (TimePoint, K)) -> Self {
        StartR2 { key, time }
    }
}

impl<K> From<K> for StartR2<K> {
    fn from(key: K) -> Self {
        StartR2 { key, time: TimePoint::zero() }
    }
}

pub struct InitializeR2<G>(pub Arc<G>);

impl<G, Start> Initializable<Start, StateR2<G::Key>> for InitializeR2<G>
where
    G: Graph,
    G::Key: Clone,
    G::Vertex: Borrow<Position>,
    Start: Into<StartR2<G::Key>>,
{
    type InitialError = InitializeR2Error<G::Key>;
    type InitialStates<'a> = [Result<StateR2<G::Key>, InitializeR2Error<G::Key>>; 1]
    where
        Self: 'a,
        Start: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
    {
        let start: StartR2<G::Key> = from_start.into();
        [
            self.0.vertex(&start.key)
            .ok_or_else(|| InitializeR2Error::MissingVertex(start.key.clone()))
            .map(|v| {
                let v: &Position = v.borrow().borrow();
                StateR2 {
                    key: start.key,
                    waypoint: Waypoint::new(start.time, v.x, v.y),
                }
            })
        ]
    }
}

#[derive(Debug, ThisError)]
pub enum InitializeR2Error<K> {
    #[error("The graph was missing the start vertex: {0}")]
    MissingVertex(K),
}
