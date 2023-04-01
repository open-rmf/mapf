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
    domain::{Key, Keyed, Keyring, SelfKey, Space, KeyedSpace, Initializable},
    motion::{Timed, TimePoint, IntegrateWaypoints, r2::*, se2::StateSE2},
    graph::Graph,
    error::{ThisError, NoError},
};
use std::borrow::Borrow;

#[derive(Debug)]
pub struct DiscreteSpaceTimeR2<Key>(std::marker::PhantomData<Key>);
impl<Key> DiscreteSpaceTimeR2<Key> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<Key> Clone for DiscreteSpaceTimeR2<Key> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

impl<Key> Copy for DiscreteSpaceTimeR2<Key> {}

impl<Key> Default for DiscreteSpaceTimeR2<Key> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Key> Space for DiscreteSpaceTimeR2<Key> {
    type State = StateR2<Key>;
    type Waypoint = timed_position::WaypointR2;

    type WaypointRef<'a> = &'a Self::Waypoint where Key: 'a;
    fn waypoint<'a>(&'a self, state: &'a Self::State) -> &'a Self::Waypoint {
        &state.waypoint
    }
}

impl<K: Key> Keyed for DiscreteSpaceTimeR2<K> {
    type Key = K;
}

impl<K: Key + Clone> Keyring<StateR2<K>> for DiscreteSpaceTimeR2<K> {
    type KeyRef<'a> = &'a Self::Key
    where
        K: 'a;

    fn key_for<'a>(&'a self, state: &'a StateR2<K>) -> &'a Self::Key
    where
        K: 'a,
    {
        &state.key
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

    fn vertex_of<'a>(
        &'a self,
        key: &'a Self::Key,
    ) -> &'a K
    where
        K: 'a
    {
        key
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StateR2<K> {
    pub key: K,
    pub waypoint: timed_position::WaypointR2,
}

impl<W: From<WaypointR2>, K> IntegrateWaypoints<W> for StateR2<K> {
    type IntegratedWaypointIter<'a> = Option<Result<W, NoError>>
    where
        W: 'a,
        K: 'a;

    type WaypointIntegrationError = NoError;
    fn integrated_waypoints<'a>(
        &'a self,
        _initial_waypoint: Option<W>,
    ) -> Self::IntegratedWaypointIter<'a>
    where
        Self: 'a,
        Self::WaypointIntegrationError: 'a,
        W: 'a,
    {
        // TODO(@mxgrey): Should it be an error if _initial_waypoint is Some?
        // States are only supposed to be integrated into waypoints at the start
        // of a trajectory, so it would be suspicious if we're given an initial
        // waypoint.
        Some(Ok(self.waypoint.into()))
    }
}

impl<K> Borrow<K> for StateR2<K> {
    fn borrow(&self) -> &K {
        &self.key
    }
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
    type KeyRef<'a> = &'a Self::Key
    where
        K: 'a;

    fn key<'a>(&'a self) -> &'a Self::Key
    where
        K: 'a
    {
        &self.key
    }
}

impl<K, const R: u32> From<StateSE2<K, R>> for StateR2<K> {
    fn from(value: StateSE2<K, R>) -> Self {
        StateR2 {
            key: value.key.vertex,
            waypoint: WaypointR2 {
                time: value.waypoint.time,
                position: value.waypoint.position.point(),
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
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

#[derive(Debug, Clone)]
pub struct InitializeR2<G>(pub G);

impl<G, Start, Goal> Initializable<Start, Goal, StateR2<G::Key>> for InitializeR2<G>
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
        Start: 'a,
        Goal: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
        _to_goal: &Goal,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
    {
        let start: StartR2<G::Key> = from_start.into();
        [
            self.0.vertex(&start.key)
            .ok_or_else(|| InitializeR2Error::MissingVertex(start.key.clone()))
            .map(|v| {
                let v: &Position = v.borrow().borrow();
                StateR2 {
                    key: start.key,
                    waypoint: WaypointR2::new(start.time, v.x, v.y),
                }
            })
        ]
    }
}

#[derive(Debug, ThisError)]
pub enum InitializeR2Error<K> {
    #[error("The graph was missing the start vertex: {0:?}")]
    MissingVertex(K),
}
