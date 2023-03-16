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
    domain::{Key, Keyed, Keyring, SelfKey, Space, KeyedSpace, Initializable, Satisfiable},
    motion::{Timed, TimePoint, DEFAULT_ROTATIONAL_THRESHOLD, se2::*, r2::MaybePositioned},
    graph::Graph,
    error::{NoError, ThisError},
};
use std::borrow::Borrow;

#[derive(Debug, Clone, Copy)]
pub struct DiscreteSpaceTimeSE2<Key, const R: u32>(std::marker::PhantomData<Key>);
impl<K, const R: u32> DiscreteSpaceTimeSE2<K, R> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<K, const R: u32> Default for DiscreteSpaceTimeSE2<K, R> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K, const R: u32> Space for DiscreteSpaceTimeSE2<K, R> {
    type State = StateSE2<K, R>;
    type Waypoint = Waypoint;
    type WaypointRef<'a> = &'a Waypoint where K: 'a;
    fn waypoint<'a>(&'a self, state: &'a Self::State) -> Self::WaypointRef<'a> {
        &state.waypoint
    }
}

impl<K: Key + Clone, const R: u32> Keyed for DiscreteSpaceTimeSE2<K, R> {
    type Key = KeySE2<K, R>;
}

impl<K: Key + Clone, const R: u32> Keyring<StateSE2<K, R>> for DiscreteSpaceTimeSE2<K, R> {
    type KeyRef<'a> = &'a Self::Key where K: 'a;
    fn key_for<'a>(&'a self, state: &'a StateSE2<K, R>) -> &'a Self::Key
    where
        K: 'a,
    {
        &state.key
    }
}

impl<K: Key + Clone, const R: u32> KeyedSpace<K> for DiscreteSpaceTimeSE2<K, R> {
    fn make_keyed_state(
        &self,
        vertex: K,
        waypoint: Self::Waypoint,
    ) -> Self::State {
        StateSE2::new(vertex, waypoint)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateSE2<K, const R: u32> {
    pub key: KeySE2<K, R>,
    pub waypoint: Waypoint,
}

impl<K, const R: u32> StateSE2<K, R> {
    pub fn new(
        vertex: K,
        waypoint: Waypoint,
    ) -> Self {
        Self {
            key: KeySE2 {
                vertex,
                orientation: (waypoint.position.rotation.angle() * (R as f64)) as i32,
            },
            waypoint,
        }
    }
}

impl<K, const R: u32> Timed for StateSE2<K, R> {
    fn set_time(&mut self, new_time: TimePoint) {
        self.waypoint.set_time(new_time);
    }

    fn time(&self) -> &TimePoint {
        self.waypoint.time()
    }
}

impl<K: Key + Clone, const R: u32> Keyed for StateSE2<K, R> {
    type Key = KeySE2<K, R>;
}

impl<K: Key + Clone, const R: u32> SelfKey for StateSE2<K, R> {
    type KeyRef<'a> = &'a Self::Key where K: 'a;
    fn key<'a>(&'a self) -> &'a Self::Key
    where
        K: 'a,
    {
        &self.key
    }
}

impl<K, const R: u32> Borrow<K> for StateSE2<K, R> {
    fn borrow(&self) -> &K {
        &self.key.vertex
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KeySE2<K, const R: u32> {
    /// The key for graph vertices
    pub vertex: K,
    /// The key of the agent's orientation. This value should be in the range of
    /// +/- PI * R
    pub orientation: i32,
}

impl<K, const R: u32> Borrow<K> for KeySE2<K, R> {
    fn borrow(&self) -> &K {
        &self.vertex
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StartSE2<K> {
    pub time: TimePoint,
    pub key: K,
    pub orientation: Orientation,
}

impl<K> From<(K, Orientation)> for StartSE2<K> {
    fn from((key, orientation): (K, Orientation)) -> Self {
        StartSE2 { time: TimePoint::zero(), key, orientation }
    }
}

impl<K> From<(K, f64)> for StartSE2<K> {
    fn from((key, angle): (K, f64)) -> Self {
        StartSE2 { time: TimePoint::zero(), key, orientation: Orientation::from_angle(angle) }
    }
}

impl<K> From<(TimePoint, K, Orientation)> for StartSE2<K> {
    fn from((time, key, orientation): (TimePoint, K, Orientation)) -> Self {
        StartSE2 { time, key, orientation }
    }
}

impl<K> From<(TimePoint, K, f64)> for StartSE2<K> {
    fn from((time, key, angle): (TimePoint, K, f64)) -> Self {
        StartSE2 { time, key, orientation: Orientation::from_angle(angle) }
    }
}

pub struct InitializeSE2<G>(pub G);

impl<G, Start, const R: u32> Initializable<Start, StateSE2<G::Key, R>> for InitializeSE2<G>
where
    G: Graph,
    G::Key: Clone,
    G::Vertex: Borrow<Point>,
    Start: Into<StartSE2<G::Key>>,
{
    type InitialError = InitializeSE2Error<G::Key>;
    type InitialStates<'a> = [Result<StateSE2<G::Key, R>, InitializeSE2Error<G::Key>>; 1]
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
        StateSE2<G::Key, R>: 'a,
    {
        let start: StartSE2<G::Key> = from_start.into();
        [
            self.0.vertex(&start.key)
            .ok_or_else(|| InitializeSE2Error::MissingVertex(start.key.clone()))
            .map(|v| {
                let v: &Point = v.borrow().borrow();
                StateSE2::new(
                    start.key,
                    Waypoint::new(start.time, v.x, v.y, start.orientation.angle()),
                )
            })
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatisfySE2 {
    pub rotational_threshold: f64,
}

impl SatisfySE2 {
    pub fn new(rotational_threshold: f64) -> Self {
        Self { rotational_threshold }
    }
}

impl Default for SatisfySE2 {
    fn default() -> Self {
        Self { rotational_threshold: DEFAULT_ROTATIONAL_THRESHOLD }
    }
}

impl From<DifferentialDriveLineFollow> for SatisfySE2 {
    fn from(value: DifferentialDriveLineFollow) -> Self {
        Self::new(value.rotational_threshold())
    }
}

impl<K, const R: u32, Goal> Satisfiable<StateSE2<K, R>, Goal> for SatisfySE2
where
    Goal: SelfKey<Key=K> + MaybeOriented,
    K: PartialEq,
{
    type SatisfactionError = NoError;
    fn is_satisfied(
        &self,
        by_state: &StateSE2<K, R>,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError> {
        if by_state.key.vertex != *for_goal.key().borrow() {
            return Ok(false);
        }

        if let Some(target_yaw) = for_goal.maybe_oriented() {
            let delta_yaw_abs = (target_yaw / by_state.waypoint.position.rotation).angle().abs();
            if delta_yaw_abs > self.rotational_threshold {
                return Ok(false);
            }
        }

        Ok(true)
    }
}

pub struct GoalSE2<K> {
    pub key: K,
    pub orientation: Option<Orientation>,
}

impl<K: Key> Keyed for GoalSE2<K> {
    type Key = K;
}

impl<K: Key> SelfKey for GoalSE2<K> {
    type KeyRef<'a> = &'a Self::Key;
    fn key<'a>(&'a self) -> Self::KeyRef<'a> where Self: 'a {
        &self.key
    }
}

impl<K> MaybeOriented for GoalSE2<K> {
    fn maybe_oriented(&self) -> Option<Orientation> {
        self.orientation
    }
}

impl<K> MaybePositioned for GoalSE2<K> {
    fn maybe_point(&self) -> Option<crate::motion::r2::Point> {
        None
    }
}

#[derive(Debug, ThisError)]
pub enum InitializeSE2Error<K> {
    #[error("The graph was missing the start vertex: {0:?}")]
    MissingVertex(K),
}
