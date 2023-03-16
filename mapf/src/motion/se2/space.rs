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
    graph::{Graph, Edge},
    error::{NoError, ThisError},
    util::FlatResultMapTrait,
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

/// Use this to initialize a SE(2) domain for most search algorithms.
///
/// When initializing a [`crate::algorithm::Dijkstra`] algorithm with a
/// [`DifferentialDriveLineFollow`], it is better to use [`InitializeStarburstSE2`]
/// because it will initialize states whose keys are sure to be visited and
/// revisited in Dijkstra searches, making the caching behaviors more effective.
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

/// Use this to initialize a SE(2) domain for [`crate::algorithm::Dijkstra`]
/// search algorithms when using a [`DifferentialDriveLineFollow`].
///
/// For a given graph key, this will initialize multiple states where each
/// state is oriented to face a direction that the robot can move towards. This
/// makes the `Dijkstra` algorithm's caching techniques more effective because
/// each of these states will have keys that will be revisited in future
/// searches instead of having keys with arbitrary orientations.
///
/// The "starburst" name comes from the fact that if you draw the initial states
/// as vectors coming out of the start vertex, it will resemble a starburst
/// pattern.
pub struct InitializeStarburstSE2<G>(pub G);

impl<G, const R: u32> Initializable<G::Key, StateSE2<G::Key, R>> for InitializeStarburstSE2<G>
where
    G: Graph,
    G::Key: Clone,
    G::Vertex: Borrow<Point>,
{
    type InitialError = InitializeSE2Error<G::Key>;
    type InitialStates<'a> = impl Iterator<Item = Result<StateSE2<G::Key, R>, Self::InitialError>> + 'a
    where
        Self: 'a,
        G: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: G::Key,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        G::Key: 'a,
        StateSE2<G::Key, R>: 'a
    {
        let from_start_clone = from_start.clone();
        self.0.vertex(&from_start)
            .ok_or_else(move || InitializeSE2Error::MissingVertex(from_start_clone))
            .flat_result_map(move |from_p_ref| {
                let from_p: Point = *from_p_ref.borrow().borrow();
                let from_start = from_start.clone();
                self.0.edges_from_vertex(&from_start)
                    .into_iter()
                    .map(move |edge| {
                        let from_start = from_start.clone();
                        self.0.vertex(edge.to_vertex())
                            .ok_or_else(|| InitializeSE2Error::MissingVertex(edge.to_vertex().clone()))
                            .map(move |to_p_ref| {
                                let to_p: Point = *to_p_ref.borrow().borrow();
                                (to_p - from_p).try_normalize(1e-6)
                                    .map(|v| v[1].atan2(v[0]))
                                    .unwrap_or(0.0)
                            })
                            .map(move |angle|
                                StateSE2::new(
                                    from_start,
                                    Waypoint::new(
                                        TimePoint::zero(),
                                        from_p.x,
                                        from_p.y,
                                        angle,
                                    )
                                )
                            )
                    })
            })
            .map(|r| r.flatten())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::SimpleGraph,
    };

    #[test]
    fn test_starburst_initialization() {
        /*      5
         *      |
         *      |
         * 0----1----2
         *     / \
         *   /     \
         * 3         4
         */

        let graph = SimpleGraph::from_iters(
            [
                Point::new(-1.0, 0.0), // 0
                Point::new(0.0, 0.0), // 1
                Point::new(1.0, 0.0), // 2
                Point::new(-1.0, -1.0), // 3
                Point::new(1.0, -1.0), // 4
                Point::new(0.0, 1.0), // 5
            ],
            [
                (0, 1, ()), (1, 0, ()),
                (2, 1, ()), (1, 2, ()),
                (3, 1, ()), (1, 3, ()),
                (4, 1, ()), (1, 4, ()),
                (5, 1, ()), (1, 5, ()),
            ]
        );

        let init = InitializeStarburstSE2(graph);
        let states: Result<Vec<StateSE2<usize, 100>>, _> = init.initialize(1).collect();
        let states = states.unwrap();
        assert!(states.len() == 5);
    }
}
