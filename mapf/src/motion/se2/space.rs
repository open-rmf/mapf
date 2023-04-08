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
        ArrivalKeyring, Initializable, Key, Keyed, KeyedSpace, Keyring, Reversible, Satisfiable,
        SelfKey, Space,
    },
    error::{NoError, ThisError},
    graph::{Edge, Graph},
    motion::{
        r2::{MaybePositioned, Point, Positioned},
        se2::*,
        IntegrateWaypoints, TimePoint, Timed, MaybeTimed, DEFAULT_ROTATIONAL_THRESHOLD,
    },
    util::{wrap_to_pi, FlatResultMapTrait},
};
use smallvec::SmallVec;
use std::borrow::Borrow;

#[derive(Debug)]
pub struct DiscreteSpaceTimeSE2<Key, const R: u32>(std::marker::PhantomData<Key>);
impl<K, const R: u32> DiscreteSpaceTimeSE2<K, R> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<K, const R: u32> Clone for DiscreteSpaceTimeSE2<K, R> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

impl<K, const R: u32> Copy for DiscreteSpaceTimeSE2<K, R> {}

impl<K, const R: u32> Default for DiscreteSpaceTimeSE2<K, R> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K, const R: u32> Space for DiscreteSpaceTimeSE2<K, R> {
    type State = StateSE2<K, R>;
    type Waypoint = WaypointSE2;
    type WaypointRef<'a> = &'a WaypointSE2 where K: 'a;
    fn waypoint<'a>(&'a self, state: &'a Self::State) -> Self::WaypointRef<'a> {
        &state.waypoint
    }
}

impl<K: Key + Clone, const R: u32> Keyed for DiscreteSpaceTimeSE2<K, R> {
    type Key = KeySE2<K, R>;
}

impl<K: Key + Clone, State, const R: u32> Keyring<State> for DiscreteSpaceTimeSE2<K, R>
where
    State: Borrow<StateSE2<K, R>>,
{
    type KeyRef<'a> = &'a Self::Key where K: 'a, State: 'a;
    fn key_for<'a>(&'a self, state: &'a State) -> &'a Self::Key
    where
        K: 'a,
        State: 'a,
    {
        &state.borrow().key
    }
}

impl<K: Key + Clone, const R: u32> KeyedSpace<K> for DiscreteSpaceTimeSE2<K, R> {
    fn make_keyed_state(&self, vertex: K, waypoint: Self::Waypoint) -> Self::State {
        StateSE2::new(vertex, waypoint)
    }

    fn vertex_of<'a>(&'a self, key: &'a Self::Key) -> &'a K
    where
        K: 'a,
    {
        &key.vertex
    }
}

impl<K, const R: u32> Reversible for DiscreteSpaceTimeSE2<K, R> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(self.clone())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateSE2<K, const R: u32> {
    pub key: KeySE2<K, R>,
    pub waypoint: WaypointSE2,
}

impl<K, const R: u32> Positioned for StateSE2<K, R> {
    fn point(&self) -> Point {
        self.waypoint.point()
    }
}

impl<K, const R: u32> MaybePositioned for StateSE2<K, R> {
    fn maybe_point(&self) -> Option<Point> {
        Some(self.point())
    }
}

impl<K, const R: u32> Oriented for StateSE2<K, R> {
    fn oriented(&self) -> Orientation {
        self.waypoint.oriented()
    }
}

impl<K, const R: u32> MaybeOriented for StateSE2<K, R> {
    fn maybe_oriented(&self) -> Option<Orientation> {
        self.waypoint.maybe_oriented()
    }
}

impl<W: From<WaypointSE2>, K, const R: u32> IntegrateWaypoints<W> for StateSE2<K, R> {
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

impl<K, const R: u32> StateSE2<K, R> {
    pub fn new(vertex: K, waypoint: WaypointSE2) -> Self {
        Self {
            key: KeySE2::new(vertex, waypoint.position.rotation.angle()),
            waypoint,
        }
    }
}

impl<K, const R: u32> Timed for StateSE2<K, R> {
    fn set_time(&mut self, new_time: TimePoint) {
        self.waypoint.set_time(new_time);
    }

    fn time(&self) -> TimePoint {
        self.waypoint.time()
    }
}

impl<K, const R: u32> MaybeTimed for StateSE2<K, R> {
    fn maybe_time(&self) -> Option<TimePoint> {
        Some(self.waypoint.time())
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

impl<K, const R: u32> KeySE2<K, R> {
    pub fn new(vertex: K, angle: f64) -> Self {
        let angle = wrap_to_pi(angle);
        let mut orientation = (angle * (R as f64)) as i32;
        if orientation == -(std::f64::consts::PI * (R as f64)) as i32 {
            // If the orientation is in the range of -PI, then flip it so that
            // its key matches +PI.
            orientation = -orientation;
        }
        Self {
            vertex,
            orientation,
        }
    }

    pub fn orientation(&self) -> Orientation {
        Orientation::from_angle(self.orientation as f64 / R as f64)
    }
}

impl<K, const R: u32> Borrow<K> for KeySE2<K, R> {
    fn borrow(&self) -> &K {
        &self.vertex
    }
}

impl<K, const R: u32> MaybeOriented for KeySE2<K, R> {
    fn maybe_oriented(&self) -> Option<Orientation> {
        Some(self.orientation())
    }
}

impl<K, const R: u32> MaybePositioned for KeySE2<K, R> {
    fn maybe_point(&self) -> Option<crate::motion::r2::Point> {
        None
    }
}

impl<K, const R: u32> MaybeTimed for KeySE2<K, R> {
    fn maybe_time(&self) -> Option<TimePoint> {
        None
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
        StartSE2 {
            time: TimePoint::zero(),
            key,
            orientation,
        }
    }
}

impl<K> From<(K, f64)> for StartSE2<K> {
    fn from((key, angle): (K, f64)) -> Self {
        StartSE2 {
            time: TimePoint::zero(),
            key,
            orientation: Orientation::from_angle(angle),
        }
    }
}

impl<K> From<(TimePoint, K, Orientation)> for StartSE2<K> {
    fn from((time, key, orientation): (TimePoint, K, Orientation)) -> Self {
        StartSE2 {
            time,
            key,
            orientation,
        }
    }
}

impl<K> From<(TimePoint, K, f64)> for StartSE2<K> {
    fn from((time, key, angle): (TimePoint, K, f64)) -> Self {
        StartSE2 {
            time,
            key,
            orientation: Orientation::from_angle(angle),
        }
    }
}

/// Use this to initialize a SE(2) domain for most search algorithms.
///
/// When initializing a [`crate::algorithm::Dijkstra`] algorithm with a
/// [`DifferentialDriveLineFollow`], it is better to use [`InitializeStarburstSE2`]
/// because it will initialize states whose keys are sure to be visited and
/// revisited in Dijkstra searches, making the caching behaviors more effective.
#[derive(Debug, Clone)]
pub struct InitializeSE2<G, const R: u32>(pub G);

impl<G, State, Start, Goal, const R: u32> Initializable<Start, Goal, State> for InitializeSE2<G, R>
where
    G: Graph,
    G::Key: Clone,
    G::Vertex: Positioned,
    Start: Into<StartSE2<G::Key>>,
    StateSE2<G::Key, R>: Into<State>,
{
    type InitialError = InitializeSE2Error<G::Key>;
    type InitialStates<'a> = [Result<State, InitializeSE2Error<G::Key>>; 1]
    where
        Self: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: Start, _to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a,
        StateSE2<G::Key, R>: 'a,
    {
        let start: StartSE2<G::Key> = from_start.into();
        [self
            .0
            .vertex(&start.key)
            .ok_or_else(|| InitializeSE2Error::MissingVertex(start.key.clone()))
            .map(|v| {
                let v = v.borrow().point();
                StateSE2::new(
                    start.key,
                    WaypointSE2::new(start.time, v.x, v.y, start.orientation.angle()),
                )
                .into()
            })]
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
#[derive(Debug, Clone)]
pub struct StarburstSE2<G, const R: u32> {
    pub graph: G,
    pub reverse: Option<G>,
    pub direction: f32,
}

impl<G, const R: u32> StarburstSE2<G, R> {
    pub fn for_start(graph: G) -> Self {
        Self {
            graph,
            reverse: None,
            direction: 1.0,
        }
    }

    pub fn for_goal(graph: G) -> Result<Self, G::ReversalError>
    where
        G: Reversible,
    {
        let reverse = graph.reversed()?;
        Ok(Self {
            graph,
            reverse: Some(reverse),
            direction: 1.0,
        })
    }

    fn starburst_impl<'a>(
        from_vertex: G::Key,
        to_vertex: G::Key,
        graph: &'a G,
        direction: f32,
    ) -> impl Iterator<Item = Result<(Point, f64), InitializeSE2Error<G::Key>>> + 'a
    where
        G: Graph,
        G::Key: Clone,
        G::Vertex: Positioned,
    {
        let from_start_clone = from_vertex.clone();
        graph
            .vertex(&from_vertex)
            .ok_or_else(move || InitializeSE2Error::MissingVertex(from_start_clone))
            .flat_result_map(move |from_p_ref| {
                let from_p: Point = from_p_ref.borrow().point();
                let from_start = from_vertex.clone();
                graph
                    .edges_from_vertex(&from_start)
                    .into_iter()
                    .chain(graph.lazy_edges_between(&from_vertex, &to_vertex))
                    .map(move |edge| {
                        graph
                            .vertex(edge.to_vertex())
                            .ok_or_else(|| {
                                InitializeSE2Error::MissingVertex(edge.to_vertex().clone())
                            })
                            .map(move |to_p_ref| {
                                let to_p: Point = to_p_ref.borrow().point();
                                (direction as f64 * (to_p - from_p))
                                    .try_normalize(1e-6)
                                    .map(|v| f64::atan2(v[1], v[0]))
                                    .unwrap_or(0.0)
                            })
                            .map(move |angle| (from_p, angle))
                    })
            })
            .map(|r| r.flatten())
    }

    fn starburst<'a>(
        &'a self,
        from_vertex: G::Key,
        to_vertex: G::Key,
    ) -> impl Iterator<Item = Result<(Point, f64), InitializeSE2Error<G::Key>>> + 'a
    where
        G: Graph,
        G::Key: Clone,
        G::Vertex: Positioned,
    {
        Self::starburst_impl(
            from_vertex.clone(),
            to_vertex.clone(),
            &self.graph,
            self.direction,
        )
        .chain(self.reverse.as_ref().into_iter().flat_map(move |r_graph| {
            Self::starburst_impl(
                from_vertex.clone(),
                to_vertex.clone(),
                &r_graph,
                -1.0 * self.direction,
            )
        }))
    }
}

impl<G, const R: u32, Start, Goal, State> Initializable<Start, Goal, State> for StarburstSE2<G, R>
where
    G: Graph,
    G::Key: Clone,
    G::Vertex: Positioned,
    Start: Borrow<G::Key>,
    Goal: Borrow<G::Key>,
    StateSE2<G::Key, R>: Into<State>,
{
    type InitialError = InitializeSE2Error<G::Key>;
    type InitialStates<'a> = impl Iterator<Item = Result<State, Self::InitialError>> + 'a
    where
        Self: 'a,
        G: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: Start, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        G::Key: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a,
    {
        let from_start = from_start.borrow().clone();
        self.starburst(from_start.clone(), to_goal.borrow().clone())
            .map(move |r| {
                let from_start = from_start.clone();
                r.map(move |(p, angle)| {
                    StateSE2::new(
                        from_start.clone(),
                        WaypointSE2::new(TimePoint::zero(), p.x, p.y, angle),
                    )
                })
            })
            .map(|r| r.map(Into::into))
    }
}

impl<G: Graph, const R: u32, Start, Goal> ArrivalKeyring<KeySE2<G::Key, R>, Start, Goal>
    for StarburstSE2<G, R>
where
    G::Key: Clone,
    G::Vertex: Positioned,
    Start: Borrow<G::Key>,
    Goal: Borrow<G::Key>,
{
    type ArrivalKeyError = InitializeSE2Error<G::Key>;
    type ArrivalKeys<'a> = impl Iterator<Item = Result<KeySE2<G::Key, R>, Self::ArrivalKeyError>> + 'a
    where
        Self: 'a,
        G: 'a,
        G::Key: 'a,
        Start: 'a,
        Goal: 'a;

    fn get_arrival_keys<'a>(&'a self, start: &Start, goal: &Goal) -> Self::ArrivalKeys<'a>
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
    {
        let goal = goal.borrow().clone();
        self
            // goal and start are intentionally swapped because Starburst::for_goal
            // reverses the graph, allowing us to query for edges that lead into the
            // goal.
            .starburst(goal.clone(), start.borrow().clone())
            .map(move |r| r.map(|(_, angle)| KeySE2::new(goal.clone(), angle)))
    }
}

impl<G: Reversible, const R: u32> Reversible for StarburstSE2<G, R> {
    type ReversalError = G::ReversalError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        let reverse = match &self.reverse {
            Some(r) => Some(r.reversed()?),
            None => None,
        };

        Ok(Self {
            graph: self.graph.reversed()?,
            reverse,
            direction: -1.0 * self.direction,
        })
    }
}

#[derive(Debug, Clone)]
pub struct PreferentialStarburstSE2<G, const R: u32> {
    pub starburst: StarburstSE2<G, R>,
}

impl<G, const R: u32> PreferentialStarburstSE2<G, R> {
    pub fn for_start(graph: G) -> Self {
        Self {
            starburst: StarburstSE2::for_start(graph),
        }
    }

    pub fn for_goal(graph: G) -> Result<Self, G::ReversalError>
    where
        G: Reversible,
    {
        Ok(Self {
            starburst: StarburstSE2::for_goal(graph)?,
        })
    }
}

impl<G: Graph, const R: u32, State> Initializable<G::Key, KeySE2<G::Key, R>, State>
    for PreferentialStarburstSE2<G, R>
where
    G::Key: Clone + std::cmp::PartialEq,
    G::Vertex: Positioned,
    StateSE2<G::Key, R>: Into<State>,
{
    type InitialError = InitializeSE2Error<G::Key>;
    type InitialStates<'a> = impl IntoIterator<Item = Result<State, Self::InitialError>> + 'a
    where
        Self: 'a,
        G: 'a,
        State: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: G::Key,
        to_goal: &KeySE2<G::Key, R>,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        G::Key: 'a,
        KeySE2<G::Key, R>: 'a,
        State: 'a,
    {
        let from_start_clone = from_start.clone();
        let mut all_states: SmallVec<[Result<_, _>; 16]> = self
            .starburst
            .starburst(from_start.clone(), to_goal.vertex.clone())
            .map(move |r| {
                let from_start = from_start.clone();
                r.map(move |(p, angle)| {
                    StateSE2::new(
                        from_start.clone(),
                        WaypointSE2::new(TimePoint::zero(), p.x, p.y, angle),
                    )
                })
            })
            .collect();

        let from_start = from_start_clone;
        if to_goal.vertex == from_start {
            let mut has_preferred_key = false;
            for state in &all_states {
                if let Ok(state) = state {
                    if state.key == *to_goal {
                        has_preferred_key = true;
                        break;
                    }
                }
            }

            if !has_preferred_key {
                // Insert the prefered key so the planner doesn't think it needs
                // to rotate
                if let Some(v) = self.starburst.graph.vertex(&from_start) {
                    let p = v.borrow().point();
                    all_states.push(Ok(StateSE2 {
                        key: to_goal.clone(),
                        waypoint: WaypointSE2::new(
                            TimePoint::zero(),
                            p.x,
                            p.y,
                            to_goal.orientation().angle(),
                        ),
                    }));
                } else {
                    all_states.push(Err(InitializeSE2Error::MissingVertex(from_start)));
                }
            }
        }

        all_states.into_iter().map(|r| r.map(Into::into))
    }
}

impl<G: Graph, const R: u32> ArrivalKeyring<KeySE2<G::Key, R>, G::Key, KeySE2<G::Key, R>>
    for PreferentialStarburstSE2<G, R>
where
    G::Key: Clone + std::cmp::PartialEq,
    G::Vertex: Positioned,
{
    type ArrivalKeyError = InitializeSE2Error<G::Key>;
    type ArrivalKeys<'a> = impl IntoIterator<Item = Result<KeySE2<G::Key, R>, Self::ArrivalKeyError>> + 'a
    where
        Self: 'a,
        G: 'a,
        G::Key: 'a;

    fn get_arrival_keys<'a>(
        &'a self,
        start: &G::Key,
        goal: &KeySE2<G::Key, R>,
    ) -> Self::ArrivalKeys<'a>
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
        KeySE2<G::Key, R>: 'a,
        G::Key: 'a,
    {
        let goal_vertex_key = &goal.vertex;
        let mut all_keys: SmallVec<[Result<_, _>; 16]> = self
            .starburst
            .get_arrival_keys(start, goal_vertex_key)
            .into_iter()
            .collect();

        // Look through all the keys to see if one of them matches the goal
        // exactly. If it does then we should simply use the exact goal key.
        // This minimizes cost estimate inaccuracies that result from ignoring
        // the rotation component.
        let mut has_preferred_key = false;
        for key in &all_keys {
            if let Ok(key) = key {
                if *key == *goal {
                    has_preferred_key = true;
                    break;
                }
            }
        }

        if has_preferred_key {
            all_keys.retain(|r| r.as_ref().ok().filter(|k| **k == *goal).is_some());
        }

        all_keys
    }
}

impl<G: Reversible, const R: u32> Reversible for PreferentialStarburstSE2<G, R> {
    type ReversalError = G::ReversalError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        Ok(Self {
            starburst: self.starburst.reversed()?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatisfySE2 {
    pub rotational_threshold: f64,
}

impl SatisfySE2 {
    pub fn new(rotational_threshold: f64) -> Self {
        Self {
            rotational_threshold,
        }
    }
}

impl Default for SatisfySE2 {
    fn default() -> Self {
        Self {
            rotational_threshold: DEFAULT_ROTATIONAL_THRESHOLD,
        }
    }
}

impl From<DifferentialDriveLineFollow> for SatisfySE2 {
    fn from(value: DifferentialDriveLineFollow) -> Self {
        Self::new(value.rotational_threshold())
    }
}

impl<K, const R: u32, Goal> Satisfiable<StateSE2<K, R>, Goal> for SatisfySE2
where
    Goal: SelfKey<Key = K> + MaybeOriented + MaybeTimed,
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
            let delta_yaw_abs = (target_yaw / by_state.waypoint.position.rotation)
                .angle()
                .abs();
            if delta_yaw_abs > self.rotational_threshold {
                return Ok(false);
            }
        }

        if let Some(target_time) = for_goal.maybe_time() {
            if by_state.time() < target_time {
                return Ok(false);
            }
        }

        Ok(true)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GoalSE2<K> {
    pub key: K,
    pub orientation: Option<Orientation>,
    pub minimum_time: Option<TimePoint>,
}

impl<K> GoalSE2<K> {
    pub fn new(key: K) -> Self {
        Self {
            key,
            orientation: None,
            minimum_time: None,
        }
    }

    pub fn with_key(mut self, key: K) -> Self {
        self.key = key;
        self
    }

    pub fn with_orientation(mut self, orientation: Option<Orientation>) -> Self {
        self.orientation = orientation;
        self
    }

    pub fn with_minimum_time(mut self, minimum_time: Option<TimePoint>) -> Self {
        self.minimum_time = minimum_time;
        self
    }
}

impl<K: Key> Keyed for GoalSE2<K> {
    type Key = K;
}

impl<K: Key> SelfKey for GoalSE2<K> {
    type KeyRef<'a> = &'a Self::Key;
    fn key<'a>(&'a self) -> Self::KeyRef<'a>
    where
        Self: 'a,
    {
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

impl<K> MaybeTimed for GoalSE2<K> {
    fn maybe_time(&self) -> Option<TimePoint> {
        self.minimum_time
    }
}

impl<K> Borrow<K> for GoalSE2<K> {
    fn borrow(&self) -> &K {
        &self.key
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
    use crate::graph::SimpleGraph;

    #[test]
    fn test_starburst_initialization() {
        /*
         *      5
         *      |
         *      |
         * 0----1----2
         *     / \
         *   /     \
         * 3         4
         *
         */

        let graph = SimpleGraph::from_iters(
            [
                Point::new(-1.0, 0.0),  // 0
                Point::new(0.0, 0.0),   // 1
                Point::new(1.0, 0.0),   // 2
                Point::new(-1.0, -1.0), // 3
                Point::new(1.0, -1.0),  // 4
                Point::new(0.0, 1.0),   // 5
            ],
            [
                (0, 1, ()),
                (1, 0, ()),
                (2, 1, ()),
                (1, 2, ()),
                (3, 1, ()),
                (1, 3, ()),
                (4, 1, ()), // (1, 4, ()),
                (5, 1, ()), // (1, 5, ()),
            ],
        );

        let arrival = StarburstSE2::for_goal(graph.clone()).unwrap();
        let goal_keys: Result<Vec<KeySE2<usize, 100>>, _> =
            arrival.get_arrival_keys(&4, &1).collect();
        let goal_keys = goal_keys.unwrap();
        assert_eq!(goal_keys.len(), 8);

        let init = StarburstSE2::for_start(graph);
        let states: Result<Vec<StateSE2<usize, 100>>, _> = init.initialize(1, &3).collect();
        let states = states.unwrap();
        assert_eq!(states.len(), 3);
    }
}
