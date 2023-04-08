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
        Activity, Closable, CloseResult, ClosedSet, ClosedStatus, Domain, Key, Keyed, Keyring,
    },
    error::ThisError,
    graph::{Edge, Graph},
    motion::{
        compute_safe_arrival_path, compute_safe_arrival_times, compute_safe_linear_path_wait_hints,
        is_safe_segment,
        r2::{Positioned, WaypointR2},
        se2::{
            DifferentialDriveLineFollow, DifferentialDriveLineFollowError, KeySE2, MaybeOriented,
            Position, StateSE2, WaypointSE2,
        },
        Duration, Environment, OverlayedDynamicEnvironment, SafeAction, SafeArrivalTimes,
        SpeedLimiter, TimePoint, Timed, WaitForObstacle,
    },
    util::{FlatResultMapTrait, ForkIter, Minimum},
};
use smallvec::SmallVec;
use std::{
    borrow::Borrow,
    collections::{hash_map::Entry, HashMap},
    sync::{Arc, RwLock},
};

pub struct SafeIntervalCache<G: Graph> {
    graph: G,
    environment: Arc<OverlayedDynamicEnvironment<WaypointSE2>>,
    earliest_time: Option<TimePoint>,
    safe_intervals: RwLock<HashMap<G::Key, SafeArrivalTimes>>,
}

impl<G: Graph> SafeIntervalCache<G> {
    pub fn new(environment: Arc<OverlayedDynamicEnvironment<WaypointSE2>>, graph: G) -> Self {
        let mut earliest_time = Minimum::new(|a: &TimePoint, b: &TimePoint| a.cmp(b));
        for obs in environment.obstacles() {
            if let Some(traj) = obs.trajectory() {
                earliest_time.consider(&traj.initial_motion_time());
            }
        }

        let earliest_time = earliest_time.result();
        Self {
            environment,
            graph,
            earliest_time,
            safe_intervals: RwLock::new(HashMap::new()),
        }
    }

    pub fn graph(&self) -> &G {
        &self.graph
    }

    pub fn environment(&self) -> &Arc<OverlayedDynamicEnvironment<WaypointSE2>> {
        &self.environment
    }

    pub fn safe_intervals_for(
        &self,
        key: &G::Key,
    ) -> Result<SafeArrivalTimes, SafeIntervalCacheError<G::Key>>
    where
        G::Key: Key + Clone,
        G::Vertex: Positioned,
    {
        let earliest_time = match self.earliest_time {
            Some(earliest_time) => earliest_time,
            None => return Ok(SafeArrivalTimes::new()),
        };

        match self.safe_intervals.read() {
            Ok(guard) => {
                if let Some(times) = guard.get(key) {
                    // We have already calculated the safe interval for this
                    // key, so just give back the view.
                    return Ok(times.clone());
                }
            }
            Err(_) => return Err(SafeIntervalCacheError::PoisonedMutex),
        }

        // Calculate the safe intervals for this key.
        let p = self
            .graph
            .vertex(key)
            .ok_or_else(|| SafeIntervalCacheError::MissingVertex(key.clone()))?
            .borrow()
            .point();

        let wp = WaypointR2::new(earliest_time, p.x, p.y);
        let safe_arrivals =
            compute_safe_arrival_times(wp, &self.environment);
        // let safe_arrivals =
        //     compute_safe_arrival_times(WaypointR2::new(earliest_time, p.x, p.y), &self.environment);
        match self.safe_intervals.write() {
            Ok(mut guard) => {
                guard.insert(key.clone(), safe_arrivals.clone());
            }
            Err(_) => return Err(SafeIntervalCacheError::PoisonedMutex),
        }
        Ok(safe_arrivals)
    }
}

#[derive(Debug, ThisError)]
pub enum SafeIntervalCacheError<K> {
    #[error("The mutex has been poisoned")]
    PoisonedMutex,
    #[error("The vertex {0:?} does not exist in the graph")]
    MissingVertex(K),
}

pub struct SafeIntervalMotion<G: Graph, const R: u32> {
    pub extrapolator: DifferentialDriveLineFollow,
    pub safe_intervals: Arc<SafeIntervalCache<G>>,
}

impl<G: Graph, const R: u32> Clone for SafeIntervalMotion<G, R> {
    fn clone(&self) -> Self {
        Self {
            extrapolator: self.extrapolator,
            safe_intervals: self.safe_intervals.clone(),
        }
    }
}

impl<G: Graph, const R: u32> SafeIntervalMotion<G, R> {
    fn compute_safe_arrivals(
        &self,
        from_state: &StateSE2<G::Key, R>,
        target_key: G::Key,
        to_target: &G::Vertex,
        with_guidance: &G::EdgeAttributes,
    ) -> impl Iterator<Item = Result<(SafePathSE2, WaypointSE2), SafeIntervalMotionError<G::Key>>>
    where
        G::Key: Key + Clone,
        G::Vertex: Positioned + MaybeOriented,
        G::EdgeAttributes: SpeedLimiter,
    {
        let mut safe_intervals = match self.safe_intervals.safe_intervals_for(&target_key) {
            Ok(r) => r,
            Err(err) => {
                return ForkIter::Left(Some(Err(SafeIntervalMotionError::Cache(err))).into_iter())
            }
        };

        let target_point = to_target.point();
        let mut arrival = match self.extrapolator.move_towards_target(
            &from_state.waypoint,
            &target_point,
            with_guidance,
        ) {
            Ok(arrival) => arrival,
            Err(err) => {
                return ForkIter::Left(
                    Some(Err(SafeIntervalMotionError::Extrapolator(err))).into_iter(),
                )
            }
        };

        if arrival.waypoints.len() > 1 {
            assert!(arrival.waypoints.len() < 3);
            let wp0 = arrival.waypoints[0].clone().into();
            // Make sure the act of rotating to face the target is valid
            if !is_safe_segment(
                (&from_state.waypoint.into(), &wp0),
                None,
                &self.safe_intervals.environment,
            ) {
                // We cannot rotate to face the target, so there is no way to
                // avoid conflicts from the start state.
                return ForkIter::Left(None.into_iter());
            }
        }

        let to_position = match arrival.waypoints.last() {
            Some(p) => *p,
            // No motion is needed, the agent is already on the target
            None => {
                return ForkIter::Left(Some(Ok((SmallVec::new(), from_state.waypoint))).into_iter())
            }
        };

        let maybe_oriented = to_target.maybe_oriented();
        let from_point: WaypointR2 = arrival.facing_target.into();
        let to_point: WaypointR2 = to_position.into();
        let yaw = arrival.yaw.angle();
        let ranked_hints = compute_safe_linear_path_wait_hints(
            (&from_point, &to_point),
            None,
            &self.safe_intervals.environment,
        );

        safe_intervals.retain(|t| *t >= to_position.time);
        // Add the time when the agent would normally arrive at the vertex.
        safe_intervals.insert(0, to_position.time);

        let paths: SmallVec<[_; 5]> = safe_intervals
            .into_iter()
            .filter_map(|arrival_time| {
                compute_safe_arrival_path(
                    from_point,
                    to_point,
                    arrival_time,
                    &ranked_hints,
                    &self.safe_intervals.environment,
                )
            })
            .filter_map(move |action| {
                let mut action: SmallVec<[SafeAction<WaypointSE2, WaitForObstacle>; 5]> = action
                    .into_iter()
                    .map(|a| a.map_movement(|wp| wp.with_yaw(yaw)))
                    .collect();

                if arrival.waypoints.len() > 1 {
                    // Add the initial rotation to the safe actions
                    action.insert(0, SafeAction::Move(arrival.waypoints[0]));
                }

                // TODO(@mxgrey): Remove these unwraps before targeting production.
                let arrival_wp = *action.last().unwrap().movement().unwrap();
                if let Some(target_yaw) = maybe_oriented {
                    // TODO(@mxgrey): Consider how to de-duplicate this block
                    // from the Extrapolator impl.
                    let delta_yaw_abs = (target_yaw / arrival.yaw).angle().abs();
                    if delta_yaw_abs > self.extrapolator.rotational_threshold() {
                        arrival.time += Duration::from_secs_f64(
                            self.extrapolator.direction() * delta_yaw_abs
                                / self.extrapolator.rotational_speed(),
                        );
                        let final_wp = WaypointSE2 {
                            time: arrival.time,
                            position: Position::new(target_point.coords, target_yaw.angle()),
                        };

                        if !is_safe_segment(
                            (&arrival_wp.into(), &final_wp.into()),
                            None,
                            &self.safe_intervals.environment,
                        ) {
                            // We cannot rotate to face the target orientation
                            // so this is not a valid action.
                            return None;
                        }
                        action.push(SafeAction::Move(final_wp));
                    }
                }

                let wp = *action.last().unwrap().movement().unwrap();
                Some(Ok((action, wp)))
            })
            .collect();

        ForkIter::Right(paths.into_iter())
    }
}

impl<G: Graph, const R: u32> Domain for SafeIntervalMotion<G, R> {
    type State = StateSE2<G::Key, R>;
    type Error = SafeIntervalMotionError<G::Key>;
}

pub type SafePathSE2 = SmallVec<[SafeAction<WaypointSE2, WaitForObstacle>; 5]>;

impl<G: Graph, const R: u32> Activity<StateSE2<G::Key, R>> for SafeIntervalMotion<G, R>
where
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: Clone + SpeedLimiter,
{
    type ActivityAction = SafePathSE2;
    type ActivityError = SafeIntervalMotionError<G::Key>;
    type Choices<'a> = impl Iterator<Item=Result<(Self::ActivityAction, StateSE2<G::Key, R>), Self::ActivityError>> + 'a
    where
        G: 'a;

    fn choices<'a>(&'a self, from_state: StateSE2<G::Key, R>) -> Self::Choices<'a>
    where
        G: 'a,
        G::EdgeAttributes: 'a,
    {
        // TODO(@mxgrey): We could simplify this implementation considerably if
        // we tweak the definition of Extrapolator to also take in the vertex key.
        // Then we could implement SafeIntervalMotion as an Extrapolator instead
        // of an Activity. Then possibly take advantage of the ConflictAvoidance
        // template to implement an Extrapolator. This is all being hard-coded
        // for now as a proof of concept.
        let vertex = from_state.key.vertex.clone();
        self.safe_intervals
            .graph
            .edges_from_vertex(&vertex)
            .into_iter()
            .flat_map(move |edge| {
                let from_state = from_state.clone();
                let to_vertex = edge.to_vertex().clone();
                let edge = edge.attributes().clone();
                self.safe_intervals
                    .graph
                    .vertex(&to_vertex)
                    .ok_or_else(|| SafeIntervalMotionError::MissingVertex(to_vertex.clone()))
                    .flat_result_map(move |v| {
                        let from_state = from_state.clone();
                        let to_vertex = to_vertex.clone();
                        self.compute_safe_arrivals(
                            &from_state,
                            to_vertex.clone(),
                            v.borrow(),
                            &edge,
                        )
                        .map(move |r| {
                            let to_vertex = to_vertex.clone();
                            r.map(move |(action, wp)| {
                                let state = StateSE2::new(to_vertex.clone(), wp);
                                (action, state)
                            })
                        })
                    })
                    .map(|x| x.flatten())
            })
    }
}

impl<G: Graph, const R: u32> Keyed for SafeIntervalMotion<G, R>
where
    G::Key: Key,
{
    type Key = KeySE2<G::Key, R>;
}

impl<G: Graph, const R: u32> Keyring<StateSE2<G::Key, R>> for SafeIntervalMotion<G, R>
where
    G::Key: Key,
{
    type KeyRef<'a> = &'a KeySE2<G::Key, R>
    where
        Self: 'a,
        StateSE2<G::Key, R>: 'a;

    fn key_for<'a>(&'a self, state: &'a StateSE2<G::Key, R>) -> Self::KeyRef<'a>
    where
        Self: 'a,
        StateSE2<G::Key, R>: 'a,
    {
        &state.key
    }
}

#[derive(Debug, ThisError)]
pub enum SafeIntervalMotionError<K> {
    #[error("The safe interval cache experienced an error:\n{0:?}")]
    Cache(SafeIntervalCacheError<K>),
    #[error("The vertex {0:?} does not exist in the graph")]
    MissingVertex(K),
    // Give something
    #[error("An error occurred in the extrapolator:\n{0:?}")]
    Extrapolator(DifferentialDriveLineFollowError),
}

#[derive(Clone)]
pub struct SafeIntervalCloser<Ring, G: Graph> {
    keyring: Ring,
    cache: Arc<SafeIntervalCache<G>>,
}

impl<Ring, G: Graph> SafeIntervalCloser<Ring, G> {
    pub fn new(keyring: Ring, safe_intervals: Arc<SafeIntervalCache<G>>) -> Self {
        Self {
            keyring,
            cache: safe_intervals,
        }
    }
}

impl<Ring, G, State> Closable<State> for SafeIntervalCloser<Ring, G>
where
    Ring: Keyring<State> + Clone,
    Ring::Key: Borrow<G::Key> + Clone,
    G: Graph,
    G::Vertex: Positioned,
    G::Key: Key + Clone,
    State: Timed,
{
    type ClosedSet<T> = SafeIntervalClosedSet<Ring, G, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        SafeIntervalClosedSet::new(self.keyring.clone(), self.cache.clone())
    }
}

pub struct SafeIntervalClosedSet<Ring: Keyed, G: Graph, T> {
    keyring: Ring,
    cache: Arc<SafeIntervalCache<G>>,
    container: HashMap<Ring::Key, ClosedIntervals<T>>,
}

impl<Ring: Keyed, G: Graph, T> SafeIntervalClosedSet<Ring, G, T> {
    pub fn new(keyring: Ring, cache: Arc<SafeIntervalCache<G>>) -> Self {
        Self {
            keyring,
            cache,
            container: HashMap::new(),
        }
    }

    fn get_closed_intervals<'a, State: Timed>(
        &'a mut self,
        state: &State,
    ) -> Option<&'a mut ClosedIntervals<T>>
    where
        Ring: Keyring<State>,
        Ring::Key: Borrow<G::Key> + Clone,
        G::Vertex: Positioned,
        G::Key: Key + Clone,
    {
        let key = self.keyring.key_for(state);
        match self.container.entry(key.borrow().clone()) {
            Entry::Occupied(entry) => Some(entry.into_mut()),
            Entry::Vacant(entry) => {
                // This key has never been closed, so we need to fetch the
                // safe interval timings first.
                let safe_intervals = match self.cache.safe_intervals_for(key.borrow().borrow()) {
                    Ok(r) => r,
                    Err(_) => {
                        // TODO(@mxgrey): This trait doesn't allow an error.
                        // Should we change the trait's return to value for
                        // errors?
                        return None;
                    }
                };

                Some(entry.insert(ClosedIntervals::new(safe_intervals)))
            }
        }
    }
}

impl<State, Ring, G, T> ClosedSet<State, T> for SafeIntervalClosedSet<Ring, G, T>
where
    Ring: Keyring<State>,
    Ring::Key: Borrow<G::Key> + Clone,
    G: Graph,
    G::Vertex: Positioned,
    G::Key: Key + Clone,
    State: Timed,
{
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        match self.get_closed_intervals(state) {
            Some(closed_intervals) => closed_intervals.close(state.time(), value),
            None => CloseResult::Accepted,
        }
    }

    fn replace(&mut self, state: &State, value: T) -> Option<T> {
        match self.get_closed_intervals(state) {
            Some(closed_intervals) => closed_intervals.replace(state.time(), value),
            None => Some(value),
        }
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key = self.keyring.key_for(state);
        let time = state.time();
        match self.container.get(key.borrow().borrow()) {
            Some(closed_intervals) => closed_intervals.status(time),
            None => ClosedStatus::Open,
        }
    }

    type ClosedSetIter<'a> = impl Iterator<Item=&'a T> + 'a
    where
        Self: 'a,
        State: 'a,
        T: 'a;

    fn iter_closed<'a>(&'a self) -> Self::ClosedSetIter<'a>
    where
        Self: 'a,
        State: 'a,
        T: 'a,
    {
        self.container.values().flat_map(|c| c.iter())
    }
}

struct ClosedIntervals<T> {
    /// A state that was closed before the first safe interval, or if the entire
    /// timeline is a safe interval
    indefinite_start: Option<T>,
    intervals: SmallVec<[(TimePoint, Option<T>); 5]>,
}

impl<T> ClosedIntervals<T> {
    fn new(safe_intervals: SafeArrivalTimes) -> Self {
        let mut intervals: SmallVec<[_; 5]> =
            safe_intervals.into_iter().map(|t| (t, None)).collect();

        intervals.sort_unstable_by(|a, b| a.0.cmp(&b.0));

        Self {
            indefinite_start: None,
            intervals,
        }
    }

    fn get_index(&self, time: TimePoint) -> Option<usize> {
        match self.intervals.binary_search_by_key(&time, |x| x.0) {
            Ok(index) => {
                // A perfect match was found for the interval, so just return
                // the index for it.
                Some(index)
            }
            Err(ceiling) => {
                // A perfect match was not found, so we're instead given the
                // index where the time could be inserted while maintaining
                // the sorted order. That means `ceiling` represents the
                // `ceiling` of the interval that that we're looking for.
                // However, our container is indexing the intervals based on
                // the floor of the interval, so we need to go down by one.
                if ceiling == 0 {
                    // We are actually in the indefinite start region of the
                    // timeline.
                    None
                } else {
                    Some(ceiling - 1)
                }
            }
        }
    }

    fn close<'a>(&'a mut self, time: TimePoint, value: T) -> CloseResult<'a, T> {
        let index = match self.get_index(time) {
            Some(index) => index,
            None => {
                // We do some weird replacing gymnastics here because the borrow
                // checker complains if I try to implement this in the more
                // sensible way with pattern matching.
                let prior = self.indefinite_start.replace(value);
                let value = if let Some(prior) = prior {
                    self.indefinite_start.replace(prior)
                } else {
                    None
                };

                if let (Some(prior), Some(value)) = (&mut self.indefinite_start, value) {
                    return CloseResult::Rejected { value, prior };
                } else {
                    return CloseResult::Accepted;
                }
            }
        };

        let (_, entry) = self.intervals.get_mut(index).unwrap();
        if let Some(prior) = entry {
            return CloseResult::Rejected { value, prior };
        }

        *entry = Some(value);
        CloseResult::Accepted
    }

    fn replace(&mut self, time: TimePoint, value: T) -> Option<T> {
        match self.get_index(time) {
            Some(index) => self.intervals.get_mut(index).unwrap().1.replace(value),
            None => self.indefinite_start.replace(value),
        }
    }

    fn status<'a>(&'a self, time: TimePoint) -> ClosedStatus<'a, T> {
        let prior = match self.get_index(time) {
            Some(index) => self.intervals.get(index).unwrap().1.as_ref(),
            None => self.indefinite_start.as_ref(),
        };

        prior.into()
    }

    fn iter<'a>(&'a self) -> impl Iterator<Item = &'a T> + 'a {
        self.indefinite_start.iter().chain(
            self.intervals
                .iter()
                .filter_map(|(_, value)| value.as_ref()),
        )
    }
}
