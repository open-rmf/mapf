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
    domain::{Closable, CloseResult, ClosedSet, ClosedStatus, Key, Keyed, Keyring},
    error::ThisError,
    graph::Graph,
    motion::{
        compute_safe_arrival_times,
        r2::{Positioned, WaypointR2},
        se2::WaypointSE2,
        CcbsEnvironment, SafeArrivalTimes, TimePoint, Timed,
    },
    util::Minimum,
};
use smallvec::SmallVec;
use std::{
    borrow::Borrow,
    collections::{hash_map::Entry, HashMap},
    sync::{Arc, RwLock},
};

pub struct SafeIntervalCache<G: Graph> {
    graph: G,
    environment: Arc<CcbsEnvironment<WaypointSE2, G::Key>>,
    earliest_time: Option<TimePoint>,
    safe_intervals: RwLock<HashMap<G::Key, SafeArrivalTimes>>,
}

impl<G: Graph> SafeIntervalCache<G> {
    pub fn new(environment: Arc<CcbsEnvironment<WaypointSE2, G::Key>>, graph: G) -> Self
    where
        G::Key: Key,
    {
        let mut earliest_time = Minimum::new(|a: &TimePoint, b: &TimePoint| a.cmp(b));
        for obs in environment.iter_all_obstacles() {
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

    pub fn environment(&self) -> &Arc<CcbsEnvironment<WaypointSE2, G::Key>> {
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
        let ccbs_key = (key.clone(), key.clone());
        let safe_arrivals =
            compute_safe_arrival_times(wp, &self.environment.view_for(Some(&ccbs_key)));
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

#[derive(Debug, ThisError)]
pub enum SafeIntervalMotionError<K, E> {
    #[error("The safe interval cache experienced an error:\n{0:?}")]
    Cache(SafeIntervalCacheError<K>),
    #[error("The vertex {0:?} does not exist in the graph")]
    MissingVertex(K),
    // Give something
    #[error("An error occurred in the extrapolator:\n{0:?}")]
    Extrapolator(E),
}
