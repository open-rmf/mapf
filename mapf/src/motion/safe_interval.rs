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
        Duration, CcbsEnvironment, SafeAction, SafeArrivalTimes,
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
