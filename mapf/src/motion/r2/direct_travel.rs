/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
    graph::Graph,
    motion::{
        TimePoint,
        r2::{
            Position, DiscreteSpaceTimeR2, StateR2 as StateR2,
            timed_position::{LineFollow, LineFollowError, Waypoint},
        },
    },
    domain::{Informed, Weighted, Extrapolator, Key, Space, Keyring, KeyedSpace, SelfKey},
};
use num::Zero;
use arrayvec::ArrayVec;
use std::{
    sync::Arc,
    borrow::Borrow,
};

#[derive(Debug)]
pub struct DirectTravelHeuristic<G: Graph, W> {
    pub space: DiscreteSpaceTimeR2<G::Key>,
    pub graph: Arc<G>,
    pub weight: W,
    pub extrapolator: LineFollow,
}

impl<G, W, Goal> Informed<StateR2<G::Key>, Goal> for DirectTravelHeuristic<G, W>
where
    G: Graph,
    G::Key: Key + Clone,
    G::Vertex: Borrow<Position>,
    W: Weighted<StateR2<G::Key>, ArrayVec<Waypoint, 1>>,
    Goal: SelfKey<Key=G::Key>,
{
    type CostEstimate = W::Cost;
    type InformedError = DirectTravelError<W::WeightedError>;

    fn estimate_remaining_cost(
        &self,
        from_state: &StateR2<G::Key>,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let p0 = {
            // Should this be an error instead? The current state is off the
            // graph entirely.
            if let Some(p) = self.graph.vertex(&self.space.key_for(from_state)) {
                p
            } else {
                return Ok(None);
            }
        };

        let p1 = {
            if let Some(p) = self.graph.vertex(&to_goal.key()) {
                p
            } else {
                return Ok(None);
            }
        };

        let wp0 = Waypoint {
            time: TimePoint::zero(),
            position: *p0.borrow().borrow(),
        };

        self
        .extrapolator
        .extrapolate(&from_state.waypoint, p1.borrow().borrow(), &())
        .map_err(DirectTravelError::Extrapolator)
        .map(|r|
            r
            .map(|(action, child_wp)| {
                let child_state = self.space.make_keyed_state(to_goal.key(), child_wp);
                self.weight.cost(from_state, &action, &child_state)
                .map_err(DirectTravelError::Weighted)
            })
            .transpose()
            .map(|x| x.flatten())
        )
        .flatten()
    }
}

pub enum DirectTravelError<W> {
    Weighted(W),
    Extrapolator(LineFollowError),
}
