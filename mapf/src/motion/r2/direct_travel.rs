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
        r2::{
            Position, DiscreteSpaceTimeR2, StateR2 as StateR2,
            timed_position::{LineFollow, LineFollowError, Waypoint},
        },
    },
    domain::{
        Informed, Weighted, Extrapolator, Key, Reversible, KeyedSpace, SelfKey
    },
};
use arrayvec::ArrayVec;
use thiserror::Error as ThisError;
use std::{sync::Arc, borrow::Borrow};

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
        let p_target = {
            if let Some(p) = self.graph.vertex(to_goal.key().borrow()) {
                p
            } else {
                return Ok(None);
            }
        };

        self
        .extrapolator
        .extrapolate(&from_state.waypoint, p_target.borrow().borrow(), &())
        .map_err(DirectTravelError::Extrapolator)
        .map(|r|
            r
            .map(|(action, child_wp)| {
                let child_state = self.space.make_keyed_state(
                    to_goal.key().borrow().clone(), child_wp
                );
                self.weight.cost(from_state, &action, &child_state)
                .map_err(DirectTravelError::Weighted)
            })
            .transpose()
            .map(|x| x.flatten())
        )
        .flatten()
    }
}

// NOTE(MXG): With this implementation, we assume that the reverse graph's
// vertices are in the same locations as the forward graph's vertices. We could
// consider loosening this assumption in the future.
impl<G: Graph, W: Reversible> Reversible for DirectTravelHeuristic<G, W> {
    type Reverse = DirectTravelHeuristic<G, W::Reverse>;
    type ReversalError = W::ReversalError;
    fn reversed(&self) -> Result<Self::Reverse, Self::ReversalError> {
        Ok(DirectTravelHeuristic {
            space: self.space,
            graph: self.graph.clone(),
            weight: self.weight.reversed()?,
            extrapolator: self.extrapolator.reversed().unwrap(),
        })
    }
}

#[derive(ThisError, Debug, Clone)]
pub enum DirectTravelError<W> {
    #[error("The cost calculator had an error:\n{0}")]
    Weighted(W),
    #[error("The extrapolator had an error:\n{0}")]
    Extrapolator(LineFollowError),
}
