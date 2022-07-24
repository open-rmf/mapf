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
    node::{PartialKeyed, Keyed, Weighted, KeyedSet},
    expander::{InitAimless, Closable, NodeOf, SolutionOf},
    tree::{garden, Garden},
    motion::{
        se2, r2, trajectory::CostCalculator, reach::NoReach,
        graph_search::StateKey,
    },
    heuristic::{Heuristic, Uninformed},
    graph::Graph,
};
use std::sync::Arc;

pub struct QuickestPath<G, C>
where
    G: Graph<Vertex=r2::Position>,
    C: CostCalculator<r2::timed_position::Waypoint>,
{
    garden: Garden<r2::graph_search::TimeInvariantExpander<G, C, Uninformed>>,
}

impl<G, C> QuickestPath<G, C>
where
    G: Graph<Vertex=r2::Position>,
    C: CostCalculator<r2::timed_position::Waypoint>,
{
    pub fn new(
        graph: Arc<G>,
        cost_calculator: Arc<C>,
        extrapolator: Arc<r2::timed_position::LineFollow>,
    ) -> Self {
        Self{
            garden: Garden::new(
                Arc::new(r2::graph_search::TimeInvariantExpander{
                    graph,
                    extrapolator,
                    cost_calculator,
                    heuristic: Arc::new(Uninformed),
                    reacher: Arc::new(NoReach),
                })
            )
        }
    }
}

type UninformedExpanderR2<G, C> = r2::graph_search::TimeInvariantExpander<G, C, Uninformed>;

impl<G, C, S, Goal> Heuristic<S, Goal, C::Cost> for QuickestPath<G, C>
where
    G: Graph<Vertex=r2::Position>,
    C: CostCalculator<r2::timed_position::Waypoint>,
    S: StateKey<G::Key, se2::timed_position::Waypoint>,
    Goal: Keyed<Key=G::Key>,
    UninformedExpanderR2<G, C>: InitAimless<G::Key> + Closable<ClosedSet: KeyedSet<NodeOf<UninformedExpanderR2<G, C>>, Key=G::Key>>,
    NodeOf<UninformedExpanderR2<G, C>>: PartialKeyed<Key=G::Key> + Weighted,
    SolutionOf<UninformedExpanderR2<G, C>>: Clone + Weighted,
{
    type Error = garden::Error<r2::graph_search::TimeInvariantExpander<G, C, Uninformed>, G::Key>;

    fn estimate_cost(
        &self,
        from_state: &S,
        to_goal: &Goal,
    ) -> Result<Option<C::Cost>, Self::Error> {
        let start_key: G::Key = from_state.graph_key();
        self.garden.solve(&start_key, to_goal.key())
        .map(|solution_opt| solution_opt.map(|solution| solution.cost().clone()))
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn build() {

    }
}
