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
    Graph, Planner,
    motion::{
        SpeedLimiter,
        se2::{
            DiscreteSpaceTimeSE2, StarburstSE2, StateSE2,
            MaybeOriented,
            timed_position::{
                DifferentialDriveLineFollow,
                Waypoint,
            },
        },
        r2::Positioned,
    },
    domain::{Informed, Weighted, KeyedCloser, Reversible, Key},
    algorithm::BackwardDijkstra,
    templates::{
        IncrementalGraphMotion, UninformedSearch,
        incremental_graph_motion::IncrementalState,
        informed_search::InformedSearchReversalError,
    },
    error::{Anyhow, ThisError},
};
use arrayvec::ArrayVec;
use std::{
    borrow::Borrow,
    ops::Add,
};

type QuickestPathSearch<G, W, const R: u32> =
    UninformedSearch<
        IncrementalGraphMotion<
            DiscreteSpaceTimeSE2<<G as Graph>::Key, R>,
            G,
            DifferentialDriveLineFollow,
        >,
        W,
        KeyedCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, R>>,
        StarburstSE2<G, R>,
        StarburstSE2<G, R>,
        (),
    >;

type QuickestPathPlanner<G, W, const R: u32> = Planner<BackwardDijkstra<QuickestPathSearch<G, W, R>>>;

pub struct QuickestPathHeuristic<G, W, const R: u32>
where
    W: Reversible,
    W: Weighted<IncrementalState<StateSE2<G::Key, R>, G>, ArrayVec<Waypoint, 1>>,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone + 'static,
{
    planner: QuickestPathPlanner<G, W, R>,
}

impl<G, W, const R: u32> QuickestPathHeuristic<G, W, R>
where
    W: Reversible,
    W: Weighted<IncrementalState<StateSE2<G::Key, R>, G>, ArrayVec<Waypoint, 1>>,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    pub fn new(
        graph: G,
        weight: W,
        motion: DifferentialDriveLineFollow,
    ) -> Result<Self, <QuickestPathSearch<G, W, R> as Reversible>::ReversalError> {
        Ok(Self {
            planner: Planner::new(
                BackwardDijkstra::new(
                    &UninformedSearch::new_uninformed(
                        IncrementalGraphMotion {
                            space: DiscreteSpaceTimeSE2::new(),
                            graph: graph.clone(),
                            extrapolator: motion,
                        },
                        weight,
                        KeyedCloser(DiscreteSpaceTimeSE2::new()),
                    )
                    .with_initializer(StarburstSE2::for_start(graph.clone()))
                    .with_satisfier(StarburstSE2::for_goal(&graph).map_err(InformedSearchReversalError::Satisfier)?)
                )?
            )
        })
    }
}

impl<G, W, const R: u32, State, Goal> Informed<State, Goal> for QuickestPathHeuristic<G, W, R>
where
    W: Reversible,
    W: Weighted<IncrementalState<StateSE2<G::Key, R>, G>, ArrayVec<Waypoint, 1>>,
    W::Cost: Clone + Ord + Add<W::Cost, Output=W::Cost> + std::fmt::Debug,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone + std::fmt::Debug,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone + std::fmt::Debug,
    State: Borrow<StateSE2<G::Key, R>> + std::fmt::Debug,
    Goal: Borrow<StateSE2<G::Key, R>>,
{
    type CostEstimate = W::Cost;
    type InformedError = QuickestPathHeuristicError;
    fn estimate_remaining_cost(
        &self,
        from_state: &State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let start: &StateSE2<G::Key, R> = from_state.borrow();
        let goal: &StateSE2<G::Key, R> = to_goal.borrow();
        self.planner
            .plan(start.key.vertex.clone(), goal.key.vertex.clone())
            .map_err(|_| QuickestPathHeuristicError::PlannerError)?
            .solve().map_err(|_| QuickestPathHeuristicError::PlannerError)
            .map(|status| status.solution().map(|s| {
                s.total_cost
            }))
    }
}

// TODO(MXG): Put actual error information inside of here.
#[derive(Debug, ThisError)]
pub enum QuickestPathHeuristicError {
    #[error("An error occurred in the planner")]
    PlannerError,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::{SimpleGraph, SharedGraph},
        motion::{TravelTimeCost, se2::Point, TimePoint},
    };
    use approx::assert_relative_eq;

    #[test]
    fn test_quickest_path() {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
         */

        let graph = SimpleGraph::from_iters(
            [
                Point::new(0.0, 0.0), // 0
                Point::new(1.0, 0.0), // 1
                Point::new(2.0, 0.0), // 2
                Point::new(3.0, 0.0), // 3
                Point::new(1.0, -1.0), // 4
                Point::new(2.0, -1.0), // 5
                Point::new(3.0, -1.0), // 6
                Point::new(2.0, -2.0), // 7
                Point::new(3.0, -2.0), // 8
            ],
            [
                (0, 1, ()), (1, 0, ()),
                (1, 2, ()), (2, 1, ()),
                (2, 3, ()), (3, 2, ()),
                (2, 4, ()), (4, 2, ()),
                (3, 6, ()), (6, 3, ()),
                (4, 5, ()), (5, 4, ()),
                (5, 7, ()), (7, 5, ()),
                (7, 8, ()), (8, 7, ()),
            ]
        );

        let heuristic: QuickestPathHeuristic<_, _, 100> = QuickestPathHeuristic::new(
            SharedGraph::new(graph),
            TravelTimeCost(1.0),
            DifferentialDriveLineFollow::new(1.0, 1.0).unwrap(),
        ).unwrap();

        let start = StateSE2::new(0usize, Waypoint::new(TimePoint::zero(), 0.0, 0.0, 0.0));
        let goal = StateSE2::new(8usize, Waypoint::new(TimePoint::zero(), 3.0, -2.0, 0.0));

        let estimate = heuristic.estimate_remaining_cost(&start, &goal).unwrap().unwrap();

        let expected_estimate = 5.0 + 2_f64.sqrt() + 135_f64.to_radians() * 2.0 + 90_f64.to_radians() * 2.0;
        assert_relative_eq!(estimate.0, expected_estimate, max_relative = 0.0001);
    }
}
