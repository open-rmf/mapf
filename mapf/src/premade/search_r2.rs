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
    templates::{InformedSearch, GraphMotion},
    motion::{TravelTimeCost, SpeedLimiter, r2::*},
    graph::{Graph, SharedGraph},
    domain::{Key, KeyedCloser, Reversible},
};

/// A specialization of InformedSearch for searching through R2 (real^2) space,
/// e.g. the x-y plane.
pub type SearchR2<G> = InformedSearch<
    GraphMotion<DiscreteSpaceTimeR2<<G as Graph>::Key>, SharedGraph<G>, LineFollow>,
    TravelTimeCost,
    DirectTravelHeuristic<SharedGraph<G>, TravelTimeCost>,
    KeyedCloser<DiscreteSpaceTimeR2<<G as Graph>::Key>>,
    InitializeR2<SharedGraph<G>>,
    (),
    (),
>;

impl<G> SearchR2<G>
where
    G: Graph + Reversible,
    G::Key: Key + Clone,
    G::Vertex: Positioned,
    G::EdgeAttributes: SpeedLimiter,
{
    /// Create an informed search to explore a graph for an agent that moves
    /// through R2.
    pub fn new_r2(
        graph: SharedGraph<G>,
        line_follow: LineFollow,
    ) -> Self {
        InformedSearch::new(
            GraphMotion {
                space: DiscreteSpaceTimeR2::<G::Key>::new(),
                graph: graph.clone(),
                extrapolator: line_follow,
            },
            TravelTimeCost(1.0),
            DirectTravelHeuristic {
                space: DiscreteSpaceTimeR2::<G::Key>::new(),
                graph: graph.clone(),
                weight: TravelTimeCost(1.0),
                extrapolator: line_follow,
            },
            KeyedCloser(DiscreteSpaceTimeR2::<G::Key>::new()),
        )
        .with_initializer(InitializeR2(graph))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        algorithm::AStar, Planner,
        graph::SimpleGraph,
        motion::SpeedLimit,
        domain::AsTimeVariant,
    };
    use std::sync::Arc;

    fn make_test_graph() -> SimpleGraph<Position, SpeedLimit> {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
         */

        let s = SpeedLimit(None);
        SimpleGraph::from_iters(
            [
                Position::new(0.0, 0.0), // 0
                Position::new(1.0, 0.0), // 1
                Position::new(2.0, 0.0), // 2
                Position::new(3.0, 0.0), // 3
                Position::new(1.0, -1.0), // 4
                Position::new(2.0, -1.0), // 5
                Position::new(3.0, -1.0), // 6
                Position::new(2.0, -2.0), // 7
                Position::new(3.0, -2.0), // 8
            ],
            [
                (0, 1, s), (1, 0, s),
                (1, 2, s), (2, 1, s),
                (2, 3, s), (3, 2, s),
                (2, 4, s), (4, 2, s),
                (3, 6, s), (6, 3, s),
                (4, 5, s), (5, 4, s),
                (5, 7, s), (7, 5, s),
                (7, 8, s), (8, 7, s),
            ]
        )
    }

    #[test]
    fn test_simple_r2() {
        let planner = Planner::new(
            AStar(
                InformedSearch::new_r2(
                    SharedGraph::new(make_test_graph()),
                    LineFollow::new(2.0).unwrap(),
                )
            )
        );

        let solution = planner.plan(0usize, 8usize).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());
    }

    use crate::graph::occupancy::{
        Visibility, SparseGrid, Cell, NeighborhoodGraph,
    };

    #[test]
    fn test_occupancy_r2() {
        let mut visibility = Visibility::new(SparseGrid::new(0.5), 1.25);

        // Fill in a square for the range x=[0, 5], y=[0, 5]
        visibility.change_cells(
            &(0..=5).into_iter()
            .flat_map(|i|
                (0..=5).into_iter()
                .map(move |j|
                    (Cell::new(i, j), true)
                )
            )
            .collect()
        );

        let graph = NeighborhoodGraph::new(Arc::new(visibility), []);

        let planner = Planner::new(
            AStar(
                InformedSearch::new_r2(
                    SharedGraph::new(graph),
                    LineFollow::new(2.0).unwrap(),
                )
            )
        );

        let solution = planner.plan(Cell::new(-3, -3), Cell::new(10, 10)).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());
    }

    #[test]
    fn test_simple_time_variant_r2() {
        let planner = Planner::new(
            AStar(
                InformedSearch::new_r2(
                    SharedGraph::new(make_test_graph()),
                    LineFollow::new(2.0).unwrap(),
                )
                .as_time_variant()
            )
        );

        let solution = planner.plan(0usize, 8usize).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());
    }
}
