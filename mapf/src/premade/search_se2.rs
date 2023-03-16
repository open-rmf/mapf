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
    motion::{
        TravelTimeCost, SpeedLimiter,
        se2::*,
        r2::{DirectTravelHeuristic, Positioned, DiscreteSpaceTimeR2, StateR2},
    },
    graph::Graph,
    domain::{Key, KeyedCloser, Lifted, Lift, DefineTrait, StateInto},
    error::Anyhow,
};
use std::sync::Arc;

const DEFAULT_RES: u32 = 360;

/// A specialization of InformedSearch for searching through SE(2) space
/// (Special Euclidean Group, dimension 2), e.g. the x-y plane with rigid body
/// rotations.
///
/// WARNING: To have a final orientation in the goal conditions, this domain
/// needs to be given to [`crate::a_star::AStarConnect`] instead of
/// [`crate::a_star::AStar`], otherwise the planner will fail to reach the goal.
pub type SearchSE2<G> = InformedSearch<
    GraphMotion<DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>, G, DifferentialDriveLineFollow>,
    TravelTimeCost,
    Lifted<
        DefineTrait<StateSE2<<G as Graph>::Key, DEFAULT_RES>, Anyhow>,
        StateInto<StateR2<<G as Graph>::Key>>,
        DirectTravelHeuristic<G, TravelTimeCost>,
    >,
    KeyedCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>>,
    InitializeSE2<G>,
    SatisfySE2,
    MergeIntoGoal<DEFAULT_RES>,
>;

impl<G> SearchSE2<G>
where
    G: Graph,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter,
{
    pub fn new_se2(
        graph: Arc<G>,
        drive: DifferentialDriveLineFollow,
    ) -> Self {
        InformedSearch::new(
            GraphMotion {
                space: DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
                graph: graph.clone(),
                extrapolator: drive,
            },
            TravelTimeCost(1.0),
            DefineTrait::<StateSE2<G::Key, DEFAULT_RES>>::new()
            .lift(
                StateInto::<StateR2<G::Key>>::new(),
                DirectTravelHeuristic {
                    space: DiscreteSpaceTimeR2::<G::Key>::new(),
                    graph: graph.clone(),
                    weight: TravelTimeCost(1.0),
                    extrapolator: drive.into(),
                }
            ),
            KeyedCloser(DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new()),
        )
        .with_initializer(InitializeSE2(graph))
        .with_satisfier(SatisfySE2::from(drive))
        .with_connector(MergeIntoGoal(drive))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        algorithm::AStarConnect, Planner,
        graph::SimpleGraph,
        motion::SpeedLimit,
    };

    #[test]
    fn test_simple_se2() {
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
                (0, 1, s), (1, 0, s),
                (1, 2, s), (2, 1, s),
                (2, 3, s), (3, 2, s),
                (2, 4, s), (4, 2, s),
                (3, 6, s), (6, 3, s),
                (4, 5, s), (5, 4, s),
                (5, 7, s), (7, 5, s),
                (7, 8, s), (8, 7, s),
            ]
        );

        let planner = Planner::new(
            AStarConnect(
                InformedSearch::new_se2(
                    Arc::new(graph),
                    DifferentialDriveLineFollow::new(2.0, 1.0).unwrap(),
                )
            )
        );

        let solution = planner.plan((0usize, 20_f64.to_radians()), 8usize).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());

        let solution = planner.plan(
            (0usize, 20_f64.to_radians()),
            GoalSE2 {
                key: 8usize,
                orientation: Some(Orientation::from_angle(90_f64.to_radians())),
            }).unwrap().solve().unwrap();
        println!("{solution:#?}");
        assert!(solution.solved());
    }

    use crate::graph::occupancy::{
        Visibility, SparseGrid, Cell, NeighborhoodGraph
    };

    #[test]
    fn test_occupancy_se2() {
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
            AStarConnect(
                InformedSearch::new_se2(
                    Arc::new(graph),
                    DifferentialDriveLineFollow::new(2.0, 1.0).unwrap(),
                )
            )
        );

        let solution = planner.plan(
            (Cell::new(-3, -3), 20_f64.to_radians()),
            GoalSE2 {
                key: Cell::new(10, 10),
                orientation: None,
            }).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());

        let solution = planner.plan(
            (Cell::new(-3, -3), 20_f64.to_radians()),
            GoalSE2 {
                key: Cell::new(10, 10),
                orientation: Some(Orientation::from_angle(-60_f64.to_radians())),
            }).unwrap().solve().unwrap();
        // println!("{solution:#?}");
        assert!(solution.solved());
    }
}
