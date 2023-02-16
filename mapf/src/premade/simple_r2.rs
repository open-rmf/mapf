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
        TravelTimeCost,
        r2::{*, timed_position::{LineFollow, SpeedLimiter}},
    },
    directed::simple::SimpleGraph,
    domain::KeyedCloser,
};
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedLimit(pub Option<f64>);
impl SpeedLimiter for SpeedLimit {
    fn speed_limit(&self) -> Option<f64> {
        self.0
    }
}

pub type SimpleR2 = InformedSearch<
    GraphMotion<
        DiscreteSpaceTimeR2<usize>,
        SimpleGraph<Position, SpeedLimit>,
        LineFollow,
    >,
    TravelTimeCost,
    DirectTravelHeuristic<
        SimpleGraph<Position, SpeedLimit>,
        TravelTimeCost,
    >,
    KeyedCloser<DiscreteSpaceTimeR2<usize>>,
    (),
    (),
    (),
>;

impl SimpleR2 {
    pub fn new_simple_r2(
        graph: Arc<SimpleGraph<Position, SpeedLimit>>,
        line_follow: LineFollow,
    ) -> Self {
        InformedSearch::new(
            GraphMotion {
                space: DiscreteSpaceTimeR2::<usize>::new(),
                graph: graph.clone(),
                extrapolator: line_follow,
            },
            TravelTimeCost(1.0),
            DirectTravelHeuristic {
                space: DiscreteSpaceTimeR2::<usize>::new(),
                graph,
                weight: TravelTimeCost(1.0),
                extrapolator: line_follow,
            },
            KeyedCloser(DiscreteSpaceTimeR2::<usize>::new()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AStar, Planner};

    #[test]
    fn test_simple_r2() {
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
        );

        let planner = Planner::new(
            AStar(
                InformedSearch::new_simple_r2(
                    Arc::new(graph),
                    LineFollow::new(2.0).unwrap(),
                )
            )
        );

        planner.plan(0usize, 8usize).unwrap();
    }
}
