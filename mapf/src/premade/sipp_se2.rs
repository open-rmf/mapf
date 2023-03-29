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
    templates::{InformedSearch, ConflictAvoidance, GraphMotion},
    motion::{
        TravelTimeCost, SpeedLimiter, OverlayedDynamicEnvironment,
        SharedEnvironment,
        se2::{*, quickest_path::QuickestPathSearch},
        r2::Positioned,
    },
    graph::{Graph, SharedGraph},
    domain::{Key, TimeVariantKeyedCloser, Reversible},
};

const DEFAULT_RES: u32 = 360;

/// A specialization of InformedSearch for performing Safe Interval Path
/// Planning (SIPP) through the SE(2) space (Special Euclidean Group, dimension
/// 2), e.g. the x-y plane with rigid body rotations.
///
/// WARNING: To have a final orientation in the goal conditions, this domain
/// needs to be given to a [`crate::algorithm::AStarConnect`] instead of
/// [`crate::algorithm::AStar`], otherwise the planner will fail to reach the
/// goal.
pub type SippSE2<G> = InformedSearch<
    GraphMotion<
        DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>,
        SharedGraph<G>,
        ConflictAvoidance<
            DifferentialDriveLineFollow,
            SharedEnvironment<OverlayedDynamicEnvironment<WaypointSE2>>
        >
    >,
    TravelTimeCost,
    QuickestPathHeuristic<SharedGraph<G>, TravelTimeCost, DEFAULT_RES>,
    TimeVariantKeyedCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>>,
    InitializeSE2<SharedGraph<G>>,
    SatisfySE2,
    SafeMergeIntoGoal<DEFAULT_RES>,
>;

impl<G> SippSE2<G>
where
    G: Graph + Reversible,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    pub fn new_sipp_se2(
        graph: SharedGraph<G>,
        motion: DifferentialDriveLineFollow,
        environment: SharedEnvironment<OverlayedDynamicEnvironment<WaypointSE2>>,
    ) -> Result<Self, <QuickestPathSearch<SharedGraph<G>, TravelTimeCost, DEFAULT_RES> as Reversible>::ReversalError> {
        Ok(
            InformedSearch::new(
                GraphMotion {
                    space: DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
                    graph: graph.clone(),
                    extrapolator: ConflictAvoidance {
                        avoider: motion,
                        environment: environment.clone(),
                    },
                },
                TravelTimeCost(1.0),
                QuickestPathHeuristic::new(
                        graph.clone(),
                        TravelTimeCost(1.0),
                        motion,
                    )?,
                TimeVariantKeyedCloser::new(DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new()),
            )
            .with_initializer(InitializeSE2(graph))
            .with_satisfier(SatisfySE2::from(motion))
            .with_connector(SafeMergeIntoGoal::new(motion, environment))
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        algorithm::AStarConnect, Planner,
        graph::SimpleGraph,
        motion::{
            DynamicEnvironment, DynamicCircularObstacle, Trajectory,
            CircularProfile
        },
    };
    use std::sync::Arc;
    use approx::assert_relative_eq;

    #[test]
    fn test_simple_sipp_se2() {
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

        let profile = CircularProfile::new(0.1, 0.25, 1.0).unwrap();
        let environment = SharedEnvironment::new(
            OverlayedDynamicEnvironment::new(
                Arc::new({
                    let mut env = DynamicEnvironment::new(profile);
                    env.obstacles.push(
                        DynamicCircularObstacle::new(profile)
                        .with_trajectory(Some(Trajectory::from_iter([
                            WaypointSE2::new_f64(2.5, 2.0, -1.0, 0.0),
                            WaypointSE2::new_f64(6.0, 1.0, -1.0, 0.0),
                            WaypointSE2::new_f64(10.0, 1.0, -1.0, 0.0),
                        ]).unwrap()))
                    );
                    env
                })
            )
        );

        let domain = InformedSearch::new_sipp_se2(
            SharedGraph::new(graph),
            DifferentialDriveLineFollow::new(2.0, 1.0).unwrap(),
            environment,
        ).unwrap();

        let planner = Planner::new(AStarConnect(domain));
        let solution = planner
            .plan((0usize, 20_f64.to_radians()), 8usize).unwrap()
            .solve().unwrap()
            .solution().unwrap();

        // Note: Vertex 4 is blocked until 10s.
        let expected_cost = 10.2 + (135_f64 + 2.0*90.0).to_radians() + 3.0/2.0;
        assert_relative_eq!(solution.total_cost.0, expected_cost, max_relative=0.1);
    }
}
