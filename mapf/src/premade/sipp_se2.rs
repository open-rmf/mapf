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
        se2::{*, quickest_path::QuickestPathSearch},
        r2::Positioned,
    },
    graph::{Graph, SharedGraph},
    domain::{Key, TimeVariantKeyedCloser, Reversible, Configurable},
};
use std::sync::Arc;

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
            Arc<OverlayedDynamicEnvironment<WaypointSE2>>,
        >
    >,
    TravelTimeCost,
    QuickestPathHeuristic<SharedGraph<G>, TravelTimeCost, DEFAULT_RES>,
    TimeVariantKeyedCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>>,
    InitializeSE2<SharedGraph<G>>,
    SatisfySE2,
    SafeMergeIntoGoal<DEFAULT_RES>,
>;

pub type NewSippSE2Error<G> = <QuickestPathSearch<SharedGraph<G>, TravelTimeCost, DEFAULT_RES> as Reversible>::ReversalError;

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
        environment: Arc<OverlayedDynamicEnvironment<WaypointSE2>>,
    ) -> Result<Self, NewSippSE2Error<G>> {
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

pub struct SippSE2Configuration<G> {
    pub graph: SharedGraph<G>,
    pub motion: DifferentialDriveLineFollow,
    pub environment: Arc<OverlayedDynamicEnvironment<WaypointSE2>>,
}

impl<G> SippSE2Configuration<G> {
    pub fn modify_environment<F>(mut self, f: F) -> Self
    where
        F: FnOnce(OverlayedDynamicEnvironment<WaypointSE2>) -> OverlayedDynamicEnvironment<WaypointSE2>,
    {
        // Attempt to reclaim the data held by the Arc, as long as it's not
        // being shared anywhere else. Otherwise we have to make a clone of it.
        let env = match Arc::try_unwrap(self.environment) {
            Ok(env) => { dbg!(); env},
            Err(arc_env) => { dbg!(); (*arc_env).clone() },
        };

        self.environment = Arc::new(f(env));
        self
    }
}

impl<G> Configurable for SippSE2<G>
where
    G: Graph + Reversible,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    type Configuration = SippSE2Configuration<G>;
    type ConfigurationError = NewSippSE2Error<G>;
    fn configure<F>(self, f: F) -> Result<Self, Self::ConfigurationError>
    where
        F: FnOnce(Self::Configuration) -> Self::Configuration,
    {
        let config = {
            // Move from the self variable into a temporary that will die at the
            // end of this scope, allowing us to potentially transfer all shared
            // data ownership into the config.
            let scoped_self = self;
            SippSE2Configuration {
                graph: scoped_self.activity.graph,
                motion: scoped_self.activity.extrapolator.avoider,
                environment: scoped_self.activity.extrapolator.environment,
            }
        };

        let config = f(config);
        Self::new_sipp_se2(config.graph, config.motion, config.environment)
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

    fn make_simple_sipp_se2_domain() -> SippSE2<SimpleGraph<Point, ()>> {
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
        let environment = Arc::new(
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

        InformedSearch::new_sipp_se2(
            SharedGraph::new(graph),
            DifferentialDriveLineFollow::new(2.0, 1.0).unwrap(),
            environment,
        ).unwrap()
    }

    #[test]
    fn test_simple_sipp_se2() {
        let domain = make_simple_sipp_se2_domain();
        let planner = Planner::new(AStarConnect(domain));
        let solution = planner
            .plan((0usize, 20_f64.to_radians()), 8usize).unwrap()
            .solve().unwrap()
            .solution().unwrap();

        // Note: Vertex 4 is blocked until 10s.
        let expected_cost = 10.2 + (135_f64 + 2.0*90.0).to_radians() + 3.0/2.0;
        assert_relative_eq!(solution.total_cost.0, expected_cost, max_relative=0.1);

        let trajectory: Trajectory<WaypointSE2> = solution.make_trajectory().unwrap().unwrap();
        assert!(trajectory.len() >= 11);
    }

    #[test]
    fn test_sipp_se2_configure() {
        let domain = make_simple_sipp_se2_domain();
        let planner = Planner::new(AStarConnect(domain))
            .configure(|domain|
                domain
                .modify_environment(|mut env| {
                    env.overlay_trajectory(0, None).unwrap();
                    env
                })
            ).unwrap();

        // With the obstacle having an overlayed trajectory of None, the arrival
        // time should be as if there are no obstacles at all.
        let solution = planner
            .plan((0usize, 0.0), 8usize).unwrap()
            .solve().unwrap()
            .solution().unwrap();

        let expected_cost = (5.0 + f64::sqrt(2.0))/2.0 + (2.0 * 135_f64 + 2.0 * 90_f64).to_radians();
        assert_relative_eq!(solution.total_cost.0, expected_cost, max_relative=0.01);

        let planner = planner.configure(|mut domain| {
            domain.motion.set_translational_speed(
                2.0 * domain.motion.translational_speed()
            ).unwrap();
            domain.motion.set_rotational_speed(
                2.0 * domain.motion.rotational_speed()
            ).unwrap();
            domain
        }).unwrap();

        // Since the speeds of motion have been doubled, the arrival time should
        // be half of what it previously was.
        let solution = planner
            .plan((0usize, 0.0), 8usize).unwrap()
            .solve().unwrap()
            .solution().unwrap();

        let expected_cost = expected_cost/2.0;
        assert_relative_eq!(solution.total_cost.0, expected_cost, max_relative=0.01);

        let expected_delay = 5.0;
        let planner = planner.configure(|domain| {
            domain.modify_environment(|env| {
                env.modify_base(|base| {
                    base.obstacles.push(
                        DynamicCircularObstacle::new(
                            CircularProfile::new(0.1, 1.0, 1.0).unwrap(),
                        )
                        .with_trajectory(Some(Trajectory::from_iter([
                            WaypointSE2::new_f64(0.0, 2.0, 0.0, 0.0),
                            WaypointSE2::new_f64(0.25 + expected_delay, 1.5, 0.0, 0.0),
                        ]).unwrap()))
                    );
                })
            })
        }).unwrap();

        // Since a new obstacle was added, the arrival time should be delayed by
        // at least the delay amount.
        let solution = planner
            .plan((0usize, 0.0), 8usize).unwrap()
            .solve().unwrap()
            .solution().unwrap();
        let expected_cost = expected_cost + expected_delay;
        // TODO(@mxgrey): Make a more specific expectation.
        assert!(solution.total_cost.0 >= expected_cost);
    }
}
