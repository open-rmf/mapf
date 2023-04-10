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
    domain::{Configurable, Key, Reversible},
    graph::{Graph, SharedGraph},
    motion::{
        r2::Positioned,
        se2::{quickest_path::QuickestPathSearch, *},
        CcbsEnvironment, SpeedLimiter, TravelEffortCost,
    },
    premade::{SafeIntervalCache, SafeIntervalCloser, SafeIntervalMotion},
    templates::{ConflictAvoidance, GraphMotion, InformedSearch, LazyGraphMotion},
    error::{StdError, Anyhow},
};
use std::sync::Arc;

const DEFAULT_RES: u32 = 360;

/// A specialization of InformedSearch for performing Safe Interval Path
/// Planning (SIPP) through the SE(2) space (Special Euclidean Group, dimension
/// 2), e.g. the x-y plane with rigid body rotations.
///
/// The graph used to calculate the heuristic can be a different type from the
/// one used to guide the activity. This allows the heuristic to use faster or
/// simplified graph representations to calculate cost estimates.
///
/// WARNING: To have a final orientation in the goal conditions, this domain
/// needs to be given to a [`crate::algorithm::AStarConnect`] instead of
/// [`crate::algorithm::AStar`], otherwise the planner will fail to reach the
/// goal.
pub type SippSE2<G, H = G> = InformedSearch<
    SafeIntervalMotion<SharedGraph<G>, DEFAULT_RES>,
    TravelEffortCost,
    QuickestPathHeuristic<SharedGraph<H>, TravelEffortCost, TravelEffortCost, DEFAULT_RES>,
    SafeIntervalCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>, SharedGraph<G>>,
    InitializeSE2<SharedGraph<G>, DEFAULT_RES>,
    SatisfySE2,
    LazyGraphMotion<
        DiscreteSpaceTimeSE2<<G as Graph>::Key, DEFAULT_RES>,
        SharedGraph<G>,
        ConflictAvoidance<
            DifferentialDriveLineFollow,
            Arc<CcbsEnvironment<WaypointSE2, <G as Graph>::Key>>,
        >,
        (),
        SafeMergeIntoGoal<<G as Graph>::Key, DEFAULT_RES>,
    >,
>;

pub type NewSippSE2Error<H> =
    <QuickestPathSearch<SharedGraph<H>, TravelEffortCost> as Reversible>::ReversalError;

impl<G, H> SippSE2<G, H>
where
    G: Graph,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
    H: Graph<Key = G::Key> + Reversible,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
{
    pub fn new_sipp_se2(
        activity_graph: SharedGraph<G>,
        heuristic_graph: SharedGraph<H>,
        extrapolator: DifferentialDriveLineFollow,
        environment: Arc<CcbsEnvironment<WaypointSE2, G::Key>>,
        weight: TravelEffortCost,
    ) -> Result<Self, NewSippSE2Error<H>> {
        let cache = Arc::new(SafeIntervalCache::new(
            environment.clone(),
            activity_graph.clone(),
        ));

        let activity_motion = SafeIntervalMotion {
            extrapolator,
            safe_intervals: cache.clone(),
        };

        let connect_motion = GraphMotion {
            space: DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
            graph: activity_graph.clone(),
            extrapolator: ConflictAvoidance {
                avoider: extrapolator,
                environment: environment.clone(),
            },
        };

        Ok(InformedSearch::new(
            activity_motion,
            weight,
            QuickestPathHeuristic::new(heuristic_graph, weight, weight, extrapolator)?,
            SafeIntervalCloser::new(
                DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
                cache.clone(),
            ),
        )
        .with_initializer(InitializeSE2(activity_graph))
        .with_satisfier(SatisfySE2::from(extrapolator))
        .with_connector(LazyGraphMotion {
            motion: connect_motion,
            keyring: (),
            chain: SafeMergeIntoGoal::new(extrapolator, environment),
        }))
    }
}

pub struct SippSE2Configuration<G, H>
where
    G: Graph,
    H: Graph + Reversible,
    H::Key: Key + Clone,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
{
    safe_intervals: Arc<SafeIntervalCache<SharedGraph<G>>>,
    cache: SippSE2ManageCache<H>,
}

impl<G, H> SippSE2Configuration<G, H>
where
    G: Graph + Clone,
    G::Key: Key + Clone,
    H: Graph + Reversible,
    H::Key: Key + Clone,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
    H::ReversalError: StdError + Send + Sync,
{
    /// Modify the environment of the domain. This allows dynamic obstacles to
    /// be modified.
    ///
    /// ### Warning
    ///
    /// Using this function will invalidate the [`SafeIntervalCache`] so safe
    /// intervals will be recalculated during the next search. This is a
    /// relatively inexpensive cache to repopulate, but it's still a hit to
    /// performance, so you should not call this function needlessly.
    pub fn modify_environment<F>(mut self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(
            CcbsEnvironment<WaypointSE2, G::Key>
        ) -> Result<CcbsEnvironment<WaypointSE2, G::Key>, Anyhow>,
    {
        let (env, graph) = {
            let (environment, graph) = {
                let scoped = self.safe_intervals;
                (scoped.environment().clone(), scoped.graph().clone())
            };

            // Attempt to reclaim the data held by the Arc, as long as it's not
            // being shared anywhere else. Otherwise we have to make a clone of it.
            match Arc::try_unwrap(environment) {
                Ok(env) => (env, graph),
                Err(arc_env) => ((*arc_env).clone(), graph),
            }
        };

        match f(env) {
            Ok(env) => {
                self.safe_intervals = Arc::new(SafeIntervalCache::new(Arc::new(env), graph));
                Ok(self)
            }
            Err(err) => Err(err),
        }
    }

    /// Modify the graph used to define the activity. This allows you to close
    /// or open lanes, toggle occupancy of cells, etc.
    ///
    /// ### Warning
    ///
    /// This does not modify the graph used to calculate the heuristic. To
    /// access that graph, use [`discard_cache`]. If the activity graph and the
    /// heuristic graph are too far out of sync then you may get sub-optimal
    /// results or worse performance.
    ///
    /// Using this function will invalidate the [`SafeIntervalCache`] so safe
    /// intervals will be recalculated during the next search. This is a
    /// relatively inexpensive cache to repopulate, but it's still a hit to
    /// performance, so you should not call this function needlessly.
    pub fn modify_activity_graph<F>(mut self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(G) -> Result<G, Anyhow>,
    {
        let (env, graph) = {
            // Move the values out of self so that they are not owned elsewhere.
            let scoped = self.safe_intervals;
            (scoped.environment().clone(), scoped.graph().clone())
        };

        match graph.modify(f) {
            Ok(graph) => {
                self.safe_intervals = Arc::new(SafeIntervalCache::new(env, graph));
                Ok(self)
            }
            Err(err) => Err(err),
        }
    }

    /// Replace the graph used to define the activity. With this function, you
    /// can provide a graph that is shared with other stakeholders. In contrast,
    /// `modify_activity_graph` will allow you to change the graph `G` instance
    /// itself, either by cloning a new one or by handing over sole ownership if
    /// the graph was not being shared.
    ///
    /// ### Warning
    ///
    /// This does not modify the graph used to calculate the heuristic. To
    /// access that graph, use [`discard_cache`]. If the activity graph and the
    /// heuristic graph are too far out of sync then you may get sub-optimal
    /// results or worse performance.
    ///
    /// Using this function will invalidate the [`SafeIntervalCache`] so safe
    /// intervals will be recalculated during the next search. This is a
    /// relatively inexpensive cache to repopulate, but it's still a hit to
    /// performance, so you should not call this function needlessly.
    pub fn replace_activity_graph(mut self, graph: SharedGraph<G>) -> Result<Self, Anyhow> {
        self.safe_intervals = Arc::new(SafeIntervalCache::new(
            self.safe_intervals.environment().clone(),
            graph,
        ));
        Ok(self)
    }

    /// Get access to parts of the configuration that cannot be modified without
    /// discarding the whole heuristic cache. Recalculating the heuristic cache
    /// can incur a substantial performance cost, potentially doubling the
    /// planning time or worse.
    ///
    /// Before using this function in production software, it is recommended
    /// that you run benchmarks to test your usage strategy, unless maximum
    /// performance is not a prevailing concern for your use case.
    pub fn discard_cache<F>(mut self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(SippSE2DiscardCache<H>) -> Result<SippSE2DiscardCache<H>, Anyhow>,
        H: Reversible,
        H::ReversalError: Into<Anyhow> + 'static,
    {
        let discard = match self.cache {
            SippSE2ManageCache::Preserve(cache) => {
                let backward_domain = cache.heuristic.planner().algorithm().backward().domain();
                SippSE2DiscardCache {
                    motion: cache.motion,
                    weight: cache.weight,
                    heuristic_graph: backward_domain.activity.graph.reversed()
                        .map_err(|e| Anyhow::from(e))?,
                }
            }
            SippSE2ManageCache::Discard(discard) => discard,
        };

        match f(discard) {
            Ok(discard) => {
                self.cache = SippSE2ManageCache::Discard(discard);
                Ok(self)
            }
            Err(err) => Err(err),
        }
    }
}

pub enum SippSE2ManageCache<H>
where
    H: Graph + Reversible,
    H::Key: Key + Clone,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
{
    Preserve(SippSE2PreserveCache<H>),
    Discard(SippSE2DiscardCache<H>),
}

pub struct SippSE2PreserveCache<H>
where
    H: Graph + Reversible,
    H::Key: Key + Clone,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
{
    motion: DifferentialDriveLineFollow,
    weight: TravelEffortCost,
    heuristic: QuickestPathHeuristic<SharedGraph<H>, TravelEffortCost, TravelEffortCost, DEFAULT_RES>,
}

pub struct SippSE2DiscardCache<H> {
    pub motion: DifferentialDriveLineFollow,
    pub heuristic_graph: SharedGraph<H>,
    pub weight: TravelEffortCost,
}

impl<G, H> Configurable for SippSE2<G, H>
where
    G: Graph + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
    H: Graph<Key = G::Key> + Reversible,
    H::Vertex: Positioned + MaybeOriented,
    H::EdgeAttributes: SpeedLimiter + Clone,
    H::ReversalError: StdError + Send + Sync + 'static,
{
    type Configuration = SippSE2Configuration<G, H>;
    fn configure<F>(self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(Self::Configuration) -> Result<Self::Configuration, Anyhow>,
    {
        let config = {
            // Move from the self variable into a temporary that will die at the
            // end of this scope, allowing us to potentially transfer all shared
            // data ownership into the config.
            let scoped_self = self;
            let preserve = SippSE2PreserveCache {
                motion: scoped_self.connector.motion.extrapolator.avoider,
                weight: scoped_self.weight,
                heuristic: scoped_self.heuristic.clone(),
            };

            SippSE2Configuration {
                safe_intervals: scoped_self.activity.safe_intervals,
                cache: SippSE2ManageCache::Preserve(preserve),
            }
        };

        let config = match f(config) {
            Ok(config) => config,
            Err(err) => return Err(err),
        };

        let domain = match config.cache {
            SippSE2ManageCache::Preserve(preserve) => {
                let activity_graph = config.safe_intervals.graph().clone();
                let environment = config.safe_intervals.environment().clone();
                let connect_motion = GraphMotion {
                    space: DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
                    graph: activity_graph.clone(),
                    extrapolator: ConflictAvoidance {
                        avoider: preserve.motion,
                        environment: environment.clone(),
                    }
                };

                let closer = SafeIntervalCloser::new(
                    DiscreteSpaceTimeSE2::<G::Key, DEFAULT_RES>::new(),
                    config.safe_intervals.clone()
                );

                let activity_motion = SafeIntervalMotion {
                    extrapolator: preserve.motion,
                    safe_intervals: config.safe_intervals,
                };

                InformedSearch::new(
                    activity_motion,
                    preserve.weight,
                    preserve.heuristic,
                    closer,
                )
                .with_initializer(InitializeSE2(activity_graph.clone()))
                .with_satisfier(SatisfySE2::from(preserve.motion))
                .with_connector(LazyGraphMotion {
                    motion: connect_motion,
                    keyring: (),
                    chain: SafeMergeIntoGoal::new(preserve.motion, environment),
                })
            }
            SippSE2ManageCache::Discard(discard) => {
                Self::new_sipp_se2(
                    config.safe_intervals.graph().clone(),
                    discard.heuristic_graph,
                    discard.motion,
                    config.safe_intervals.environment().clone(),
                    discard.weight,
                ).map_err(|e| Anyhow::new(e))?
            }
        };
        Ok(domain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        algorithm::AStarConnect,
        graph::{
            occupancy::{Cell, NeighborhoodGraph, SparseGrid, Visibility, VisibilityGraph},
            SimpleGraph,
        },
        motion::{
            CircularProfile, DynamicCircularObstacle, DynamicEnvironment, TimePoint, Trajectory,
        },
        Planner,
    };
    use approx::assert_relative_eq;
    use std::sync::Arc;

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
                Point::new(0.0, 0.0),  // 0
                Point::new(1.0, 0.0),  // 1
                Point::new(2.0, 0.0),  // 2
                Point::new(3.0, 0.0),  // 3
                Point::new(1.0, -1.0), // 4
                Point::new(2.0, -1.0), // 5
                Point::new(3.0, -1.0), // 6
                Point::new(2.0, -2.0), // 7
                Point::new(3.0, -2.0), // 8
            ],
            [
                (0, 1, ()),
                (1, 0, ()),
                (1, 2, ()),
                (2, 1, ()),
                (2, 3, ()),
                (3, 2, ()),
                (2, 4, ()),
                (4, 2, ()),
                (3, 6, ()),
                (6, 3, ()),
                (4, 5, ()),
                (5, 4, ()),
                (5, 7, ()),
                (7, 5, ()),
                (7, 8, ()),
                (8, 7, ()),
            ],
        );

        let profile = CircularProfile::new(0.1, 0.25, 1.0).unwrap();
        let environment = Arc::new(CcbsEnvironment::new(Arc::new({
            let mut env = DynamicEnvironment::new(profile);
            env.obstacles.push(
                DynamicCircularObstacle::new(profile).with_trajectory(Some(
                    Trajectory::from_iter([
                        WaypointSE2::new_f64(2.5, 2.0, -1.0, 0.0),
                        WaypointSE2::new_f64(6.0, 1.0, -1.0, 0.0),
                        WaypointSE2::new_f64(10.0, 1.0, -1.0, 0.0),
                    ])
                    .unwrap(),
                )),
            );
            env
        })));

        let graph = SharedGraph::new(graph);
        InformedSearch::new_sipp_se2(
            graph.clone(),
            graph.clone(),
            DifferentialDriveLineFollow::new(2.0, 1.0).unwrap(),
            environment,
            TravelEffortCost::default(),
        )
        .unwrap()
    }

    #[test]
    fn test_simple_sipp_se2() {
        let domain = make_simple_sipp_se2_domain();
        let planner = Planner::new(AStarConnect(domain));
        let solution = planner
            .plan((0usize, 20_f64.to_radians()), 8usize)
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();

        // Note: Vertex 4 is blocked until 10s.
        let expected_arrival = 10.2 + (135_f64 + 2.0 * 90.0).to_radians() + 3.0 / 2.0;
        let arrival_time = solution
            .sequence
            .last()
            .unwrap()
            .1
            .waypoint
            .time
            .as_secs_f64();
        assert_relative_eq!(arrival_time, expected_arrival, max_relative = 0.1);

        let trajectory: Trajectory<WaypointSE2> = solution.make_trajectory().unwrap().unwrap().trajectory;
        assert!(trajectory.len() >= 11);
    }

    #[test]
    fn test_sipp_se2_configure() {
        let domain = make_simple_sipp_se2_domain();
        let planner = Planner::new(AStarConnect(domain))
            .configure(|domain| {
                domain.modify_environment(|mut env| {
                    env.overlay_trajectory(0, None).unwrap();
                    Ok(env)
                })
            })
            .unwrap();

        // With the obstacle having an overlayed trajectory of None, the arrival
        // time should be as if there are no obstacles at all.
        let solution = planner
            .plan((0usize, 0.0), 8usize)
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();

        let expected_arrival =
            (5.0 + f64::sqrt(2.0)) / 2.0 + (2.0 * 135_f64 + 2.0 * 90_f64).to_radians();
        let arrival_time = solution
            .sequence
            .last()
            .unwrap()
            .1
            .waypoint
            .time
            .as_secs_f64();
        assert_relative_eq!(arrival_time, expected_arrival, max_relative = 0.01);

        let planner = planner
            .configure(|domain| {
                Ok(domain.discard_cache(|mut params| {
                    params.motion.set_translational_speed(
                        2.0 * params.motion.translational_speed()
                    ).unwrap();

                    params.motion.set_rotational_speed(
                        2.0 * params.motion.rotational_speed()
                    ).unwrap();

                    Ok(params)
                }).unwrap())
            })
            .unwrap();

        // Since the speeds of motion have been doubled, the arrival time should
        // be half of what it previously was.
        let solution = planner
            .plan((0usize, 0.0), 8usize)
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();

        let expected_arrival = expected_arrival / 2.0;
        let arrival_time = solution
            .sequence
            .last()
            .unwrap()
            .1
            .waypoint
            .time
            .as_secs_f64();
        assert_relative_eq!(arrival_time, expected_arrival, max_relative = 0.01);

        let expected_delay = 5.0;
        let planner = planner
            .configure(|domain| {
                domain.modify_environment(|env| {
                    Ok(env.modify_base(|base| {
                        base.obstacles.push(
                            DynamicCircularObstacle::new(
                                CircularProfile::new(0.1, 1.0, 1.0).unwrap(),
                            )
                            .with_trajectory(Some(
                                Trajectory::from_iter([
                                    WaypointSE2::new_f64(0.0, 2.0, 0.0, 0.0),
                                    WaypointSE2::new_f64(0.25 + expected_delay, 1.5, 0.0, 0.0),
                                ])
                                .unwrap()
                            )),
                        );
                    }))
                })
            })
            .unwrap();

        // Since a new obstacle was added, the arrival time should be delayed by
        // at least the delay amount.
        let solution = planner
            .plan((0usize, 0.0), 8usize)
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();
        let expected_arrival = expected_arrival + expected_delay;
        let arrival_time = solution
            .sequence
            .last()
            .unwrap()
            .1
            .waypoint
            .time
            .as_secs_f64();
        assert_relative_eq!(arrival_time, expected_arrival, epsilon = 0.5);
    }

    #[test]
    fn test_sipp_se2_open_freespace() {
        let profile = CircularProfile::new(0.75, 0.0, 0.0).unwrap();
        let visibility = Arc::new(Visibility::new(
            SparseGrid::new(1.0),
            profile.footprint_radius(),
        ));

        let planner = Planner::new(AStarConnect(
            InformedSearch::new_sipp_se2(
                SharedGraph::new(NeighborhoodGraph::new(visibility.clone(), [])),
                SharedGraph::new(VisibilityGraph::new(visibility, [])),
                DifferentialDriveLineFollow::new(3.0, 1.0).unwrap(),
                Arc::new(CcbsEnvironment::new(Arc::new(
                    DynamicEnvironment::new(profile),
                ))),
                TravelEffortCost::default(),
            )
            .unwrap(),
        ));

        let solution = planner
            .plan(
                StartSE2 {
                    time: TimePoint::from_secs_f64(0.0),
                    key: Cell::new(0, 0),
                    orientation: Orientation::new(0.0),
                },
                GoalSE2::new(Cell::new(10, 1)),
            )
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();

        let trajectory = solution.make_trajectory::<WaypointSE2>().unwrap().unwrap().trajectory;
        assert_eq!(3, trajectory.len());
    }

    #[test]
    fn test_sipp_se2_obstructed_freespace() {
        let profile = CircularProfile::new(0.75, 0.0, 0.0).unwrap();
        let visibility = Arc::new({
            let mut vis = Visibility::new(SparseGrid::new(1.0), profile.footprint_radius());
            vis.change_cells(&[(Cell::new(5, 0), true)].into_iter().collect());
            vis
        });

        let planner = Planner::new(AStarConnect(
            InformedSearch::new_sipp_se2(
                SharedGraph::new(NeighborhoodGraph::new(visibility.clone(), [])),
                SharedGraph::new(VisibilityGraph::new(visibility, [])),
                DifferentialDriveLineFollow::new(3.0, 1.0).unwrap(),
                Arc::new(CcbsEnvironment::new(Arc::new(
                    DynamicEnvironment::new(profile),
                ))),
                TravelEffortCost::default(),
            )
            .unwrap(),
        ));

        let solution = planner
            .plan(
                StartSE2 {
                    time: TimePoint::from_secs_f64(0.0),
                    key: Cell::new(0, 0),
                    orientation: Orientation::new(0.0),
                },
                GoalSE2::new(Cell::new(10, 0)),
            )
            .unwrap()
            .solve()
            .unwrap()
            .solution()
            .unwrap();

        let trajectory = solution.make_trajectory::<WaypointSE2>().unwrap().unwrap().trajectory;
        assert_eq!(5, trajectory.len());
    }
}
