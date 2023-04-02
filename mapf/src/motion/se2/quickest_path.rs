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
            DiscreteSpaceTimeSE2, StateSE2, PreferentialStarburstSE2,
            MaybeOriented, DifferentialDriveLineFollow, WaypointSE2, MergeIntoGoal,
        },
        r2::Positioned,
    },
    domain::{Informed, Weighted, KeyedCloser, Reversible, Key},
    algorithm::BackwardDijkstra,
    templates::{
        GraphMotion, LazyGraphMotion, UninformedSearch,
        informed_search::InformedSearchReversalError,
    },
    error::{Anyhow, ThisError},
};
use arrayvec::ArrayVec;
use std::{
    borrow::Borrow,
    ops::Add,
    sync::Arc,
};

pub type QuickestPathSearch<G, W, const R: u32> =
    UninformedSearch<
        GraphMotion<
            DiscreteSpaceTimeSE2<<G as Graph>::Key, R>,
            G,
            DifferentialDriveLineFollow,
        >,
        W,
        KeyedCloser<DiscreteSpaceTimeSE2<<G as Graph>::Key, R>>,
        PreferentialStarburstSE2<G, R>,
        PreferentialStarburstSE2<G, R>,
        LazyGraphMotion<
            DiscreteSpaceTimeSE2<<G as Graph>::Key, R>,
            G,
            DifferentialDriveLineFollow,
            (),
            MergeIntoGoal<R>,
        >,
    >;

pub type QuickestPathPlanner<G, W, const R: u32> = Planner<Arc<BackwardDijkstra<QuickestPathSearch<G, W, R>>>>;

#[derive(Clone)]
pub struct QuickestPathHeuristic<G, W, const R: u32>
where
    W: Reversible,
    W: Weighted<StateSE2<G::Key, R>, ArrayVec<WaypointSE2, 3>>,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    planner: QuickestPathPlanner<G, W, R>,
}

impl<G, W, const R: u32> QuickestPathHeuristic<G, W, R>
where
    W: Reversible,
    W: Weighted<StateSE2<G::Key, R>, ArrayVec<WaypointSE2, 3>>,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    pub fn new(
        graph: G,
        weight: W,
        extrapolator: DifferentialDriveLineFollow,
    ) -> Result<Self, <QuickestPathSearch<G, W, R> as Reversible>::ReversalError> {
        let motion = GraphMotion {
            space: DiscreteSpaceTimeSE2::new(),
            graph: graph.clone(),
            extrapolator,
        };

        Ok(Self {
            planner: Planner::new(
                Arc::new(
                    BackwardDijkstra::new(
                        &UninformedSearch::new_uninformed(
                            motion.clone(),
                            weight,
                            KeyedCloser(DiscreteSpaceTimeSE2::new()),
                        )
                        .with_initializer(
                            PreferentialStarburstSE2::for_start(graph.clone())
                        )
                        .with_satisfier(
                            PreferentialStarburstSE2::for_goal(graph.clone())
                            .map_err(InformedSearchReversalError::Satisfier)?
                        )
                        .with_connector(LazyGraphMotion {
                            motion,
                            keyring: (),
                            chain: MergeIntoGoal(extrapolator),
                        })
                    )?
                )
            )
        })
    }

    pub fn planner(&self) -> &QuickestPathPlanner<G, W, R> {
        &self.planner
    }
}

impl<G, W, const R: u32, State, Goal> Informed<State, Goal> for QuickestPathHeuristic<G, W, R>
where
    W: Reversible,
    W: Weighted<StateSE2<G::Key, R>, ArrayVec<WaypointSE2, 3>>,
    W::Cost: Clone + Ord + Add<W::Cost, Output=W::Cost>,
    W::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
    State: Borrow<StateSE2<G::Key, R>>,
    Goal: Borrow<G::Key>,
{
    type CostEstimate = W::Cost;
    type InformedError = QuickestPathHeuristicError;
    fn estimate_remaining_cost(
        &self,
        from_state: &State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let start: &StateSE2<G::Key, R> = from_state.borrow();
        let goal: &G::Key = to_goal.borrow();

        self.planner
            .plan(start.key.clone(), goal.clone())
            .map_err(|_| QuickestPathHeuristicError::PlannerError)?
            .solve().map_err(|_| QuickestPathHeuristicError::PlannerError)
            .map(|status| status.solution().map(|s| {
                s.total_cost
            }))
    }
}

// TODO(@mxgrey): Put actual error information inside of here.
#[derive(Debug, ThisError)]
pub enum QuickestPathHeuristicError {
    #[error("An error occurred in the planner")]
    PlannerError,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::{
            SimpleGraph, SharedGraph,
            occupancy::{Cell, Visibility, VisibilityGraph, SparseGrid},
        },
        motion::{
            TravelTimeCost, TimePoint, CircularProfile,
            se2::{Point, GoalSE2},
        },
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

        let start = StateSE2::new(0usize, WaypointSE2::new(TimePoint::zero(), 0.0, 0.0, 0.0));
        let goal = 8usize;

        let estimate = heuristic.estimate_remaining_cost(&start, &goal).unwrap().unwrap();

        let expected_estimate = 5.0 + 2_f64.sqrt() + 135_f64.to_radians() * 2.0 + 90_f64.to_radians() * 2.0;
        assert_relative_eq!(estimate.0, expected_estimate, max_relative = 0.0001);
    }

    #[test]
    fn test_open_freespace_quickest_path() {
        let cell_size = 1.0;
        let profile = CircularProfile::new(0.75, 0.0, 0.0).unwrap();
        let visibility = Arc::new(Visibility::new(
            SparseGrid::new(cell_size),
            profile.footprint_radius(),
        ));

        let drive = DifferentialDriveLineFollow::new(3.0, 1.0).unwrap();
        let heuristic: QuickestPathHeuristic<_, _, 100> = QuickestPathHeuristic::new(
            SharedGraph::new(VisibilityGraph::new(visibility, [])),
            TravelTimeCost(1.0),
            drive,
        ).unwrap();

        let state_cell = Cell::new(0, 0);
        let state_p = state_cell.to_center_point(cell_size);
        let goal_cell = Cell::new(10, 0);
        let goal_p = goal_cell.to_center_point(cell_size);
        let from_state = StateSE2::new(
            state_cell,
            WaypointSE2::new_f64(0.0, state_p.x, state_p.y, 0.0),
        );

        let goal = GoalSE2 {
            key: goal_cell,
            orientation: None,
        };

        let remaining_cost_estimate = heuristic.estimate_remaining_cost(
            &from_state, &goal
        ).unwrap().unwrap();
        assert_relative_eq!(
            (goal_p - state_p).norm()/drive.translational_speed(),
            remaining_cost_estimate.0,
            max_relative = 0.01,
        );
    }

    #[test]
    fn test_obstructed_freespace_quickest_path() {
        let cell_size = 1.0;
        let profile = CircularProfile::new(0.75, 0.0, 0.0).unwrap();
        let visibility = Arc::new({
            let mut vis = Visibility::new(
                SparseGrid::new(cell_size),
                profile.footprint_radius(),
            );
            vis.change_cells(
                &(-10..=-1_i64)
                .into_iter()
                .map(|y| (Cell::new(3, y), true))
                .collect()
            );
            vis
        });

        let drive = DifferentialDriveLineFollow::new(3.0, 1.0).unwrap();
        let heuristic: QuickestPathHeuristic<_, _, 360> = QuickestPathHeuristic::new(
            SharedGraph::new(VisibilityGraph::new(visibility, [])),
            TravelTimeCost(1.0),
            drive,
        ).unwrap();

        let from_state = StateSE2::new(
            Cell::new(-6, -3),
            WaypointSE2::new_f64(1.256802684, -5.5, -2.5, 45_f64.to_radians()),
        );

        let goal = GoalSE2 {
            key: Cell::new(18, 3),
            orientation: None,
        };

        let remaining_cost_estimate = heuristic.estimate_remaining_cost(
            &from_state, &goal
        ).unwrap().unwrap();
        dbg!(remaining_cost_estimate);
    }
}
