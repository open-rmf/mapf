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
    algorithm::{BackwardDijkstra, Path},
    domain::{Extrapolator, Connectable, Informed, Key, KeyedCloser, Reversible, Weighted},
    error::{Anyhow, ThisError},
    motion::{
        r2::{DiscreteSpaceTimeR2, InitializeR2, LineFollow, Positioned, StateR2, WaypointR2, MaybePositioned},
        se2::{
            DifferentialDriveLineFollow, DifferentialDriveLineFollowMotion,
            KeySE2, MaybeOriented, StateSE2, MergeIntoGoal,
        },
        SpeedLimiter, Timed, MaybeTimed, TimePoint,
    },
    templates::{GraphMotion, LazyGraphMotion, UninformedSearch},
    Graph, Planner,
};
use arrayvec::ArrayVec;
use std::{
    borrow::Borrow,
    collections::HashMap,
    ops::Add,
    sync::{Arc, RwLock},
};
use num::traits::Zero;

pub type QuickestPathSearch<G, W> = UninformedSearch<
    GraphMotion<DiscreteSpaceTimeR2<<G as Graph>::Key>, G, LineFollow>,
    W,
    KeyedCloser<DiscreteSpaceTimeR2<<G as Graph>::Key>>,
    InitializeR2<G>,
    (),
    LazyGraphMotion<DiscreteSpaceTimeR2<<G as Graph>::Key>, G, LineFollow, (), ()>,
>;

pub type QuickestPathPlanner<G, W> = Planner<Arc<BackwardDijkstra<QuickestPathSearch<G, W>>>>;

#[derive(Clone)]
pub struct QuickestPathHeuristic<G, WeightR2, WeightSE2, const R: u32>
where
    WeightR2: Reversible,
    WeightR2: Weighted<StateR2<G::Key>, ArrayVec<WaypointR2, 1>>,
    WeightR2::WeightedError: Into<Anyhow>,
    WeightSE2: Weighted<StateSE2<G::Key, R>, DifferentialDriveLineFollowMotion>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    planner: QuickestPathPlanner<G, WeightR2>,
    extrapolator: DifferentialDriveLineFollow,
    weight_se2: WeightSE2,
    cost_cache: Arc<RwLock<HashMap<HeuristicKey<G::Key, R>, Option<CostCache<WeightSE2::Cost, G::Key, R>>>>>,
}

#[derive(Clone, Copy)]
struct CostCache<Cost, Key, const R: u32> {
    cost: Cost,
    arrival_state: StateSE2<Key, R>,
}

type HeuristicKey<K, const R: u32> = (KeySE2<K, R>, K);

impl<G, WeightR2, WeightSE2, const R: u32> QuickestPathHeuristic<G, WeightR2, WeightSE2, R>
where
    WeightR2: Reversible,
    WeightR2: Weighted<StateR2<G::Key>, ArrayVec<WaypointR2, 1>>,
    WeightR2::WeightedError: Into<Anyhow>,
    WeightSE2: Weighted<StateSE2<G::Key, R>, DifferentialDriveLineFollowMotion>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
{
    pub fn new(
        graph: G,
        weight_r2: WeightR2,
        weight_se2: WeightSE2,
        extrapolator: DifferentialDriveLineFollow,
    ) -> Result<Self, <QuickestPathSearch<G, WeightR2> as Reversible>::ReversalError> {
        let line_follow: LineFollow = extrapolator.into();
        let motion = GraphMotion {
            space: DiscreteSpaceTimeR2::new(),
            graph: graph.clone(),
            extrapolator: line_follow,
        };

        Ok(Self {
            planner: Planner::new(Arc::new(BackwardDijkstra::new(
                &UninformedSearch::new_uninformed(
                    motion.clone(),
                    weight_r2,
                    KeyedCloser(DiscreteSpaceTimeR2::new()),
                )
                .with_initializer(InitializeR2(graph.clone()))
                .with_connector(LazyGraphMotion {
                    motion,
                    keyring: (),
                    chain: (),
                }),
            )?)),
            extrapolator,
            weight_se2,
            cost_cache: Arc::new(RwLock::new(HashMap::new())),
        })
    }

    pub fn planner(&self) -> &QuickestPathPlanner<G, WeightR2> {
        &self.planner
    }

    fn invariant_cost<State, Goal>(
        &self,
        from_state: &State,
        to_goal: &Goal,
    ) -> Result<Option<CostCache<WeightSE2::Cost, G::Key, R>>, QuickestPathHeuristicError>
    where
        WeightR2: Reversible + Weighted<StateR2<G::Key>, ArrayVec<WaypointR2, 1>>,
        WeightR2::Cost: Clone + Ord + Add<WeightR2::Cost, Output = WeightR2::Cost>,
        WeightR2::WeightedError: Into<Anyhow>,
        WeightSE2: Weighted<StateSE2<G::Key, R>, DifferentialDriveLineFollowMotion>,
        WeightSE2::Cost: Clone + Add<Output = WeightSE2::Cost> + Zero + std::fmt::Debug,
        WeightSE2::WeightedError: Into<Anyhow>,
        G: Graph + Reversible + Clone,
        G::Key: Key + Clone,
        G::Vertex: Positioned + MaybeOriented,
        G::EdgeAttributes: SpeedLimiter + Clone,
        State: Borrow<StateSE2<G::Key, R>> + std::fmt::Debug,
        Goal: Borrow<G::Key> + MaybePositioned + MaybeOriented + MaybeTimed + std::fmt::Debug,
    {
        let start: &StateSE2<G::Key, R> = from_state.borrow();
        let goal: &G::Key = to_goal.borrow();

        // First check if we've calculated this cost before
        match self.cost_cache.read() {
            Ok(cache) => match cache.get(&(start.key.clone(), goal.clone())) {
                Some(cost) => return Ok(cost.clone()),
                None => {}
            },
            Err(_) => return Err(QuickestPathHeuristicError::PoisonedMutex),
        }

        // The cost wasn't in the cache, so let's use the planner to find it.
        let invariant = 'cost: {
            let solution: Path<_, _, _> = match self
                .planner
                .plan(start.key.vertex.clone(), goal.clone())
                .map_err(|_| QuickestPathHeuristicError::PlannerError)?
                .solve()
                .map_err(|_| QuickestPathHeuristicError::PlannerError)?
                .solution()
            {
                Some(solution) => solution,
                None => break 'cost None,
            };

            let mut cost = match self
                .weight_se2
                .initial_cost(start)
                .map_err(|_| QuickestPathHeuristicError::BrokenWeight)?
            {
                Some(cost) => cost,
                None => break 'cost None,
            };
            let mut previous_state = start.clone();

            for (_, child_state) in &solution.sequence {
                // TODO(@mxgrey): Should we consider pulling information from
                // the graph for the guidance argument?
                let (action, child_wp) = match self.extrapolator.extrapolate(
                    &previous_state.waypoint,
                    &child_state.waypoint.position,
                    &(),
                ) {
                    Some(wp) => wp.map_err(|_| QuickestPathHeuristicError::Extrapolation)?,
                    None => break 'cost None,
                };

                let child_state = StateSE2::new(child_state.key.clone(), child_wp);
                let child_cost = match self
                    .weight_se2
                    .cost(&previous_state, &action, &child_state)
                    .map_err(|_| QuickestPathHeuristicError::BrokenWeight)?
                {
                    Some(cost) => cost,
                    None => break 'cost None,
                };

                cost = cost + child_cost;
                previous_state = child_state;
            }

            // Shift the time of the final state to what it would be if the
            // start time had been zero.
            previous_state.waypoint.time = TimePoint::zero()
                + (previous_state.waypoint.time - start.waypoint.time);
            Some(CostCache {
                cost,
                arrival_state: previous_state,
            })
        };

        match self.cost_cache.write() {
            Ok(mut cache) => {
                cache.insert((start.key.clone(), goal.clone()), invariant.clone());
            }
            Err(_) => return Err(QuickestPathHeuristicError::PoisonedMutex),
        }

        Ok(invariant)
    }
}

impl<G, WeightR2, WeightSE2, const R: u32, State, Goal> Informed<State, Goal>
    for QuickestPathHeuristic<G, WeightR2, WeightSE2, R>
where
    WeightR2: Reversible + Weighted<StateR2<G::Key>, ArrayVec<WaypointR2, 1>>,
    WeightR2::Cost: Clone + Ord + Add<WeightR2::Cost, Output = WeightR2::Cost>,
    WeightR2::WeightedError: Into<Anyhow>,
    WeightSE2: Weighted<StateSE2<G::Key, R>, DifferentialDriveLineFollowMotion>,
    WeightSE2::Cost: Clone + Add<Output = WeightSE2::Cost> + Zero + std::fmt::Debug,
    WeightSE2::WeightedError: Into<Anyhow>,
    G: Graph + Reversible + Clone,
    G::Key: Key + Clone,
    G::Vertex: Positioned + MaybeOriented,
    G::EdgeAttributes: SpeedLimiter + Clone,
    State: Borrow<StateSE2<G::Key, R>> + std::fmt::Debug,
    Goal: Borrow<G::Key> + MaybePositioned + MaybeOriented + MaybeTimed + std::fmt::Debug,
{
    type CostEstimate = WeightSE2::Cost;
    type InformedError = QuickestPathHeuristicError;
    fn estimate_remaining_cost(
        &self,
        from_state: &State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        dbg!((from_state, to_goal));

        // Calculate an invariant for getting to this goal from the start, no
        // matter what time it is being done.
        let invariant = match self.invariant_cost(from_state, to_goal)? {
            Some(r) => r,
            None => return Ok(None),
        };

        let start: &StateSE2<G::Key, R> = from_state.borrow();
        // Shift the time of the arrival state so that it it makes sense for the
        // starting time.
        // TODO(@mxgrey): Write unit tests specifically to make sure this time
        // shifting is working correctly.
        let arrival_state = invariant.arrival_state.time_shifted_by(
            start.time() - TimePoint::zero()
        );

        // Now add the cost of the final connection, which may vary based on time
        let child_cost = match MergeIntoGoal(self.extrapolator).connect(
            arrival_state.clone(), to_goal,
        ) {
            Some(r) => {
                let (connect, final_state): (DifferentialDriveLineFollowMotion, _) = r
                    .map_err(|_| QuickestPathHeuristicError::Extrapolation)?;

                dbg!((&arrival_state, &final_state));
                match self.weight_se2.cost(&arrival_state, &connect, &final_state)
                    .map_err(|_| QuickestPathHeuristicError::BrokenWeight)?
                {
                    Some(cost) => dbg!(cost),
                    None => {
                        dbg!();
                        return Ok(None);
                    }
                }
            }
            None => {
                dbg!();
                WeightSE2::Cost::zero()
            }
        };
        Ok(Some(invariant.cost + child_cost))
    }
}

// TODO(@mxgrey): Put actual error information inside of here.
#[derive(Debug, ThisError)]
pub enum QuickestPathHeuristicError {
    #[error("An error occurred in the planner")]
    PlannerError,
    #[error("The mutex was poisoned")]
    PoisonedMutex,
    #[error("An error occurred during SE2 extrapolation")]
    Extrapolation,
    #[error("An error occurred while calculating SE2 cost")]
    BrokenWeight,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::{
            occupancy::{Cell, SparseGrid, Visibility, VisibilityGraph},
            SharedGraph, SimpleGraph,
        },
        motion::{
            se2::{GoalSE2, Point, WaypointSE2},
            CircularProfile, TimePoint, TravelTimeCost,
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

        let heuristic: QuickestPathHeuristic<_, _, _, 100> = QuickestPathHeuristic::new(
            SharedGraph::new(graph),
            TravelTimeCost(1.0),
            TravelTimeCost(1.0),
            DifferentialDriveLineFollow::new(1.0, 1.0).unwrap(),
        )
        .unwrap();

        let start = StateSE2::new(0usize, WaypointSE2::new(TimePoint::zero(), 0.0, 0.0, 0.0));
        let goal = 8usize;

        let estimate = heuristic
            .estimate_remaining_cost(&start, &goal)
            .unwrap()
            .unwrap();

        let expected_estimate =
            5.0 + 2_f64.sqrt() + 135_f64.to_radians() * 2.0 + 90_f64.to_radians() * 2.0;
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
        let heuristic: QuickestPathHeuristic<_, _, _, 100> = QuickestPathHeuristic::new(
            SharedGraph::new(VisibilityGraph::new(visibility, [])),
            TravelTimeCost(1.0),
            TravelTimeCost(1.0),
            drive,
        )
        .unwrap();

        let state_cell = Cell::new(0, 0);
        let state_p = state_cell.to_center_point(cell_size);
        let goal_cell = Cell::new(10, 0);
        let goal_p = goal_cell.to_center_point(cell_size);
        let from_state = StateSE2::new(
            state_cell,
            WaypointSE2::new_f64(0.0, state_p.x, state_p.y, 0.0),
        );

        let goal = GoalSE2::new(goal_cell);

        let remaining_cost_estimate = heuristic
            .estimate_remaining_cost(&from_state, &goal)
            .unwrap()
            .unwrap();
        assert_relative_eq!(
            (goal_p - state_p).norm() / drive.translational_speed(),
            remaining_cost_estimate.0,
            max_relative = 0.01,
        );
    }

    #[test]
    fn test_obstructed_freespace_quickest_path() {
        let cell_size = 1.0;
        let profile = CircularProfile::new(0.75, 0.0, 0.0).unwrap();
        let visibility = Arc::new({
            let mut vis = Visibility::new(SparseGrid::new(cell_size), profile.footprint_radius());
            vis.change_cells(
                &(-10..=-1_i64)
                    .into_iter()
                    .map(|y| (Cell::new(3, y), true))
                    .collect(),
            );
            vis
        });

        let drive = DifferentialDriveLineFollow::new(3.0, 1.0).unwrap();
        let heuristic: QuickestPathHeuristic<_, _, _, 360> = QuickestPathHeuristic::new(
            SharedGraph::new(VisibilityGraph::new(visibility, [])),
            TravelTimeCost(1.0),
            TravelTimeCost(1.0),
            drive,
        )
        .unwrap();

        let from_state = StateSE2::new(
            Cell::new(-6, -3),
            WaypointSE2::new_f64(1.256802684, -5.5, -2.5, 45_f64.to_radians()),
        );

        let goal = GoalSE2::new(Cell::new(18, 3));

        heuristic
            .estimate_remaining_cost(&from_state, &goal)
            .unwrap()
            .unwrap();
    }
}
