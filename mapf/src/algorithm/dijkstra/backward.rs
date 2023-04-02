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
    domain::{
        Domain, Reversible, Keyed, Closable, Activity, Weighted, Initializable,
        Keyring, ClosedStatusForKey, Backtrack, ArrivalKeyring, Connectable,
        Configurable,
    },
    algorithm::{
        Algorithm, Coherent, Solvable, SearchStatus, Path,
        dijkstra::{
            Dijkstra,
            forward::{Memory, DijkstraSearchError},
        },
    },
    error::ThisError,
};
use std::ops::Add;

pub struct BackwardDijkstra<D: Reversible>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    backward: Dijkstra<D>,
}

impl<D: Reversible> BackwardDijkstra<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    pub fn new(domain: &D) -> Result<Self, D::ReversalError> {
        Ok(Self { backward: Dijkstra::new(domain.reversed()?) })
    }

    pub fn backward(&self) -> &Dijkstra<D> {
        &self.backward
    }
}

impl<D: Reversible> Algorithm for BackwardDijkstra<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    type Memory = BackwardMemory<D>;
}

pub struct BackwardMemory<D: Reversible>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    backward: Memory<D>,
}

impl<D: Reversible, Start, Goal> Coherent<Start, Goal> for BackwardDijkstra<D>
where
    D: Domain
    + Keyring<D::State>
    + Initializable<Goal, Start, D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
    + Connectable<D::State, D::ActivityAction, D::Key>
    + ArrivalKeyring<D::Key, Goal, Start>,
    D::ClosedSet<usize>: ClosedStatusForKey<D::Key, usize>,
    D::ActivityAction: Clone,
    D::InitialError: Into<D::Error>,
    D::ArrivalKeyError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::ConnectionError: Into<D::Error>,
    D::State: Clone,
    D::Cost: Clone + Ord + Add<D::Cost, Output = D::Cost>,
    D::Key: Clone,
    Start: Clone,
    Goal: Clone,
{
    type InitError = DijkstraSearchError<D::Error>;

    fn initialize(
        &self,
        start: Start,
        goal: &Goal,
    ) -> Result<Self::Memory, Self::InitError> {
        let memory = self.backward.initialize(goal.clone(), &start)?;
        Ok(BackwardMemory {
            backward: memory,
        })
    }
}

impl<D, Goal> Solvable<Goal> for BackwardDijkstra<D>
where
    D: Domain
    + Reversible
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Keyring<D::State>
    + Closable<D::State>
    + Connectable<D::State, D::ActivityAction, D::Key>
    + Backtrack<
        D::State,
        D::ActivityAction,
    >,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Cost: Clone + Ord + Add<D::Cost, Output = D::Cost>,
    D::ClosedSet<usize>: ClosedStatusForKey<D::Key, usize>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::ConnectionError: Into<D::Error>,
    D::BacktrackError: Into<D::Error>,
{
    type Solution = Path<D::State, D::ActivityAction, D::Cost>;
    type StepError = DijkstraSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        _: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError> {
        // Note: Passing in the goal doesn't matter because the Dijkstra
        // algorithm memory saves the goal information anyway.
        self.backward.step(&mut memory.backward, &())
            .and_then(|r|
                // If a solution is found, backtrack the path to make it run
                // forward instead of being in reverse.
                r.and_then(|path| path.backtrack(self.backward.domain()))
                    .map_err(DijkstraSearchError::Domain)
            )
    }
}

/// Note that configuring a Dijkstra algorithm will clear the cache. If the
/// underlying configuration changes then we cannot assume that any of the
/// previous search remains valid.
impl<D: Configurable> Configurable for BackwardDijkstra<D>
where
    D: Domain
    + Reversible
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    type Configuration = D::Configuration;
    type ConfigurationError = BackwardConfigurationError<D::ReversalError, D::ConfigurationError>;
    fn configure<F>(self, f: F) -> Result<Self, Self::ConfigurationError>
    where
        F: FnOnce(Self::Configuration) -> Self::Configuration,
    {
        let configured = self.backward.domain().reversed()
            .map_err(BackwardConfigurationError::Reverse)?
            .configure(f)
            .map_err(BackwardConfigurationError::Configure)?;

        Self::new(&configured).map_err(BackwardConfigurationError::Reverse)
    }
}

#[derive(Debug, ThisError)]
pub enum BackwardConfigurationError<R, C> {
    #[error("An error occurred while reversing the domain:\n{0:?}")]
    Reverse(R),
    #[error("An error occured while configuring the forward domain:\n{0:?}")]
    Configure(C),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::{SimpleGraph, SharedGraph},
        motion::{
            se2::*,
            TravelTimeCost,
        },
        templates::{UninformedSearch, GraphMotion, LazyGraphMotion},
        domain::KeyedCloser,
    };
    use std::sync::Arc;

    #[test]
    fn test_dijkstra_same_start_and_goal_se2() {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
         */

        let graph = SharedGraph::new(SimpleGraph::from_iters(
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
        ));

        let extrapolator = DifferentialDriveLineFollow::new(1.0, 1.0).unwrap();
        let weight = TravelTimeCost(1.0);
        let motion = GraphMotion {
            space: DiscreteSpaceTimeSE2::<usize, 100>::new(),
            graph: graph.clone(),
            extrapolator,
        };

        let planner = quickest_path::QuickestPathPlanner::new(
            Arc::new(
                BackwardDijkstra::new(
                    &UninformedSearch::new_uninformed(
                        motion.clone(),
                        weight,
                        KeyedCloser(DiscreteSpaceTimeSE2::new()),
                    )
                    .with_initializer(PreferentialStarburstSE2::for_start(graph.clone()))
                    .with_satisfier(PreferentialStarburstSE2::for_goal(graph).unwrap())
                    .with_connector(
                        LazyGraphMotion {
                            motion,
                            keyring: (),
                            chain: MergeIntoGoal(extrapolator),
                        }
                    )
                ).unwrap()
            )
        );

        for i in 0..=8 {
            for angle in [0.0, 90.0, 180.0, 30.0, -140.0_f64] {
                let start_key = KeySE2::new(i, angle.to_radians());
                let result = planner.plan(start_key, i).unwrap().solve().unwrap();
                let solution = result.solution().unwrap();
                assert_eq!(solution.total_cost.0, 0.0);
            }
        }
    }
}
