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

pub mod search;
pub use search::{Search, AbstractSearch};

pub mod halt;
pub use halt::Halt;

use crate::{
    algorithm::{Coherent, Solvable},
    error::Anyhow,
};
use anyhow;
use std::cell::RefCell;

/// The Planner class spawns Search instances to find plans using its provided
/// algorithm.
///
/// The Planner::plan(start, goal) function will create a Search object which
/// manages the planning progress and allows you to tweak planning settings
/// during runtime as needed.
///
/// The Planner can also be given default halting behavior which will be passed
/// along to the Search when `plan(start, goal)` is used. The Halting generic
/// must implement the [`Halt`] trait for `Algo`.
#[derive(Debug, Clone)]
pub struct Planner<Algo, Halting = ()> {
    /// The object which determines the search pattern
    algorithm: Algo,

    /// The factory for generating progress objects
    default_halting: Halting,
}

impl<Algo> Planner<Algo, ()> {
    /// Construct a new planner that has no halting behavior.
    pub fn new(algorithm: Algo) -> Self {
        Self {
            algorithm,
            default_halting: (),
        }
    }
}

impl<Algo, Halting> Planner<Algo, Halting> {
    pub fn new_haltable(algorithm: Algo, halting: Halting) -> Self {
        Self {
            algorithm,
            default_halting: halting,
        }
    }

    /// Consume this Planner and create a new Planner with a different default
    /// Halting value.
    pub fn with_halting<NewHalting>(self, halting: NewHalting) -> Planner<Algo, NewHalting> {
        Planner {
            algorithm: self.algorithm,
            default_halting: halting,
        }
    }

    /// Consume this Planner, modify its Algorithm, and return a new planner
    /// with the modified algorithm.
    ///
    /// To fork off a new Planner while preserving the previous Planner, use
    /// `.clone()` before `.map(~)`. Note that requires the Algorithm to be
    /// clonable.
    pub fn map<NewAlgo, F: FnOnce(Algo) -> NewAlgo>(self, f: F) -> Planner<NewAlgo, Halting> {
        Planner {
            algorithm: f(self.algorithm),
            default_halting: self.default_halting,
        }
    }

    /// Begin planning from the start conditions to the goal conditions.
    ///
    /// This requires the Algorithm and Halting to be clonable. To produce a
    /// single search using an Algorithm that cannot be cloned, use
    /// [`Planner::into_search`].
    pub fn plan<Start, Goal>(
        &self,
        start: Start,
        goal: Goal,
    ) -> Result<Search<Algo, Goal, Halting>, Algo::InitError>
    where
        Algo: Coherent<Start, Goal> + Clone,
        Halting: Halt<Algo::Memory> + Clone,
    {
        self.plan_with_halting(start, goal, self.default_halting.clone())
    }

    /// Begin planning from the start conditions to the goal conditions using
    /// a custom halting behavior.
    ///
    /// This requires the Algorithm to be clonable. To produce a single search
    /// using an Algorithm that cannot be cloned, use [`Planner::into_search`].
    pub fn plan_with_halting<Start, Goal, WithHalt: Halt<Algo::Memory>>(
        &self,
        start: Start,
        goal: Goal,
        halt: WithHalt,
    ) -> Result<Search<Algo, Goal, WithHalt>, Algo::InitError>
    where
        Algo: Coherent<Start, Goal> + Clone,
    {
        let memory = self.algorithm.initialize(start, &goal)?;

        Ok(Search::new(
            memory,
            self.algorithm.clone(),
            goal,
            halt,
        ))
    }

    /// Convert the planner into a single [`Search`] instance. This can be used
    /// for Algorithms that don't implement the [`Clone`] trait.
    ///
    /// To produce multiple searches, use [`Planner::plan`] instead.
    pub fn into_search<Start, Goal>(
        self,
        start: Start,
        goal: Goal,
    ) -> Result<Search<Algo, Goal, Halting>, Algo::InitError>
    where
        Algo: Coherent<Start, Goal>,
        Halting: Halt<Algo::Memory>,
    {
        let memory = self.algorithm.initialize(start, &goal)?;

        Ok(Search::new(
            memory,
            self.algorithm,
            goal,
            self.default_halting,
        ))
    }

    /// Convert this Planner into an abstract one which hides the underlying
    /// algorithm. This can be useful for mixing this planner into a container
    /// with other planners that support the same input/output but use
    /// different algorithms.
    pub fn into_abstract<S, G>(self) -> AbstractPlanner<S, G, Algo::Solution>
    where
        Algo: Coherent<S, G> + Solvable<G> + Clone + 'static,
        Algo::InitError: Into<Anyhow> + 'static,
        Algo::StepError: Into<Anyhow> + 'static,
        Halting: Halt<Algo::Memory> + 'static,
        G: 'static,
    {
        AbstractPlanner {
            implementation: Box::new(RefCell::new(self)),
        }
    }
}

pub trait PlannerInterface<S, G, Solution> {
    fn plan(&self, start: S, goal: G) -> anyhow::Result<AbstractSearch<Solution>>;
}

impl<A, H, S, G> PlannerInterface<S, G, A::Solution> for Planner<A, H>
where
    A: Solvable<G> + Coherent<S, G> + Clone + 'static,
    A::InitError: Into<Anyhow>,
    A::StepError: Into<Anyhow>,
    H: Halt<A::Memory> + 'static,
    G: 'static,
{
    fn plan(&self, start: S, goal: G) -> anyhow::Result<AbstractSearch<A::Solution>> {
        Planner::plan(self, start, goal)
            .map(Into::into)
            .map_err(Into::into)
    }
}

pub struct AbstractPlanner<S, G, Solution> {
    implementation: Box<RefCell<dyn PlannerInterface<S, G, Solution>>>,
}

impl<S, G, Solution> PlannerInterface<S, G, Solution> for AbstractPlanner<S, G, Solution> {
    fn plan(&self, start: S, goal: G) -> anyhow::Result<AbstractSearch<Solution>> {
        self.implementation.borrow_mut().plan(start, goal)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::{
        algorithm::{Algorithm, Status, MinimumCostBound},
        error::NoError,
    };
    use std::sync::Arc;

    struct CountingNode {
        value: u64,
        cost: u64,
        parent: Option<Arc<Self>>,
    }

    #[derive(Debug, Clone)]
    struct CountingSolution {
        #[allow(dead_code)] cost: u64,
        sequence: Vec<u64>,
    }

    impl From<Arc<CountingNode>> for CountingSolution {
        fn from(value: Arc<CountingNode>) -> Self {
            let cost = value.cost;
            let mut sequence = Vec::<u64>::new();
            let mut gather = Some(value);
            while let Some(next) = gather {
                sequence.push(next.value);
                gather = next.parent.clone();
            }
            sequence.reverse();

            CountingSolution { cost, sequence }
        }
    }

    struct TestAlgorithmMemory {
        queue: std::vec::Vec<Arc<CountingNode>>,
    }

    impl MinimumCostBound for TestAlgorithmMemory {
        type Cost = u64;
        fn minimum_cost_bound(&self) -> Option<Self::Cost> {
            self.queue.first().map(|n| n.cost)
        }
    }

    #[derive(Default, Debug, Clone)]
    struct CountingAlgorithm;
    impl Algorithm for CountingAlgorithm {
        type Memory = TestAlgorithmMemory;
    }
    impl Solvable<u64> for CountingAlgorithm {
        type StepError = NoError;
        type Solution = CountingSolution;

        fn step(
            &self,
            memory: &mut Self::Memory,
            goal: &u64,
        ) -> Result<Status<Self::Solution>, Self::StepError> {
            let top = match memory.queue.pop() {
                Some(top) => top,
                None => return Ok(Status::Impossible),
            };

            if top.value == *goal {
                return Ok(Status::Solved(top.into()));
            }

            if top.value > *goal {
                return Ok(Status::Impossible);
            }

            memory.queue.push(Arc::new(CountingNode {
                value: top.value + 1,
                cost: top.cost + 1,
                parent: Some(top),
            }));
            Ok(Status::Incomplete)
        }
    }

    impl Coherent<u64, u64> for CountingAlgorithm {
        type InitError = NoError;

        fn initialize(
            &self,
            start: u64,
            _: &u64,
        ) -> Result<Self::Memory, Self::InitError> {
            let queue = vec![Arc::new(CountingNode {
                value: start,
                cost: 0,
                parent: None,
            })];

            Ok(TestAlgorithmMemory { queue })
        }
    }

    #[test]
    fn counting_expander_can_reach_a_higher_goal() {
        let planner = Planner::new(CountingAlgorithm);
        let start = 5;
        let goal = 10;
        let result = planner
            .plan(start, goal)
            .unwrap()
            .solve()
            .unwrap();
        assert!(matches!(result, Status::Solved(_)));
        if let Status::Solved(solution) = result {
            assert!(solution.sequence.len() == (goal - start + 1) as usize);
            assert!(solution.sequence.first() == Some(&start));
            assert!(solution.sequence.last() == Some(&goal));
        }
    }

    #[test]
    fn counting_expander_finds_lower_goal_impossible() {
        let planner = Planner::new(CountingAlgorithm);
        let start = 10;
        let goal = 5;
        let result = planner
            .plan(start, goal)
            .unwrap()
            .solve()
            .unwrap();
        assert!(matches!(result, Status::Impossible));
    }

    #[test]
    fn planner_incomplete_after_insufficient_steps() {
        let planner = Planner::new(CountingAlgorithm);
        let start = 5;
        let goal = 10;
        let mut progress = planner.plan(start, goal).unwrap();
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Solved(_)));
    }
}
