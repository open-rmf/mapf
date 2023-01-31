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
    algorithm::{Algorithm, Coherent},
    expander::{Goal, InitTargeted, InitTargetedErrorOf, Solvable, Targeted},
    search::{self, Search},
    halt::Halt,
    trace::{NoTrace, Trace},
};
use anyhow;
use std::{cell::RefCell, sync::Arc};

/// The Planner class manages an (algorithm, expander) pair to generate plans.
/// Splitting the implementation of the Planner into algorithm and expander
/// components allows for the largest possible amount of code reusability.
///
/// The Planner::plan(start, goal) function will create a Progress object which
/// manages the planning progress and allows you to tweak planning settings
/// during runtime as needed.
pub struct Planner<A: Algorithm, H: Halt<A> = ()> {
    /// The object which determines the search pattern
    algorithm: Arc<A>,

    /// The factory for generating progress objects
    default_halt: H,
}

impl<A: Algorithm, H: Halt<A>> Planner<A, H> {
    /// Construct a new planner with the given configuration and a set of
    /// default options.
    pub fn new(algorithm: A) -> Self
    where
        H: Default,
    {
        Self {
            algorithm: Arc::new(algorithm),
            default_halt: Default::default(),
        }
    }

    pub fn new_with_halt(algorithm: A, halt: H) -> Self {
        Self {
            algorithm: Arc::new(algorithm),
            default_halt: halt,
        }
    }

    /// Begin planning from the start conditions to the goal conditions.
    pub fn plan<Start, Goal>(
        &self,
        start: Start,
        goal: Goal,
    ) -> Result<Search<A, H>, A::InitError>
    where
        A: Coherent<Start, Goal>,
    {
        self.plan_with_options(start, goal, self.default_options.clone())
    }

    pub fn plan_with_halt<Start, Goal, WithHalt: Halt<A>>(
        &self,
        start: Start,
        goal: Goal,
        halt: WithHalt,
    ) -> Result<Search<A, WithHalt>, A::InitError>
    where
        A: Coherent<Start, Goal>,
    {
        let mut trace = NoTrace::default();
        let memory = self.algorithm.initialize(start, goal)?;

        Ok(Search::new(
            memory,
            self.algorithm.clone(),
            halt,
        ))
    }

    pub fn into_abstract<S, G>(self) -> Abstract<S, G, A::Solution>
    where
        A: Coherent<S, G> + 'static,
        H: 'static,
    {
        Abstract {
            implementation: Box::new(RefCell::new(self)),
        }
    }
}

pub trait Interface<S, G, Solution> {
    fn plan(&self, start: &S, goal: G) -> anyhow::Result<search::Abstract<Solution>>;
}

impl<A, H, S, G> Interface<S, G, A::Solution> for Planner<A, H>
where
    A: Algorithm + Coherent<S, G> + 'static,
    H: Halt<A> + 'static,
{
    fn plan(&self, start: &S, goal: G) -> anyhow::Result<search::Abstract<A::Solution>> {
        Planner::plan(self, start, goal)
            .map(Search::into_abstract)
            .map_err(anyhow::Error::new)
    }
}

pub struct Abstract<S, G, Solution> {
    implementation: Box<RefCell<dyn Interface<S, G, Solution>>>,
}

impl<S, G, Solution> Interface<S, G, Solution> for Abstract<S, G, Solution> {
    fn plan(&self, start: &S, goal: G) -> anyhow::Result<search::Abstract<Solution>> {
        self.implementation.borrow_mut().plan(start, goal)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::{
        algorithm::{Status, StepError, WeightSorted},
        error::NoError,
        expander::{
            Closable, CostOf, Expander, ExpansionErrorOf, Goal, InitTargeted, Solvable, Targeted,
        },
        node::{self, traits::*},
    };

    struct CountingNode {
        value: u64,
        cost: u64,
        remaining_cost_estimate: u64,
        parent: Option<Arc<Self>>,
    }

    impl PartialKeyed for CountingNode {
        type Key = u64;

        fn partial_key(&self) -> Option<&Self::Key> {
            Some(&self.value)
        }
    }

    impl PathSearch for CountingNode {
        fn parent(&self) -> &Option<Arc<Self>> {
            return &self.parent;
        }
    }

    impl Weighted for CountingNode {
        type Cost = u64;
        fn cost(&self) -> u64 {
            return self.cost;
        }
    }

    impl node::Informed for CountingNode {
        fn remaining_cost_estimate(&self) -> u64 {
            return self.remaining_cost_estimate;
        }
    }

    #[derive(Clone, Default)]
    struct CountingExpanderOptions;

    #[derive(Debug)]
    struct CountingExpander;

    struct CountingGoal {
        value: u64,
    }

    impl Goal<CountingNode> for CountingGoal {
        fn is_satisfied(&self, node: &CountingNode) -> bool {
            return node.value == self.value;
        }
    }

    struct CountingExpansion<'a> {
        next_node: Option<Arc<CountingNode>>,
        _ignore: std::marker::PhantomData<&'a u8>,
    }

    impl<'a> Iterator for CountingExpansion<'a> {
        type Item = Result<Arc<CountingNode>, NoError>;

        fn next(&mut self) -> Option<Self::Item> {
            return self.next_node.take().map(|v| Ok(v));
        }
    }

    #[derive(Debug, Clone)]
    struct CountingSolution {
        cost: u64,
        sequence: std::vec::Vec<u64>,
    }

    impl node::Weighted for CountingSolution {
        type Cost = u64;
        fn cost(&self) -> u64 {
            self.cost
        }
    }

    impl InitTargeted<u64, CountingGoal> for CountingExpander {
        type InitTargetedError = NoError;
        type InitialTargetedNodes<'a> = CountingExpansion<'a>;

        fn start<'a>(&'a self, start: &u64, goal: &CountingGoal) -> Self::InitialTargetedNodes<'a> {
            if *start <= goal.value {
                return CountingExpansion {
                    next_node: Some(Arc::new(CountingNode {
                        value: *start,
                        cost: 0u64,
                        remaining_cost_estimate: goal.value - start,
                        parent: None,
                    })),
                    _ignore: Default::default(),
                };
            }

            return CountingExpansion {
                next_node: None,
                _ignore: Default::default(),
            };
        }
    }

    impl Targeted<CountingGoal> for CountingExpander {
        type TargetedError = NoError;
        type TargetedExpansion<'a> = CountingExpansion<'a>;

        fn expand<'a>(
            &'a self,
            parent: &Arc<CountingNode>,
            goal: &CountingGoal,
        ) -> Self::TargetedExpansion<'a> {
            if parent.value <= goal.value {
                return CountingExpansion {
                    next_node: Some(Arc::new(CountingNode {
                        value: parent.value + 1,
                        cost: parent.cost + 1,
                        remaining_cost_estimate: goal.value - parent.value,
                        parent: Some(parent.clone()),
                    })),
                    _ignore: Default::default(),
                };
            }

            return CountingExpansion {
                next_node: None,
                _ignore: Default::default(),
            };
        }
    }

    impl Solvable for CountingExpander {
        type SolveError = NoError;
        type Solution = CountingSolution;

        fn make_solution(
            &self,
            solution_node: &Arc<CountingNode>,
        ) -> Result<Self::Solution, NoError> {
            let mut solution = std::vec::Vec::<u64>::new();
            let mut next: Option<Arc<CountingNode>> = Some(solution_node.clone());
            while let Some(n) = next {
                solution.push(n.value);
                next = n.parent.clone();
            }

            solution.reverse();
            Ok(CountingSolution {
                cost: solution_node.cost,
                sequence: solution,
            })
        }
    }

    impl Closable for CountingExpander {
        type ClosedSet = node::PartialKeyedClosedSet<CountingNode>;
    }

    impl Expander for CountingExpander {
        type Node = CountingNode;
    }

    struct TestAlgorithmMemory<Expander: Solvable> {
        expander: Arc<Expander>,
        queue: std::vec::Vec<Arc<Expander::Node>>,
    }

    impl<E: Solvable> Memory for TestAlgorithmMemory<E> {
        fn node_count(&self) -> usize {
            return self.queue.len();
        }
    }

    impl<E: Expander<Node: Weighted> + Solvable> WeightSorted<E> for TestAlgorithmMemory<E> {
        fn top_cost_estimate(&self) -> Option<CostOf<E>> {
            self.queue.last().map(|v| v.cost())
        }
    }

    #[derive(Default, Debug)]
    struct TestAlgorithm;
    impl<E: Solvable> Algorithm<E> for TestAlgorithm {
        type Memory = TestAlgorithmMemory<E>;
        type InitError = NoError;
        type StepError = NoError;

        fn initialize<S, G: Goal<E::Node>, T: Trace<E::Node>>(
            &self,
            expander: Arc<E>,
            start: &S,
            goal: &G,
            trace: &mut T,
        ) -> Result<Self::Memory, InitError<Self::InitError, InitTargetedErrorOf<E, S, G>>>
        where
            E: InitTargeted<S, G>,
        {
            let mut queue: std::vec::Vec<Arc<E::Node>> = Vec::new();
            for node in expander.start(start, goal) {
                let node = node.map_err(InitError::Expander)?;
                trace.expanded_to(&node);
                queue.push(node);
            }

            Ok(Self::Memory { expander, queue })
        }

        fn step<G: Goal<E::Node>, T: Trace<E::Node>>(
            &self,
            memory: &mut Self::Memory,
            goal: &G,
            tracker: &mut T,
        ) -> Result<
            Status<E::Solution>,
            StepError<Self::StepError, ExpansionErrorOf<E, G>, E::SolveError>,
        >
        where
            E: Targeted<G>,
        {
            let top_opt = memory.queue.pop();
            if let Some(top) = top_opt {
                if goal.is_satisfied(&top) {
                    tracker.solution_found_from(&top);
                    return memory
                        .expander
                        .make_solution(&top)
                        .map(Status::Solved)
                        .map_err(StepError::Solve);
                }

                for node in memory.expander.expand(&top, goal) {
                    let node = node.map_err(StepError::Expansion)?;
                    memory.queue.push(node);
                }

                if memory.queue.is_empty() {
                    return Ok(Status::Impossible);
                }

                return Ok(Status::Incomplete);
            } else {
                return Ok(Status::Impossible);
            }
        }
    }

    type CountingPlanner = Planner<CountingExpander, TestAlgorithm>;

    #[test]
    fn counting_expander_can_reach_a_higher_goal() {
        let planner = CountingPlanner::new(Arc::new(CountingExpander {}));
        let start = 5;
        let goal = 10;
        let result = planner
            .plan(&start, CountingGoal { value: goal })
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
        let planner = CountingPlanner::new(Arc::new(CountingExpander {}));
        let start = 10;
        let goal = 5;
        let result = planner
            .plan(&start, CountingGoal { value: goal })
            .unwrap()
            .solve()
            .unwrap();
        assert!(matches!(result, Status::Impossible));
    }

    #[test]
    fn planner_incomplete_after_insufficient_steps() {
        let planner = CountingPlanner::new(Arc::new(CountingExpander {}));
        let start = 5;
        let goal = 10;
        let mut progress = planner.plan(&start, CountingGoal { value: goal }).unwrap();
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Incomplete));
        assert!(matches!(progress.step().unwrap(), Status::Solved(_)));
    }
}
