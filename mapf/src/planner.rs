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

use std::rc::Rc;
use std::ops::FnMut;
use std::boxed::Box;

use super::expander;
use super::expander::Cost;
use super::algorithm;
use super::algorithm::{Result, Storage};
use super::node;
use super::tracker;

pub struct Planner<Expander, Algorithm>  where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander>, {

    /// The object which determines the search pattern
    algorithm: Rc<Algorithm>,

    /// The object which determines patterns for expansion
    expander: Rc<Expander>,
}

impl<Expander, Algorithm> Planner<Expander, Algorithm>  where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander> {

    /// Construct a new planner with the given configuration and a set of
    /// default options.
    pub fn new(
        expander: Rc<Expander>
    ) -> Self
    where Algorithm: Default {
        return Self {
            algorithm: Rc::new(Algorithm::default()),
            expander
        }
    }

    pub fn with_algorithm(
        algorithm: Rc<Algorithm>,
        expander: Rc<Expander>
    ) -> Self {
        return Self { algorithm, expander }
    }

    pub fn plan(
        &self,
        start: &Expander::Start,
        goal: Expander::Goal,
    ) -> Progress<Expander, Algorithm, tracker::NoDebug> {
        let mut tracker = tracker::NoDebug::default();
        let storage = self.algorithm.initialize(
            self.expander.clone(), &start, &goal, &mut tracker
        );

        return Progress {
            storage,
            algorithm: self.algorithm.clone(),
            progression_options: ProgressionOptions::default(),
            expansion_options: self.expander.default_options(),
            tracker
        }
    }

    pub fn debug<Tracker>(
        &self,
        start: &Expander::Start,
        goal: Expander::Goal,
        mut tracker: Tracker
    ) -> Progress<Expander, Algorithm, Tracker>
    where Tracker: tracker::Tracker<Expander::Node> {

        let storage = self.algorithm.initialize(
            self.expander.clone(), &start, goal, &mut tracker
        );

        return Progress {
            storage,
            algorithm: self.algorithm.clone(),
            progression_options: ProgressionOptions::default(),
            expansion_options: self.expander.default_options(),
            tracker,
        }
    }
}

pub enum Interruption {
    Continue,
    Stop
}

pub type Interrupter = Box<dyn FnMut() -> Interruption>;

pub struct ProgressionOptions<Expander: expander::Expander> {

    /// A function that can decide to interrupt the planning
    pub interrupter: Option<Interrupter>,

    /// The maximum total cost estimate that the top node can reach before
    /// the solve attempt quits.
    pub max_cost_estimate: Option<Cost<Expander>>,

    /// The maximum size that the search queue can reach before the
    /// solve attemptp quits.
    pub search_queue_limit: Option<usize>,
}

impl<Expander: expander::Expander> Default for ProgressionOptions<Expander> {
    fn default() -> Self {
        return Self {
            interrupter: None,
            max_cost_estimate: None,
            search_queue_limit: None,
        }
    }
}

/// Progress manages the progress of a planning effort.
pub struct Progress<Expander, Algorithm, Tracker> where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander>,
    Tracker: tracker::Tracker<Expander::Node> {

    /// Storage container for the progress of the search algorithm
    storage: Algorithm::Storage,

    /// The object which determines the search pattern
    algorithm: Rc<Algorithm>,

    /// The options that moderate the progress of the solving
    progression_options: ProgressionOptions<Expander>,

    /// The options that should be used by the expander
    expansion_options: Expander::Options,

    /// The object which tracks planning progress
    tracker: Tracker
}

impl<Expander, Algorithm, Tracker>
Progress<Expander, Algorithm, Tracker>
where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander>,
    Tracker: tracker::Tracker<Expander::Node> {

    pub fn with_expansion_options(&mut self, expansion_opts: Expander::Options)
    -> &mut Self {
        self.expansion_options = expansion_opts;
        return self;
    }

    pub fn with_progression_options(
        &mut self,
        progression_opts: ProgressionOptions<Expander>
    ) -> &mut Self {
        self.progression_options = progression_opts;
        return self;
    }

    pub fn with_interrupter(
        &mut self,
        interrupter: Option<Interrupter>
    ) -> &mut Self {
        self.progression_options.interrupter = interrupter;
        return self;
    }

    pub fn with_max_cost_estimate(
        &mut self,
        max_cost_estimate: Option<Cost<Expander>>
    ) -> &mut Self {
        self.progression_options.max_cost_estimate = max_cost_estimate;
        return self;
    }

    pub fn with_search_queue_limit(
        &mut self,
        search_queue_limit: Option<usize>
    ) -> &mut Self {
        self.progression_options.search_queue_limit = search_queue_limit;
        return self;
    }

    pub fn expansion_options_mut(&mut self) -> &mut Expander::Options {
        return &mut self.expansion_options;
    }

    pub fn expansion_options(&self) -> &Expander::Options {
        return &self.expansion_options;
    }

    pub fn progression_options_mut(&mut self) -> &mut ProgressionOptions<Expander> {
        return &mut self.progression_options;
    }

    pub fn progression_options(&self) -> &ProgressionOptions<Expander> {
        return &self.progression_options;
    }

    pub fn solve(&mut self) -> Result<Expander> {
        loop {
            if self.need_to_interrupt() {
                return Result::Incomplete;
            }

            let result = self.step();
            if let Result::Incomplete = result {
                continue;
            }

            return result;
        }
    }

    pub fn step(&mut self) -> Result<Expander> {
        return self.algorithm.step(
            &mut self.storage, &self.expansion_options, &mut self.tracker
        );
    }

    fn need_to_interrupt(&mut self) -> bool {
        if let Some(interrupter) = &mut self.progression_options.interrupter {
            if let Interruption::Stop = interrupter() {
                return true;
            }
        }

        if let Some(max_cost_estimate) = &self.progression_options.max_cost_estimate {
            if let Some(top_cost_estimate) = self.storage.top_cost_estimate() {
                if top_cost_estimate > *max_cost_estimate {
                    return true;
                }

                // If the top_cost_estimate is None then the algorithm ought to
                // return Impossible later, which is more informative than
                // the Incomplete that would get returned to the user if we
                // returned true here.
            }
        }

        if let Some(search_queue_limit) = &self.progression_options.search_queue_limit {
            if self.storage.node_count() > *search_queue_limit {
                return true;
            }
        }

        return false;
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use super::node::Node;
    use std::hash::Hash;

    struct CountingNode {
        value: u64,
        cost: u64,
        remaining_cost_estimate: u64,
        parent: Option<Rc<Self>>,
    }

    impl Hash for CountingNode {
        fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
            state.write_u64(self.value);
        }
    }

    impl node::Node for CountingNode {
        type ClosedSet = node::HashClosedSet<Self>;
        type Cost = u64;

        fn cost(&self) -> u64 {
            return self.cost;
        }
        fn parent(&self) -> &Option<Rc<Self>> {
            return &self.parent;
        }
    }

    impl node::Informed for CountingNode {
        fn remaining_cost_estimate(&self) -> u64 {
            return self.remaining_cost_estimate;
        }
        fn total_cost_estimate(&self) -> u64 {
            return self.cost + self.remaining_cost_estimate;
        }
    }

    #[derive(Clone, Default)]
    struct CountingExpanderOptions;
    struct CountingExpander;

    struct CountingGoal {
        value: u64
    }

    impl expander::Goal<CountingNode> for CountingGoal {
        fn is_satisfied(&self, node: &CountingNode) -> bool {
            return node.value == self.value;
        }
    }

    struct CountingExpansion {
        next_node: Option<Rc<CountingNode>>
    }

    impl Iterator for CountingExpansion {
        type Item = Rc<CountingNode>;

        fn next(&mut self) -> Option<Self::Item> {
            return self.next_node.take();
        }
    }

    impl expander::Expander for CountingExpander {
        type Start = u64;
        type Goal = CountingGoal;
        type Node = CountingNode;
        type Options = CountingExpanderOptions;
        type Solution = std::vec::Vec<u64>;
        type Expansion = CountingExpansion;

        fn default_options(&self) -> Self::Options {
            return Self::Options::default();
        }

        fn start(&self, start: &u64, goal: &Self::Goal) -> Self::Expansion {
            return CountingExpansion {
                next_node: Some(
                    Rc::new(
                        CountingNode{
                            value: start,
                            cost: 0u64,
                            remaining_cost_estimate: goal.value - start,
                            parent: None
                        }
                    )
                )
            }
        }

        fn expand(
            &self,
            parent: &Rc<Self::Node>,
            goal: &Self::Goal,
            options: &Self::Options
        ) -> Self::Expansion {
            if parent.value <= goal.value {
                return CountingExpansion{
                    next_node: Some(
                        Rc::new(
                            CountingNode{
                                value: parent.value+1,
                                cost: parent.cost+1,
                                remaining_cost_estimate: goal.value - parent.value,
                                parent: Some(parent.clone())
                            }
                        )
                    )
                };
            }

            return CountingExpansion{next_node: None};
        }

        fn make_solution(
            &self,
            solution_node: &Rc<Self::Node>,
            options: &Self::Options
        ) -> Self::Solution {
            let mut solution = std::vec::Vec::<u64>::new();
            let mut next: Option<Rc<Self::Node>> = Some(solution_node.clone());
            while let Some(n) = next {
                solution.push(n.value);
                next = n.parent;
            }

            return solution;
        }
    }

    struct TestAlgorithmStorage<Expander: expander::Expander> {
        expander: Rc<Expander>,
        goal: Expander::Goal,
        queue: std::vec::Vec<Rc<Expander::Node>>,
    }

    impl<Expander: expander::Expander> Storage<Expander> for TestAlgorithmStorage<Expander> {
        fn node_count(&self) -> usize {
            return self.queue.len();
        }
        fn top_cost_estimate(&self) -> Option<expander::Cost<Expander>> {
            if let Some(last) = self.queue.last() {
                return Some((*last).cost())
            }

            return None;
        }
    }

    struct TestAlgorithm;
    impl<Expander: expander::Expander>
    algorithm::Algorithm<Expander> for TestAlgorithm {
        type Storage = TestAlgorithmStorage<Expander>;

        fn initialize<Tracker: tracker::Tracker<Expander::Node>>(
            &self,
            expander: Rc<Expander>,
            start: &Expander::Start,
            goal: Expander::Goal,
            tracker: &mut Tracker,
        ) -> Self::Storage {
            let mut queue: std::vec::Vec<Rc<Expander::Node>> = Vec::new();
            for node in expander.start(start, &goal) {
                queue.push(node);
            }

            return Self::Storage{ expander, goal, queue }
        }

        fn step<Tracker: tracker::Tracker<Expander::Node>>(
            &self,
            storage: &mut Self::Storage,
            options: &Expander::Options,
            tracker: &mut Tracker
        ) -> Result<Expander> {

        }
    }
}
