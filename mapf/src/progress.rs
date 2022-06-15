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

use std::sync::Arc;
use std::ops::Fn;
use std::boxed::Box;

use crate::node::Weighted;
use crate::expander::{Expander, Solvable, CostOf, ExpansionErrorOf, SolveErrorOf};
use crate::algorithm::{Algorithm, WeightSorted, Status, Memory, StepError};
use crate::trace::Trace;

pub trait Progress {
    type Options;
}

pub trait Factory<E: Solvable, A: Algorithm<E>>: Default {
    type Progress<T: Trace<E::Node>>: Progress;
    fn new<T: Trace<E::Node>>(
        memory: A::Memory,
        algorithm: Arc<A>,
        trace: T,
    ) -> Self::Progress<T>;
}

pub trait Options<P: Progress> {
    /// Check whether the current progress should be interrupted according to
    /// the current set of options.
    fn need_to_interrupt(&self, progress: &P) -> bool;
}

/// Tell the planner to interrupt its attempt to solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Interruption {
    Continue,
    Stop
}

/// Signature for an object that can interrupt the planner.
/// Use Progress::with_interrupter to set this.
pub type Interrupter = Box<dyn Fn() -> Interruption>;

/// Options for how progression should be performed. These can be changed in
/// between calls to Progress::solve().
#[derive(Default)]
pub struct BasicOptions {
    /// A function that can decide to interrupt the planning
    pub interrupter: Option<Interrupter>,

    /// The maximum size that the search queue can reach before the
    /// solve attemptp quits.
    pub search_queue_limit: Option<usize>,
}

impl BasicOptions {
    fn internal_need_to_interrupt<M: Memory>(&self, memory: &M) -> bool {
        if let Some(interrupter) = &self.interrupter {
            if Interruption::Stop == interrupter() {
                return true;
            }
        }

        if let Some(search_queue_limit) = &self.search_queue_limit {
            if memory.node_count() > *search_queue_limit {
                return true;
            }
        }

        return false;
    }
}

impl<E, A, T> Options<BuiltinProgress<E, A, Self, T>> for BasicOptions
where
    E: Solvable,
    A: Algorithm<E>,
    T: Trace<E::Node>,
{
    fn need_to_interrupt(&self, progress: &BuiltinProgress<E, A, Self, T>) -> bool {
        self.internal_need_to_interrupt(&progress.memory)
    }
}

pub struct WeightedSearchOptions<E: Expander<Node: Weighted> + Solvable> {

    /// The maximum total cost estimate that the top node can reach before
    /// the solve attempt quits.
    pub max_cost_estimate: Option<CostOf<E>>,

    /// The basic options that can be set
    pub basic: BasicOptions,
}

impl<E, A, T> Options<BuiltinProgress<E, A, Self, T>> for WeightedSearchOptions<E>
where
    E: Expander<Node: Weighted> + Solvable,
    A: Algorithm<E>,
    A::Memory: WeightSorted<E>,
    T: Trace<E::Node>,
{
    fn need_to_interrupt(&self, progress: &BuiltinProgress<E, A, Self, T>) -> bool {
        if let Some(max_cost_estimate) = &self.max_cost_estimate {
            if let Some(top_cost_estimate) = progress.memory.top_cost_estimate() {
                if top_cost_estimate > *max_cost_estimate {
                    return true;
                }
            }
        }

        return self.basic.internal_need_to_interrupt(&progress.memory);
    }
}

impl<E: Expander<Node: Weighted> + Solvable> Default for WeightedSearchOptions<E> {
    fn default() -> Self {
        return Self {
            max_cost_estimate: None,
            basic: Default::default(),
        }
    }
}

/// Progress manages the progress of a planning effort.
pub struct BuiltinProgress<E: Solvable, A: Algorithm<E>, O: Options<Self>, T: Trace<E::Node>> {
    /// Storage container for the progress of the search algorithm
    memory: A::Memory,

    /// The object which determines the search pattern
    algorithm: Arc<A>,

    /// The options that moderate the progress of the solving
    options: O,

    /// The object which tracks planning progress
    trace: T
}

pub type BasicProgress<E, A, T> = BuiltinProgress<E, A, BasicOptions, T>;

impl<E: Solvable, A: Algorithm<E>, O: Options<Self>, T: Trace<E::Node>> BuiltinProgress<E, A, O, T> {
    /// Set the progression options for the planning.
    pub fn with_options(mut self, options: O) -> Self {
        self.options = options;
        self
    }

    pub fn options_mut(&mut self) -> &mut O {
        return &mut self.options;
    }

    pub fn options(&self) -> &O {
        return &self.options;
    }

    /// Tell the planner to attempt to solve the problem. This will run the
    /// step() function until a solution is found, the progress gets
    /// interrupted, or the algorithm determines that the problem is impossible
    /// to solve.
    pub fn solve(&mut self) -> Result<Status<E>, StepError<A::StepError, ExpansionErrorOf<E>, SolveErrorOf<E>>> {
        loop {
            if self.options.need_to_interrupt(self) {
                return Ok(Status::Incomplete);
            }

            let result = self.step()?;
            if let Status::Incomplete = result {
                continue;
            }

            return Ok(result);
        }
    }

    pub fn step(&mut self) -> Result<Status<E>, StepError<A::StepError, ExpansionErrorOf<E>, SolveErrorOf<E>>> {
        return self.algorithm.step(&mut self.memory, &mut self.trace);
    }

    pub fn memory(&self) -> &A::Memory {
        &self.memory
    }
}

impl<E: Solvable, A: Algorithm<E>, O: Options<Self>, T: Trace<E::Node>> Progress for BuiltinProgress<E, A, O, T> {
    type Options = O;
}

impl<E: Solvable, A: Algorithm<E>, T: Trace<E::Node>> BuiltinProgress<E, A, BasicOptions, T> {
    /// Set the interrupter that can stop the progress. If the interrupter
    /// returns Stop, then the planning will be interrupted and left incomplete.
    pub fn with_interrupter(mut self, interrupter: Option<Interrupter>) -> Self {
        self.options.interrupter = interrupter;
        self
    }

    /// Set the maximum size of the search queue. If the search queue exceeds
    /// this size, then the planning will be interrupted and left incomplete.
    pub fn with_search_queue_limit(mut self, search_queue_limit: Option<usize>) -> Self {
        self.options.search_queue_limit = search_queue_limit;
        self
    }
}

impl<E: Expander<Node: Weighted> + Solvable, A: Algorithm<E, Memory: WeightSorted<E>>, T: Trace<E::Node>>
BuiltinProgress<E, A, WeightedSearchOptions<E>, T> {
    /// Set the max cost estimate that the planner will use to interrupt the
    /// planning progress. If the best possible cost estimate of the planner
    /// ever exceeds this value, the planning will be interrupted and left
    /// incomplete.
    pub fn with_max_cost_estimate(mut self, max_cost_estimate: Option<CostOf<E>>) -> Self {
        self.options.max_cost_estimate = max_cost_estimate;
        self
    }

    /// Set the interrupter that can stop the progress. If the interrupter
    /// returns Stop, then the planning will be interrupted and left incomplete.
    pub fn with_interrupter(mut self, interrupter: Option<Interrupter>) -> Self {
        self.options.basic.interrupter = interrupter;
        self
    }

    /// Set the maximum size of the search queue. If the search queue exceeds
    /// this size, then the planning will be interrupted and left incomplete.
    pub fn with_search_queue_limit(mut self, search_queue_limit: Option<usize>) -> Self {
        self.options.basic.search_queue_limit = search_queue_limit;
        self
    }
}

#[derive(Default)]
pub struct BasicFactory;

/// Progress factory implementation for algorithms that do not sort nodes by cost.
impl<E: Solvable, A: Algorithm<E>> Factory<E, A> for BasicFactory {
    type Progress<T: Trace<E::Node>> = BuiltinProgress<E, A, BasicOptions, T>;
    fn new<T: Trace<E::Node>>(
        memory: A::Memory,
        algorithm: Arc<A>,
        trace: T,
    ) -> Self::Progress<T> {
        BuiltinProgress{memory, algorithm, options: Default::default(), trace}
    }
}

#[derive(Default)]
pub struct WeightedFactory;

/// Progress factory implementation for algorithms that do sort by cost.
impl<E: Expander<Node: Weighted> + Solvable, A: Algorithm<E, Memory: WeightSorted<E>>> Factory<E, A> for WeightedFactory {
    type Progress<T: Trace<E::Node>> = BuiltinProgress<E, A, WeightedSearchOptions<E>, T>;
    fn new<T: Trace<E::Node>>(
        memory: <A as Algorithm<E>>::Memory,
        algorithm: Arc<A>,
        trace: T,
    ) -> Self::Progress<T> {
        BuiltinProgress{memory, algorithm, options: Default::default(), trace}
    }
}
