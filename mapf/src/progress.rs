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

use std::{
    sync::Arc,
    ops::Fn,
    cell::RefCell,
};
use anyhow;

use crate::{
    node::Weighted,
    expander::{Goal, Expander, Targeted, Solvable, CostOf, ExpansionErrorOf, SolveErrorOf},
    algorithm::{Algorithm, WeightSorted, Status, Memory, StepError},
    trace::Trace,
};

/// Progress manages the progress of a planning effort.
pub struct Progress<E: Solvable, A: Algorithm<E>, O: Options<E, A>, G, T: Trace<E::Node>> {
    /// Storage container for the progress of the search algorithm
    memory: A::Memory,

    /// The object which determines the search pattern
    algorithm: Arc<A>,

    /// The options that moderate the progress of the solving
    options: O,

    /// The goal that the algorithm must try to reach
    goal: G,

    /// The object which tracks planning progress
    trace: T,
}

impl<E, A, O, G, T> Progress<E, A, O, G, T>
where
    E: Targeted<G> + Solvable,
    A: Algorithm<E>,
    O: Options<E, A>,
    G: Goal<E::Node>,
    T: Trace<E::Node>,
{
    pub fn new(
        memory: A::Memory,
        algorithm: Arc<A>,
        options: O,
        goal: G,
        trace: T,
    ) -> Self {
        Self{memory, algorithm, options, goal, trace}
    }

    pub fn into_abstract(self) -> Abstract<E::Solution>
    where E: 'static, A: 'static, O: 'static, G: 'static, T: 'static {
        Abstract{implementation: Box::new(RefCell::new(self))}
    }

    /// Tell the planner to attempt to solve the problem. This will run the
    /// step() function until a solution is found, the progress gets
    /// interrupted, or the algorithm determines that the problem is impossible
    /// to solve.
    pub fn solve(&mut self) -> Result<Status<E::Solution>, StepError<A::StepError, ExpansionErrorOf<E, G>, SolveErrorOf<E>>> {
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

    pub fn step(&mut self) -> Result<Status<E::Solution>, StepError<A::StepError, ExpansionErrorOf<E, G>, SolveErrorOf<E>>> {
        return self.algorithm.step(&mut self.memory, &self.goal, &mut self.trace);
    }

    pub fn memory(&self) -> &A::Memory {
        &self.memory
    }
}

impl<E: Solvable, A: Algorithm<E>, G, T: Trace<E::Node>> Progress<E, A, BasicOptions, G, T> {
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

pub trait Options<E: Solvable, A: Algorithm<E>>: Clone {
    /// Check whether the current progress should be interrupted according to
    /// the current set of options.
    fn need_to_interrupt<G, T: Trace<E::Node>>(&self, progress: &Progress<E, A, Self, G, T>) -> bool;
}

pub trait WithOptions<O> {
    /// Set the progression options for the planning.
    fn with_options(self, options: O) -> Self;

    fn options(&self) -> &O;

    fn options_mut(&mut self) -> &mut O;
}

impl<E, A, G, O, T> WithOptions<O> for Progress<E, A, O, G, T>
where
    E: Targeted<G> + Solvable,
    A: Algorithm<E>,
    G: Goal<E::Node>,
    O: Options<E, A>,
    T: Trace<E::Node>
{
    fn with_options(mut self, options: O) -> Self {
        self.options = options;
        self
    }

    fn options(&self) -> &O {
        &self.options
    }

    fn options_mut(&mut self) -> &mut O {
        &mut self.options
    }
}

/// Tell the planner to interrupt its attempt to solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Interruption {
    Continue,
    Stop
}

/// Signature for an object that can interrupt the planner.
/// Use Progress::with_interrupter to set this.
pub type Interrupter = Arc<dyn Fn() -> Interruption>;

/// Options for how progression should be performed. These can be changed in
/// between calls to Progress::solve().
#[derive(Default, Clone)]
pub struct BasicOptions {
    /// A function that can decide to interrupt the planning
    pub interrupter: Option<Interrupter>,

    /// The maximum size that the search queue can reach before the
    /// solve attemptp quits.
    pub search_queue_limit: Option<usize>,
}

impl BasicOptions {
    pub fn internal_need_to_interrupt<M: Memory>(&self, memory: &M) -> bool {
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

impl<E, A> Options<E, A> for BasicOptions
where
    E: Solvable,
    A: Algorithm<E>,
{
    fn need_to_interrupt<G, T: Trace<E::Node>>(&self, progress: &Progress<E, A, Self, G, T>) -> bool {
        self.internal_need_to_interrupt(&progress.memory)
    }
}

pub trait WithBasicOptions {
    /// Set the interrupter that can stop the progress. If the interrupter
    /// returns Stop, then the planning will be interrupted and left incomplete.
    fn with_interrupter(self, interrupter: Option<Interrupter>) -> Self;

    /// Get a reference to the interrupter that can stop the progress.
    fn interrupter(&self) -> &Option<Interrupter>;

    /// Get a mutable reference to the interrupter that can stop the progress.
    fn interrupter_mut(&mut self) -> &mut Option<Interrupter>;

    // TODO(MXG): Should we have a take_interrupter()?

    /// Set the maximum size of the search queue. If the search queue exceeds
    /// this size, then the planning will be interrupted and left incomplete.
    fn with_search_queue_limit(self, search_queue_limit: Option<usize>) -> Self;

    /// Get the search queue limit
    fn search_queue_limit(&self) -> Option<usize>;

    /// Get a mutable reference to the search queue limit.
    fn search_queue_limit_mut(&mut self) -> &mut Option<usize>;
}

impl WithBasicOptions for BasicOptions {
    fn with_interrupter(mut self, interrupter: Option<Interrupter>) -> Self {
        self.interrupter = interrupter;
        self
    }

    fn interrupter(&self) -> &Option<Interrupter> {
        &self.interrupter
    }

    fn interrupter_mut(&mut self) -> &mut Option<Interrupter> {
        &mut self.interrupter
    }

    fn with_search_queue_limit(mut self, search_queue_limit: Option<usize>) -> Self {
        self.search_queue_limit = search_queue_limit;
        self
    }

    fn search_queue_limit(&self) -> Option<usize> {
        self.search_queue_limit
    }

    fn search_queue_limit_mut(&mut self) -> &mut Option<usize> {
        &mut self.search_queue_limit
    }
}

/// Allow these functions to be called directly from the Progress object when
/// the Options of the Progress implement WithBasicOptions
impl<E, A, G, O, T> WithBasicOptions for Progress<E, A, O, G, T>
where
    E: Targeted<G> + Solvable,
    A: Algorithm<E>,
    G: Goal<E::Node>,
    O: Options<E, A> + WithBasicOptions,
    T: Trace<E::Node>,
{
    fn with_interrupter(mut self, interrupter: Option<Interrupter>) -> Self {
        self.options = self.options.with_interrupter(interrupter);
        self
    }

    fn interrupter(&self) -> &Option<Interrupter> {
        self.options.interrupter()
    }

    fn interrupter_mut(&mut self) -> &mut Option<Interrupter> {
        self.options.interrupter_mut()
    }

    fn with_search_queue_limit(mut self, search_queue_limit: Option<usize>) -> Self {
        self.options = self.options.with_search_queue_limit(search_queue_limit);
        self
    }

    fn search_queue_limit(&self) -> Option<usize> {
        self.options.search_queue_limit()
    }

    fn search_queue_limit_mut(&mut self) -> &mut Option<usize> {
        self.options.search_queue_limit_mut()
    }
}

pub struct WeightedOptions<E: Expander<Node: Weighted>> {

    /// The maximum total cost estimate that the top node can reach before
    /// the solve attempt quits.
    pub max_cost_estimate: Option<CostOf<E>>,

    /// The basic options that can be set
    pub basic: BasicOptions,
}

impl<E: Solvable<Node: Weighted>> WeightedOptions<E> {
    pub fn internal_need_to_interrupt<M: WeightSorted<E>>(&self, memory: &M) -> bool {
        if let Some(max_cost_estimate) = &self.max_cost_estimate {
            if let Some(top_cost_estimate) = memory.top_cost_estimate() {
                if top_cost_estimate > *max_cost_estimate {
                    return true;
                }
            }
        }

        return self.basic.internal_need_to_interrupt(memory);
    }
}

impl<E, A> Options<E, A> for WeightedOptions<E>
where
    E: Solvable<Node: Weighted>,
    A: Algorithm<E, Memory: WeightSorted<E>>,
{
    fn need_to_interrupt<G, T: Trace<E::Node>>(&self, progress: &Progress<E, A, Self, G, T>) -> bool {
        self.internal_need_to_interrupt(&progress.memory)
    }
}

impl<E: Expander<Node: Weighted>> WithBasicOptions for WeightedOptions<E> {
    fn with_interrupter(mut self, interrupter: Option<Interrupter>) -> Self {
        self.basic.interrupter = interrupter;
        self
    }

    fn interrupter(&self) -> &Option<Interrupter> {
        &self.basic.interrupter
    }

    fn interrupter_mut(&mut self) -> &mut Option<Interrupter> {
        &mut self.basic.interrupter
    }

    fn with_search_queue_limit(mut self, search_queue_limit: Option<usize>) -> Self {
        self.basic.search_queue_limit = search_queue_limit;
        self
    }

    fn search_queue_limit(&self) -> Option<usize> {
        self.basic.search_queue_limit
    }

    fn search_queue_limit_mut(&mut self) -> &mut Option<usize> {
        &mut self.basic.search_queue_limit
    }
}

pub trait WithWeightedOptions<E: Expander<Node: Weighted>> {
    /// Set the max cost estimate that the planner will use to interrupt the
    /// planning progress. If the best possible cost estimate of the planner
    /// ever exceeds this value, the planning will be interrupted and left
    /// incomplete.
    fn with_max_cost_estimate(self, max_cost_estimate: Option<CostOf<E>>) -> Self;

    fn max_cost_estimate(&self) -> Option<CostOf<E>>;

    fn max_cost_estimate_mut(&mut self) -> &mut Option<CostOf<E>>;
}

impl<E: Expander<Node: Weighted>> WithWeightedOptions<E> for WeightedOptions<E> {
    fn with_max_cost_estimate(mut self, max_cost_estimate: Option<CostOf<E>>) -> Self {
        self.max_cost_estimate = max_cost_estimate;
        self
    }

    fn max_cost_estimate(&self) -> Option<CostOf<E>> {
        self.max_cost_estimate
    }

    fn max_cost_estimate_mut(&mut self) -> &mut Option<CostOf<E>> {
        &mut self.max_cost_estimate
    }
}

impl<E: Expander<Node: Weighted>> Default for WeightedOptions<E> {
    fn default() -> Self {
        return Self {
            max_cost_estimate: None,
            basic: Default::default(),
        }
    }
}

impl<E: Expander<Node: Weighted>> Clone for WeightedOptions<E> {
    fn clone(&self) -> Self {
        Self { max_cost_estimate: self.max_cost_estimate, basic: self.basic.clone() }
    }
}

impl<E, A, G, O, T> WithWeightedOptions<E> for Progress<E, A, O, G, T>
where
    E: Targeted<G> + Solvable<Node: Weighted>,
    A: Algorithm<E>,
    G: Goal<E::Node>,
    O: Options<E, A> + WithWeightedOptions<E>,
    T: Trace<E::Node>,
{
    fn with_max_cost_estimate(mut self, max_cost_estimate: Option<CostOf<E>>) -> Self {
        self.options = self.options.with_max_cost_estimate(max_cost_estimate);
        self
    }

    fn max_cost_estimate(&self) -> Option<CostOf<E>> {
        self.options.max_cost_estimate()
    }

    fn max_cost_estimate_mut(&mut self) -> &mut Option<CostOf<E>> {
        self.options.max_cost_estimate_mut()
    }
}

pub trait Interface<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>>;

    fn step(&mut self) -> anyhow::Result<Status<Solution>>;
}

impl<E, A, O, G, T> Interface<E::Solution> for Progress<E, A, O, G, T>
where
    E: Targeted<G> + Solvable,
    A: Algorithm<E>,
    O: Options<E, A>,
    G: Goal<E::Node>,
    T: Trace<E::Node>,
{
    fn solve(&mut self) -> anyhow::Result<Status<E::Solution>> {
        Progress::solve(self).map_err(anyhow::Error::new)
    }

    fn step(&mut self) -> anyhow::Result<Status<E::Solution>> {
        Progress::step(self).map_err(anyhow::Error::new)
    }
}

pub trait InterfaceWithOptions<Solution, Options>: Interface<Solution> + WithOptions<Options> { }

/// Everything that implements Interface<Solution> and WithOptions<Options>
/// automatically implements InterfaceWithOptions<Solution, Options>
impl<Solution, Options, T: Interface<Solution> + WithOptions<Options>>
InterfaceWithOptions<Solution, Options> for T { }

pub struct Abstract<Solution> {
    implementation: Box<RefCell<dyn Interface<Solution>>>,
}

impl<Solution> Interface<Solution> for Abstract<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().solve()
    }

    fn step(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().step()
    }
}
