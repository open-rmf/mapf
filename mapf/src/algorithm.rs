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
    node,
    expander::{Goal, Expander, Initializable, Expandable, Solvable, CostOf, InitErrorOf, ExpansionErrorOf, SolveErrorOf},
    trace::Trace,
    error::Error,
};
use std::{
    sync::Arc,
    fmt::Debug,
};
use thiserror::Error as ThisError;

#[derive(ThisError, Debug)]
pub enum InitError<A: Error, E: Error> {
    #[error("An error occurred while initializing the algorithm:\n{0}")]
    Algorithm(A),
    #[error("An error occurred while constructing the initial nodes:\n{0}")]
    Expander(E),
}

#[derive(ThisError, Debug)]
pub enum StepError<A: Error, E: Error, S: Error> {
    #[error("An error occurred in the algorithm:\n{0}")]
    Algorithm(A),
    #[error("An error occurred during expansion:\n{0}")]
    Expansion(E),
    #[error("An error occurred while constructing the solution:\n{0}")]
    Solve(S),
}

#[derive(Debug, Clone)]
pub enum Status<Solution> {
    Incomplete,
    Impossible,
    Solved(Solution),
}

pub trait Memory {
    fn node_count(&self) -> usize;
}

/// A trait to attach to the Memory of an Algorithm that sorts its nodes
/// according to a cost/weight.
pub trait WeightSorted<E: Expander<Node: node::Weighted>>: Memory {
    fn top_cost_estimate(&self) -> Option<CostOf<E>>;
}

pub trait Algorithm<E: Solvable>: Sized {
    type Memory: Memory;

    type InitError: Error;
    type StepError: Error;

    fn initialize<S, G: Goal<E::Node>, T: Trace<E::Node>>(
        &self,
        expander: Arc<E>,
        start: &S,
        goal: &G,
        trace: &mut T,
    ) -> Result<Self::Memory, InitError<Self::InitError, InitErrorOf<E, S, G>>>
    where E: Initializable<S, G>;

    fn step<G: Goal<E::Node>, T: Trace<E::Node>>(
        &self,
        memory: &mut Self::Memory,
        goal: &G,
        tracker: &mut T,
    ) -> Result<Status<E::Solution>, StepError<Self::StepError, ExpansionErrorOf<E, G>, SolveErrorOf<E>>>
    where E: Expandable<G>;
}
