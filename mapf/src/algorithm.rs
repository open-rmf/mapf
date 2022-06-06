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

use crate::node;
use crate::expander::{Expander, CostOf, InitErrorOf, ExpansionErrorOf, SolveErrorOf};
use crate::trace::Trace;
use std::sync::Arc;
use derivative::Derivative;

#[derive(Derivative)]
#[derivative(Debug)]
pub enum InitError<E: Expander, A: Algorithm<E>> {
    Algorithm(A::InitError),
    Expander(InitErrorOf<E>),
}

#[derive(Derivative)]
#[derivative(Debug)]
pub enum StepError<E: Expander, A: Algorithm<E>> {
    Algorithm(A::StepError),
    Expansion(ExpansionErrorOf<E>),
    Solve(SolveErrorOf<E>),
}

#[derive(Debug, Clone)]
pub enum Status<E: Expander> {
    Incomplete,
    Impossible,
    Solved(E::Solution),
}

pub trait Memory {
    fn node_count(&self) -> usize;
}

pub trait WeightSorted<E: Expander<Node: node::Weighted>> {
    fn top_cost_estimate(&self) -> Option<CostOf<E>>;
}

pub trait Algorithm<E: Expander>: Sized {
    type Memory: Memory;

    type InitError: std::fmt::Debug;
    type StepError: std::fmt::Debug;

    fn initialize<T: Trace<E::Node>>(
        &self,
        expander: Arc<E>,
        start: &E::Start,
        goal: E::Goal,
        trace: &mut T,
    ) -> Result<Self::Memory, InitError<E, Self>>;

    fn step<T: Trace<E::Node>>(
        &self,
        storage: &mut Self::Memory,
        tracker: &mut T,
    ) -> Result<Status<E>, StepError<E, Self>>;
}
