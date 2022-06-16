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
use crate::expander::{Goal, Expander, Initializable, Expandable, Solvable, CostOf, InitErrorOf, ExpansionErrorOf, SolveErrorOf};
use crate::trace::Trace;
use std::sync::Arc;
use std::fmt::Debug;

#[derive(Debug)]
// pub enum InitError<S, E: Solvable + Initializable<S>, A: Algorithm<E>> {
pub enum InitError<A: Debug, E: Debug> {
    Algorithm(A),
    Expander(E),
}

#[derive(Debug)]
pub enum StepError<A, E, S> {
    Algorithm(A),
    Expansion(E),
    Solve(S),
}

#[derive(Debug, Clone)]
pub enum Status<E: Solvable> {
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

pub trait Algorithm<E: Solvable>: Sized {
    type Memory: Memory;

    type InitError: std::fmt::Debug;
    type StepError: std::fmt::Debug;

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
    ) -> Result<Status<E>, StepError<Self::StepError, ExpansionErrorOf<E, G>, SolveErrorOf<E>>>
    where E: Expandable<G>;
}
