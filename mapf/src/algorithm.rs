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

use super::expander::{Expander, CostOf, InitErrorOf, ExpansionErrorOf, SolveErrorOf};
use super::tracker;
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

pub trait Storage<E: Expander> {
    fn node_count(&self) -> usize;
    fn top_cost_estimate(&self) -> Option<CostOf<E>>;
}

pub trait Algorithm<E: Expander>: Sized {
    type Storage: Storage<E>;

    type InitError: std::fmt::Debug;
    type StepError: std::fmt::Debug;

    fn initialize<Tracker: tracker::Tracker<E::Node>>(
        &self,
        expander: Arc<E>,
        start: &E::Start,
        goal: E::Goal,
        tracker: &mut Tracker,
    ) -> Result<Self::Storage, InitError<E, Self>>;

    fn step<Tracker: tracker::Tracker<E::Node>>(
        &self,
        storage: &mut Self::Storage,
        tracker: &mut Tracker,
    ) -> Result<Status<E>, StepError<E, Self>>;
}
