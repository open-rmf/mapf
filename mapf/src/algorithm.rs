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

use crate::error::Error;
use std::{fmt::Debug};
use thiserror::Error as ThisError;

#[derive(ThisError, Debug)]
pub enum InitError<A: Error, D: Error> {
    #[error("An error occurred while initializing the algorithm:\n{0}")]
    Algorithm(A),
    #[error("An error occurred while initializing the domain:\n{0}")]
    Domain(D),
}

#[derive(ThisError, Debug)]
pub enum StepError<A: Error, D: Error> {
    #[error("An error occurred in the algorithm:\n{0}")]
    Algorithm(A),
    #[error("An error occurred in the domain:\n{0}")]
    Domain(D),
}

#[derive(Debug, Clone)]
pub enum Status<Solution> {
    Incomplete,
    Impossible,
    Solved(Solution),
}

/// The `Algorithm` trait defines the basic structure that an algorithm needs
/// to satisfy in order for a Planner to operate on it.
pub trait Algorithm: Sized {
    /// The `Memory` type tracks the progress of each search.
    type Memory;

    /// The `Solution` type is what the Algorithm will return once it has found
    /// a valid solution.
    type Solution;

    /// A `StepError` will be returned when an issue is encountered during a
    /// step of the algorithm.
    type StepError: Error;

    /// Take a step in the search algorithm. The same memory instance will be
    /// passed in with each iteration.
    fn step(
        &self,
        memory: &mut Self::Memory,
    ) -> Result<Status<Self::Solution>, Self::StepError>;
}

/// The `Coherent` trait determines when the user input is coherent (usable) for
/// the algorithm. An algorithm may have multiple (Start, Goal) combinations
/// that it can solve for, so this trait can be defined for any combination that
/// the Algorithm is able to support.
pub trait Coherent<Start, Goal>: Algorithm {
    type InitError: Error;

    fn initialize(
        &self,
        start: Start,
        goal: Goal,
    ) -> Result<Self::Memory, Self::InitError>;
}

/// The `Measure` trait can be implemented by `Algorithm::Memory` types to
/// provide an indication of how large their current level of effort or memory
/// footprint is. This may be used to halt search efforts that have grown
/// excessively large.
pub trait Measure {
    /// How "big" is the current memory footprint or level of effort. The exact
    /// meaning of this value may vary between algorithms.
    fn size(&self) -> usize;
}

/// The `MinimumCostBound` trait can be implemented by `Algorithm::Memory` types
/// to report a minimum bound for the cost of any possible solution to a problem
/// if a solution exists. This can be used to halt search efforts if the cost
/// exceeds a certain bound.
pub trait MinimumCostBound {
    type Cost;
    fn minimum_cost_bound(&self) -> Self::Cost;
}
