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

pub mod a_star;
pub use a_star::{AStar, AStarConnect};

pub mod dijkstra;
pub use dijkstra::{BackwardDijkstra, Dijkstra};

pub mod tree;

pub mod path;
pub use path::Path;

// TODO(@mxgrey): Consider whether this should be in the planner::search module
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SearchStatus<Solution> {
    Incomplete,
    Impossible,
    Solved(Solution),
}

impl<S> SearchStatus<S> {
    pub fn incomplete(&self) -> bool {
        matches!(self, SearchStatus::Incomplete)
    }

    pub fn impossible(&self) -> bool {
        matches!(self, SearchStatus::Impossible)
    }

    pub fn solved(&self) -> bool {
        matches!(self, SearchStatus::Solved(_))
    }

    pub fn solution(self) -> Option<S> {
        match self {
            Self::Solved(solution) => Some(solution),
            _ => None,
        }
    }

    /// If the status contains a solution, apply a function to that solution.
    pub fn map<U, F: FnOnce(S) -> U>(self, op: F) -> SearchStatus<U> {
        match self {
            SearchStatus::Solved(solution) => SearchStatus::Solved(op(solution)),
            SearchStatus::Incomplete => SearchStatus::Incomplete,
            SearchStatus::Impossible => SearchStatus::Impossible,
        }
    }

    pub fn and_then<U, E, F: FnOnce(S) -> Result<U, E>>(self, op: F) -> Result<SearchStatus<U>, E> {
        match self {
            SearchStatus::Solved(solution) => Ok(SearchStatus::Solved(op(solution)?)),
            SearchStatus::Incomplete => Ok(SearchStatus::Incomplete),
            SearchStatus::Impossible => Ok(SearchStatus::Impossible),
        }
    }
}

impl<S> From<S> for SearchStatus<S> {
    fn from(value: S) -> Self {
        SearchStatus::Solved(value)
    }
}

pub trait Algorithm {
    /// The `Memory` type tracks the progress of each search.
    type Memory;
}

/// The `Coherent` trait determines when the user input is coherent (usable) for
/// the algorithm. An algorithm may have multiple (Start, Goal) combinations
/// that it can solve for, so this trait can be defined for any combination that
/// the Algorithm is able to support.
pub trait Coherent<Start, Goal>: Algorithm {
    type InitError;

    fn initialize(&self, start: Start, goal: &Goal) -> Result<Self::Memory, Self::InitError>;
}

/// The `Algorithm` trait defines the basic structure that an algorithm needs
/// to satisfy in order for a Planner to operate on it.
pub trait Solvable<Goal>: Algorithm + Sized {
    /// The `Solution` type is what the Algorithm will return once it has found
    /// a valid solution.
    type Solution;

    /// A `StepError` will be returned when an issue is encountered during a
    /// step of the algorithm.
    type StepError;

    /// Take a step in the search algorithm. The same memory instance will be
    /// passed in with each iteration.
    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError>;
}

// Implement the Algorithm traits for Arc<Algo> so that planners can always
// have a way to cheaply copy the algorith/domain.
use std::sync::Arc;

impl<Algo: Algorithm> Algorithm for Arc<Algo> {
    type Memory = Algo::Memory;
}

impl<Start, Goal, Algo: Coherent<Start, Goal>> Coherent<Start, Goal> for Arc<Algo> {
    type InitError = Algo::InitError;

    fn initialize(&self, start: Start, goal: &Goal) -> Result<Self::Memory, Self::InitError> {
        self.as_ref().initialize(start, goal)
    }
}

impl<Goal, Algo: Solvable<Goal>> Solvable<Goal> for Arc<Algo> {
    type Solution = Algo::Solution;
    type StepError = Algo::StepError;
    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError> {
        self.as_ref().step(memory, goal)
    }
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
///
/// Returning None implies that the minimum cost is unbounded (i.e. it could be
/// infinite or there might not be any solution at all). The [`CostLimit`]
/// halting type will cause the Search to halt if the minimum cost bound is None
/// while the cost limit is Some.
pub trait MinimumCostBound {
    type Cost;
    fn minimum_cost_bound(&self) -> Option<Self::Cost>;
}
