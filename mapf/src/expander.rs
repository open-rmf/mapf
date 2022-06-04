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
use super::node::{self, Node, Cost};

pub trait Goal<N: Node> {
    fn is_satisfied(&self, node: &N) -> bool;
}

pub trait Solution<C: Cost>: Clone {
    fn cost(&self) -> C;
}

pub trait Initializable<Start, N: Node, G: Goal<N>> {
    /// The type of error that can occur during node initialization.
    type InitError: std::fmt::Debug;

    /// An initial set of nodes, produced from a Start object
    type InitialNodes<'a>: IntoIterator<Item=Result<Arc<N>, Self::InitError>, IntoIter: 'a> where Self: 'a, Start: 'a, G: 'a;

    /// Generate an initial set of nodes based on the given start conditions
    fn start<'a>(
        &'a self,
        start: &'a Start,
        goal: Option<&'a G>,
    ) -> Self::InitialNodes<'a>;
}

pub trait Expandable<N: Node, G: Goal<N>> {
    /// The type of error that can occur during expansion.
    type ExpansionError: std::fmt::Debug;

    /// An expansion that can be generated by this Expander
    type Expansion<'a>: Iterator<Item=Result<Arc<N>, Self::ExpansionError>>
    where
        Self: 'a,
        G: 'a,
        N: 'a;

    /// Expand the given node
    fn expand<'a>(
        &'a self,
        parent: &'a Arc<N>,
        goal: Option<&'a G>,
    ) -> Self::Expansion<'a>;
}

pub trait Solvable<N: Node> {
    /// The representation of solutions that can be produced by this Expander
    type Solution: Solution<<N as Node>::Cost>;

    /// The type of error that can occur while constructing a solution
    type SolveError: std::fmt::Debug;

    /// Make a Solution for the given solution node
    fn make_solution(&self, solution_node: &Arc<N>) -> Result<Self::Solution, Self::SolveError>;
}

pub trait Expander
where
    Self:
      Initializable<Self::Start, Self::Node, Self::Goal>
    + Expandable<Self::Node, Self::Goal>
    + Solvable<Self::Node>
{
    type Start;
    type Node: Node;
    type Goal: Goal<Self::Node>;
}

/// The Reversible trait can be implemented by Expanders that support expanding
/// in reverse from a goal. Bidirectional algorithms can take advantage of this
/// trait.
pub trait Reversible: Expander where Self::Node: node::Reversible {
    type Reverse: Expander<Node=<Self::Node as node::Reversible>::Reverse, Start=Self::Goal>;
    type ReversalError: std::fmt::Debug;
    type BidirSolveError: std::fmt::Debug;

    /// Create a reverse expander for the algorithm to use.
    fn reverse(&self) -> Result<Arc<Self::Reverse>, Self::ReversalError>;

    /// Make a solution from a (Forward, Reverse) expansion node pair.
    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<<Self::Reverse as Expander>::Node>
    ) -> Result<Self::Solution, Self::BidirSolveError>;
}

pub type NodeOf<E> = <E as Expander>::Node;
pub type StartOf<E> = <E as Expander>::Start;
pub type GoalOf<E> = <E as Expander>::Goal;
pub type CostOf<E> = <NodeOf<E> as node::Node>::Cost;
pub type InitErrorOf<E> = <E as Initializable<StartOf<E>, NodeOf<E>, GoalOf<E>>>::InitError;
pub type ExpansionErrorOf<E> = <E as Expandable<NodeOf<E>, GoalOf<E>>>::ExpansionError;
pub type SolveErrorOf<E> = <E as Solvable<NodeOf<E>>>::SolveError;
pub type SolutionOf<E> = <E as Solvable<NodeOf<E>>>::Solution;
pub type ReverseOf<E> = <E as Reversible>::Reverse;
pub type ReversalErrorOf<E> = <E as Reversible>::ReversalError;
pub type BidirSolveErrorOf<E> = <E as Reversible>::BidirSolveError;
