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
    node, expander::traits::*
};
use std::sync::Arc;
use std::fmt::Debug;

pub trait Constraint<N, G> {
    type ConstraintError: std::fmt::Debug;
    fn constrain(&self, node: Arc<N>, goal: Option<&G>) -> Result<Option<Arc<N>>, Self::ConstraintError>;
}

pub trait ReversibleConstraint<N: node::Reversible, ReverseGoal: Goal<N::Reverse>> {
    type ReversalError: std::fmt::Debug;
    type Reverse: Constraint<N::Reverse, ReverseGoal>;
    fn reverse(&self) -> Result<Arc<Self::Reverse>, Self::ReversalError>;
}

pub struct Constrain<E: Expander, C: Constraint<E::Node, E::Goal>> {
    base: Arc<E>,
    constrain_with: Arc<C>,
}

pub enum ConstrainErr<E: Debug, C: Debug> {
    Base(E),
    Constraint(C),
}

impl<E: Debug, C: Debug> Debug for ConstrainErr<E, C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConstrainErr::Base(e) => {
                f.debug_tuple("ConstrainErr::Base").field(e).finish()
            },
            ConstrainErr::Constraint(e) => {
                f.debug_tuple("ConstrainErr::Constraint").field(e).finish()
            }
        }
    }
}

impl<E: Expander, C: Constraint<E::Node, E::Goal>> Expander for Constrain<E, C> {
    type Node = E::Node;
    type Goal = E::Goal;
}

impl<E: Expandable, C: Constraint<E::Node, E::Goal>> Expandable for Constrain<E, C> {
    type ExpansionError = ConstrainErr<E::ExpansionError, C::ConstraintError>;
    type Expansion<'a> where Self: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::ExpansionError>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: Option<&'a Self::Goal>,
    ) -> Self::Expansion<'a> {
        self.base.expand(parent, goal).into_iter().map(|r| r.map_err(ConstrainErr::Base))
        .map(move |r| r.and_then(|n| self.constrain_with.constrain(n, goal).map_err(ConstrainErr::Constraint)))
        .filter_map(|r| r.transpose())
    }
}

impl<E, C, S> Initializable<S> for Constrain<E, C>
where
    E: Initializable<S>,
    C: Constraint<E::Node, E::Goal>,
{
    type InitError = ConstrainErr<E::InitError, C::ConstraintError>;
    type InitialNodes<'a> where Self: 'a, S: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::InitError>> + 'a;

    fn start<'a>(
        &'a self,
        start: &'a S,
        goal: Option<&'a Self::Goal>,
    ) -> Self::InitialNodes<'a> {
        self.base.start(start, goal).into_iter().map(|r| r.map_err(ConstrainErr::Base))
        .map(move |r| r.and_then(|n| self.constrain_with.constrain(n, goal).map_err(ConstrainErr::Constraint)))
        .filter_map(|r| r.transpose())
    }
}

impl<E, C> Solvable for Constrain<E, C>
where
    E: Solvable,
    C: Constraint<E::Node, E::Goal>
{
    type Solution = E::Solution;
    type SolveError = E::SolveError;
    fn make_solution(&self, solution_node: &Arc<Self::Node>) -> Result<Self::Solution, Self::SolveError> {
        self.base.make_solution(solution_node)
    }
}

impl<E, C> Closable for Constrain<E, C>
where
    E: Closable,
    C: Constraint<E::Node, E::Goal>,
{
    type ClosedSet = E::ClosedSet;
}

impl<E, C> Reversible for Constrain<E, C>
where
    E: Reversible,
    C: Constraint<E::Node, E::Goal> + ReversibleConstraint<E::Node, ReverseGoalOf<E>>,
{
    type ReversalError = ConstrainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Constrain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<Arc<<Self as Reversible>::Reverse>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ConstrainErr::Base)?;
        let constrain_with = self.constrain_with.reverse().map_err(ConstrainErr::Constraint)?;
        Ok(Arc::new(Constrain{base, constrain_with}))
    }
}

impl<E, C> BidirSolvable for Constrain<E, C>
where
    Self: Expandable<Node=E::Node> + Reversible,
    E: BidirSolvable,
    C: Constraint<E::Node, E::Goal>,
{
    type BidirSolveError = E::BidirSolveError;

    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<ReverseNodeOf<E>>,
    ) -> Result<Self::Solution, Self::BidirSolveError> {
        self.base.make_bidirectional_solution(forward_solution_node, reverse_solution_node)
    }
}

pub struct ConstraintClosure<N, G: Goal<N>, Err: Debug, F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>> {
    closure: F,
    _ignore: std::marker::PhantomData<(N, G, Err)>,
}

impl<N, G, Err, F> ConstraintClosure<N, G, Err, F>
where
    G: Goal<N>,
    Err: Debug,
    F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>
{
    pub fn new(closure: F) -> Self {
        Self{closure, _ignore: Default::default()}
    }
}

impl<N, G, Err, F> Constraint<N, G> for ConstraintClosure<N, G, Err, F>
where
    G: Goal<N>,
    Err: Debug,
    F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>, goal: Option<&G>) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node, goal)
    }
}

pub trait Constrainable {
    type Base: Expander;
    fn constrain<C: Constraint<NodeOf<Self::Base>, GoalOf<Self::Base>>>(
        self,
        constrain_with: Arc<C>,
    ) -> Constrain<Self::Base, C>;

    fn constrain_fn<Err, F>(
        self,
        closure: F,
    ) -> Constrain<Self::Base, ConstraintClosure<NodeOf<Self::Base>, GoalOf<Self::Base>, Err, F>>
    where
        Self: Sized,
        Err: Debug,
        F: Fn(Arc<NodeOf<Self::Base>>, Option<&GoalOf<Self::Base>>) -> Result<Option<Arc<NodeOf<Self::Base>>>, Err>,
    {
        self.constrain(Arc::new(ConstraintClosure::new(closure)))
    }
}

impl<E: Expander> Constrainable for Arc<E> {
    type Base = E;
    fn constrain<C: Constraint<NodeOf<Self::Base>, GoalOf<Self::Base>>>(
        self,
        constrain_with: Arc<C>,
    ) -> Constrain<Self::Base, C> {
        Constrain{base: self, constrain_with}
    }
}
