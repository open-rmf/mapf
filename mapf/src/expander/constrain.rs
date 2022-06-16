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
    fn constrain(&self, node: Arc<N>, goal: &G) -> Result<Option<Arc<N>>, Self::ConstraintError>;
}

pub trait ReversibleConstraint<N: node::Reversible, S: Goal<N::Reverse>> {
    type ReversalError: std::fmt::Debug;
    type Reverse: Constraint<N::Reverse, S>;
    fn reverse(&self) -> Result<Arc<Self::Reverse>, Self::ReversalError>;
}

pub struct Constrain<E: Expander, C> {
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

impl<E: Expander, C> Expander for Constrain<E, C> {
    type Node = E::Node;
}

impl<E: Expandable<G>, C: Constraint<E::Node, G>, G: Goal<E::Node>> Expandable<G> for Constrain<E, C> {
    type ExpansionError = ConstrainErr<E::ExpansionError, C::ConstraintError>;
    type Expansion<'a> where Self: 'a, G: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::ExpansionError>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: &'a G,
    ) -> Self::Expansion<'a> {
        self.base.expand(parent, goal).into_iter().map(|r| r.map_err(ConstrainErr::Base))
        .map(move |r| r.and_then(|n| self.constrain_with.constrain(n, goal).map_err(ConstrainErr::Constraint)))
        .filter_map(|r| r.transpose())
    }
}

impl<E, C, S, G> Initializable<S, G> for Constrain<E, C>
where
    E: Initializable<S, G>,
    C: Constraint<E::Node, G>,
    G: Goal<E::Node>,
{
    type InitError = ConstrainErr<E::InitError, C::ConstraintError>;
    type InitialNodes<'a> where Self: 'a, S: 'a, G: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::InitError>> + 'a;

    fn start<'a>(
        &'a self,
        start: &'a S,
        goal: &'a G,
    ) -> Self::InitialNodes<'a> {
        self.base.start(start, goal).into_iter().map(|r| r.map_err(ConstrainErr::Base))
        .map(move |r| r.and_then(|n| self.constrain_with.constrain(n, goal).map_err(ConstrainErr::Constraint)))
        .filter_map(|r| r.transpose())
    }
}

impl<E: Solvable, C> Solvable for Constrain<E, C> {
    type Solution = E::Solution;
    type SolveError = E::SolveError;
    fn make_solution(&self, solution_node: &Arc<Self::Node>) -> Result<Self::Solution, Self::SolveError> {
        self.base.make_solution(solution_node)
    }
}

impl<E: Closable, C> Closable for Constrain<E, C> {
    type ClosedSet = E::ClosedSet;
}

impl<E, C, S, G> Reversible<S, G> for Constrain<E, C>
where
    E: Reversible<S, G>,
    C: Constraint<E::Node, G> + ReversibleConstraint<E::Node, S>,
    S: Goal<ReverseNodeOf<E, S, G>>,
{
    type ReversalError = ConstrainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Constrain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<Arc<ReverseOf<Self, S, G>>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ConstrainErr::Base)?;
        let constrain_with = self.constrain_with.reverse().map_err(ConstrainErr::Constraint)?;
        Ok(Arc::new(Constrain{base, constrain_with}))
    }
}

impl<E, C, S, G> BidirSolvable<S, G> for Constrain<E, C>
where
    Self: Expandable<G, Node=E::Node> + Reversible<S, G>,
    E: BidirSolvable<S, G>,
    C: Constraint<E::Node, G>,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    type BidirSolveError = E::BidirSolveError;

    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<ReverseNodeOf<E, S, G>>,
    ) -> Result<Self::Solution, Self::BidirSolveError> {
        self.base.make_bidirectional_solution(forward_solution_node, reverse_solution_node)
    }
}

pub struct ConstraintClosure<N, G: Goal<N>, Err: Debug, F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>> {
    closure: F,
    _ignore: std::marker::PhantomData<(N, G, Err)>,
}

impl<N, G, Err, F> ConstraintClosure<N, G, Err, F>
where
    G: Goal<N>,
    Err: Debug,
    F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>
{
    pub fn new(closure: F) -> Self {
        Self{closure, _ignore: Default::default()}
    }
}

impl<N, G, Err, F> Constraint<N, G> for ConstraintClosure<N, G, Err, F>
where
    G: Goal<N>,
    Err: Debug,
    F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>, goal: &G) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node, goal)
    }
}

pub trait Constrainable {
    type Base: Expander;
    fn constrain<C>(
        self,
        constrain_with: Arc<C>,
    ) -> Arc<Constrain<Self::Base, C>>;

    fn constrain_fn<G, Err, F>(
        self,
        closure: F,
    ) -> Arc<Constrain<Self::Base, ConstraintClosure<NodeOf<Self::Base>, G, Err, F>>>
    where
        Self: Sized,
        Err: Debug,
        G: Goal<NodeOf<Self::Base>>,
        F: Fn(Arc<NodeOf<Self::Base>>, &G) -> Result<Option<Arc<NodeOf<Self::Base>>>, Err>,
    {
        self.constrain(Arc::new(ConstraintClosure::new(closure)))
    }
}

impl<E: Expander> Constrainable for Arc<E> {
    type Base = E;
    fn constrain<C>(
        self,
        constrain_with: Arc<C>,
    ) -> Arc<Constrain<Self::Base, C>> {
        Arc::new(Constrain{base: self, constrain_with})
    }
}
