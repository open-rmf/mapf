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

use crate::expander::{
    traits::*, Closure,
};
use std::sync::Arc;
use std::fmt::Debug;

pub struct Chain<E: Expandable, C: Expandable<Node=E::Node, Goal=E::Goal>> {
    base: Arc<E>,
    chain_with: Arc<C>,
}

pub enum ChainErr<E: Debug, C: Debug> {
    Base(E),
    Next(C),
}

impl<E: Debug, C: Debug> Debug for ChainErr<E, C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChainErr::Base(e) => {
                f.debug_tuple("ChainErr::Base").field(e).finish()
            },
            ChainErr::Next(e) => {
                f.debug_tuple("ChainErr::Next").field(e).finish()
            }
        }
    }
}

impl<E: Expandable, C: Expandable<Node=E::Node, Goal=E::Goal>> Expander for Chain<E, C> {
    type Node = E::Node;
    type Goal = E::Goal;
}

impl<E: Expandable, C: Expandable<Node=E::Node, Goal=E::Goal>> Expandable for Chain<E, C> {
    type ExpansionError = ChainErr<E::ExpansionError, C::ExpansionError>;
    type Expansion<'a> where Self: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::ExpansionError>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: Option<&'a Self::Goal>,
    ) -> Self::Expansion<'a> {
        self.base.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Base))
        .chain(self.chain_with.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Next)))
    }
}

impl<E, C, S> Initializable<S> for Chain<E, C>
where
    E: Expandable + Initializable<S>,
    C: Expandable<Node=E::Node, Goal=E::Goal>,
{
    type InitError = E::InitError;
    type InitialNodes<'a> where Self: 'a, S: 'a = E::InitialNodes<'a>;

    fn start<'a>(
        &'a self,
        start: &'a S,
        goal: Option<&'a Self::Goal>,
    ) -> Self::InitialNodes<'a> {
        self.base.start(start, goal)
    }
}

impl<E, C> Solvable for Chain<E, C>
where
    E: Solvable,
    C: Expandable<Node=E::Node, Goal=E::Goal>,
{
    type Solution = E::Solution;
    type SolveError = E::SolveError;
    fn make_solution(&self, solution_node: &Arc<Self::Node>) -> Result<Self::Solution, Self::SolveError> {
        self.base.make_solution(solution_node)
    }
}

impl<E, C> Closable for Chain<E, C>
where
    E: Closable + Expandable,
    C: Expandable<Node=E::Node, Goal=E::Goal>,
{
    type ClosedSet = E::ClosedSet;
}

impl<E, C> Reversible for Chain<E, C>
where
    E: Expandable + Reversible,
    C: Expandable<Node=E::Node, Goal=E::Goal> + Reversible<Reverse: Expander<Node=ReverseNodeOf<E>, Goal=ReverseGoalOf<E>>>,
{
    type ReversalError = ChainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Chain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<Arc<<Self as Reversible>::Reverse>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ChainErr::Base)?;
        let chain_with = self.chain_with.reverse().map_err(ChainErr::Next)?;
        Ok(Arc::new(Chain{base, chain_with}))
    }
}

impl<E, C> BidirSolvable for Chain<E, C>
where
    E: BidirSolvable,
    C: Expandable<Node=E::Node, Goal=E::Goal> + Reversible<Reverse: Expander<Node=ReverseNodeOf<E>, Goal=ReverseGoalOf<E>>>,
{
    type BidirSolveError = E::BidirSolveError;

    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<ReverseNodeOf<Self>>
    ) -> Result<Self::Solution, Self::BidirSolveError> {
        self.base.make_bidirectional_solution(forward_solution_node, reverse_solution_node)
    }
}

pub trait Chainable {
    type Base: Expandable;
    fn chain<C: Expandable<Node=NodeOf<Self::Base>, Goal=GoalOf<Self::Base>>>(
        self,
        chain_with: Arc<C>
    ) -> Chain<Self::Base, C>;

    fn chain_fn<Err, Exp, F>(
        self,
        closure: F
    ) -> Chain<Self::Base, Closure<NodeOf<Self::Base>, GoalOf<Self::Base>, Err, Exp, F>>
    where
        Self: Sized,
        Err: Debug,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, Err>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, Option<&GoalOf<Self::Base>>) -> Exp,
    {
        self.chain(Arc::new(Closure::new(closure)))
    }

    fn chain_fn_no_err<Exp, F>(
        self,
        closure: F
    ) -> Chain<Self::Base, Closure<NodeOf<Self::Base>, GoalOf<Self::Base>, (), Exp, F>>
    where
        Self: Sized,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, ()>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, Option<&GoalOf<Self::Base>>) -> Exp,
    {
        self.chain(Arc::new(Closure::new(closure)))
    }
}

impl<E: Expandable> Chainable for Arc<E> {
    type Base = E;
    fn chain<C: Expandable<Node=NodeOf<Self::Base>, Goal=GoalOf<Self::Base>>>(
        self,
        chain_with: Arc<C>
    ) -> Chain<Self::Base, C> {
        Chain{base: self, chain_with}
    }
}
