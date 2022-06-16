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

pub struct Chain<E: Expander, C: Expander<Node=E::Node>> {
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

impl<E: Expander, C: Expander<Node=E::Node>> Expander for Chain<E, C> {
    type Node = E::Node;
}

impl<E: Expandable<G>, C: Expandable<G, Node=E::Node>, G: Goal<E::Node>> Expandable<G> for Chain<E, C> {
    type ExpansionError = ChainErr<E::ExpansionError, C::ExpansionError>;
    type Expansion<'a> where Self: 'a, G: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::ExpansionError>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: &'a G,
    ) -> Self::Expansion<'a> {
        self.base.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Base))
        .chain(self.chain_with.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Next)))
    }
}

impl<E, C, S, G> Initializable<S, G> for Chain<E, C>
where
    E: Initializable<S, G> + Expandable<G>,
    C: Expandable<G, Node=E::Node>,
    G: Goal<E::Node>,
{
    type InitError = E::InitError;
    type InitialNodes<'a> where Self: 'a, S: 'a, G: 'a = E::InitialNodes<'a>;

    fn start<'a>(
        &'a self,
        start: &'a S,
        goal: &'a G,
    ) -> Self::InitialNodes<'a> {
        self.base.start(start, goal)
    }
}

impl<E: Solvable, C: Expander<Node=E::Node>> Solvable for Chain<E, C>
{
    type Solution = E::Solution;
    type SolveError = E::SolveError;
    fn make_solution(&self, solution_node: &Arc<Self::Node>) -> Result<Self::Solution, Self::SolveError> {
        self.base.make_solution(solution_node)
    }
}

impl<E: Closable, C: Expander<Node=E::Node>> Closable for Chain<E, C> {
    type ClosedSet = E::ClosedSet;
}

impl<E, C, S, G> Reversible<S, G> for Chain<E, C>
where
    E: Reversible<S, G>,
    C: Expander<Node=E::Node> + Reversible<S, G>,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    type ReversalError = ChainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Chain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<Arc<ReverseOf<Self, S, G>>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ChainErr::Base)?;
        let chain_with = self.chain_with.reverse().map_err(ChainErr::Next)?;
        Ok(Arc::new(Chain{base, chain_with}))
    }
}

impl<E: BidirSolvable<S, G>, C, S, G> BidirSolvable<S, G> for Chain<E, C>
where
    C: Expander<Node=E::Node> + Reversible<S, G>,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    type BidirSolveError = E::BidirSolveError;

    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<ReverseNodeOf<Self, S, G>>
    ) -> Result<Self::Solution, Self::BidirSolveError> {
        self.base.make_bidirectional_solution(forward_solution_node, reverse_solution_node)
    }
}

pub trait Chainable {
    type Base: Expander;
    fn chain<C: Expander<Node=NodeOf<Self::Base>>>(
        self,
        chain_with: Arc<C>
    ) -> Arc<Chain<Self::Base, C>>;

    fn chain_fn<G, Err, Exp, F>(
        self,
        closure: F
    ) -> Arc<Chain<Self::Base, Closure<NodeOf<Self::Base>, G, Err, Exp, F>>>
    where
        Self: Sized,
        G: Goal<NodeOf<Self::Base>>,
        Err: Debug,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, Err>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, &G) -> Exp,
    {
        self.chain(Arc::new(Closure::new(closure)))
    }

    fn chain_fn_no_err<G, Exp, F>(
        self,
        closure: F
    ) -> Arc<Chain<Self::Base, Closure<NodeOf<Self::Base>, G, (), Exp, F>>>
    where
        Self: Sized,
        G: Goal<NodeOf<Self::Base>>,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, ()>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, &G) -> Exp,
    {
        self.chain(Arc::new(Closure::new(closure)))
    }
}

impl<E: Expander> Chainable for Arc<E> {
    type Base = E;
    fn chain<C: Expander<Node=E::Node>>(
        self,
        chain_with: Arc<C>
    ) -> Arc<Chain<Self::Base, C>> {
        Arc::new(Chain{base: self, chain_with})
    }
}
