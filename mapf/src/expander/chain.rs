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
    expander::{traits::*, Closure},
    error::Error,
};
use std::{
    sync::Arc,
};
use thiserror::Error as ThisError;

pub struct Chain<E: Expander, C: Expander<Node=E::Node>> {
    base: E,
    chain_with: C,
}

#[derive(ThisError, Debug)]
pub enum ChainErr<E: Error, C: Error> {
    #[error("An error occurred in the base expander:\n{0}")]
    Base(E),
    #[error("An error occurred in the chained expander:\n{0}")]
    Next(C),
}

impl<E: Expander, C: Expander<Node=E::Node>> Expander for Chain<E, C> {
    type Node = E::Node;
}

impl<E: Targeted<G>, C: Targeted<G, Node=E::Node>, G: Goal<E::Node>> Targeted<G> for Chain<E, C> {
    type TargetedError = ChainErr<E::TargetedError, C::TargetedError>;
    type TargetedExpansion<'a> where Self: 'a, G: 'a = impl Iterator<Item=Result<Arc<Self::Node>, Self::TargetedError>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: &'a G,
    ) -> Self::TargetedExpansion<'a> {
        self.base.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Base))
        .chain(self.chain_with.expand(parent, goal).into_iter().map(|r| r.map_err(ChainErr::Next)))
    }
}

impl<E, C, S, G> InitTargeted<S, G> for Chain<E, C>
where
    E: InitTargeted<S, G> + Targeted<G>,
    C: Targeted<G, Node=E::Node>,
    G: Goal<E::Node>,
{
    type InitTargetedError = E::InitTargetedError;
    type InitialTargetedNodes<'a> where Self: 'a, S: 'a, G: 'a = E::InitialTargetedNodes<'a>;

    fn start<'a>(
        &'a self,
        start: &'a S,
        goal: &'a G,
    ) -> Self::InitialTargetedNodes<'a> {
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

impl<E, C> Reversible for Chain<E, C>
where
    E: Reversible,
    C: Expander<Node=E::Node> + Reversible,
{
    type ReversalError = ChainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Chain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<ReverseOf<Self>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ChainErr::Base)?;
        let chain_with = self.chain_with.reverse().map_err(ChainErr::Next)?;
        Ok(Chain{base, chain_with})
    }
}

impl<E: BidirSolvable, C> BidirSolvable for Chain<E, C>
where
    C: Expander<Node=E::Node> + Reversible,
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
    type Base: Expander;
    fn chain<C: Expander<Node=NodeOf<Self::Base>>>(
        self,
        chain_with: C
    ) -> Chain<Self::Base, C>;

    fn chain_fn<G, Err, Exp, F>(
        self,
        closure: F
    ) -> Chain<Self::Base, Closure<NodeOf<Self::Base>, G, Err, Exp, F>>
    where
        Self: Sized,
        G: Goal<NodeOf<Self::Base>>,
        Err: Error,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, Err>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, &G) -> Exp,
    {
        self.chain(Closure::new(closure))
    }

    fn chain_fn_no_err<G, Exp, F>(
        self,
        closure: F
    ) -> Chain<Self::Base, Closure<NodeOf<Self::Base>, G, (), Exp, F>>
    where
        Self: Sized,
        G: Goal<NodeOf<Self::Base>>,
        Exp: IntoIterator<Item=Result<Arc<NodeOf<Self::Base>>, ()>>,
        F: Fn(&Arc<NodeOf<Self::Base>>, &G) -> Exp,
    {
        self.chain(Closure::new(closure))
    }
}

impl<E: Expander> Chainable for E {
    type Base = E;
    fn chain<C: Expander<Node=E::Node>>(
        self,
        chain_with: C,
    ) -> Chain<Self, C> {
        Chain{base: self, chain_with}
    }
}
