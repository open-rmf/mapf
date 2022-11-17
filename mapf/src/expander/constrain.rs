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

use crate::{error::Error, expander::traits::*};
use std::{marker::PhantomData, sync::Arc};
use thiserror::Error as ThisError;

pub trait AimlessConstraint<N> {
    type ConstraintError: Error;
    fn constrain(&self, node: Arc<N>) -> Result<Option<Arc<N>>, Self::ConstraintError>;
}

pub trait TargetedConstraint<N, G> {
    type ConstraintError: Error;
    fn constrain(&self, node: Arc<N>, goal: &G) -> Result<Option<Arc<N>>, Self::ConstraintError>;
}

pub trait ReversibleConstraint {
    type ReversalError: Error;
    type Reverse;
    fn reverse(&self) -> Result<Self::Reverse, Self::ReversalError>;
}

pub struct Constrain<E: Expander, C> {
    base: E,
    constrain_with: C,
}

#[derive(ThisError, Debug)]
pub enum ConstrainErr<E: Error, C: Error> {
    #[error("An error occurred in the expander:\n{0}")]
    Base(E),
    #[error("An error occurred in the constraint:\n{0}")]
    Constraint(C),
}

impl<E: Expander, C> Expander for Constrain<E, C> {
    type Node = E::Node;
}

impl<E: Targeted<G>, C: TargetedConstraint<E::Node, G>, G: Goal<E::Node>> Targeted<G>
    for Constrain<E, C>
{
    type TargetedError = ConstrainErr<E::TargetedError, C::ConstraintError>;
    type TargetedExpansion<'a> = impl Iterator<Item=Result<Arc<Self::Node>, Self::TargetedError>> + 'a where Self: 'a, G: 'a ;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: &'a G,
    ) -> Self::TargetedExpansion<'a> {
        self.base
            .expand(parent, goal)
            .into_iter()
            .map(|r| r.map_err(ConstrainErr::Base))
            .map(move |r| {
                r.and_then(|n| {
                    self.constrain_with
                        .constrain(n, goal)
                        .map_err(ConstrainErr::Constraint)
                })
            })
            .filter_map(|r| r.transpose())
    }
}

impl<E, C, S, G> InitTargeted<S, G> for Constrain<E, C>
where
    E: InitTargeted<S, G>,
    C: TargetedConstraint<E::Node, G>,
    G: Goal<E::Node>,
{
    type InitTargetedError = ConstrainErr<E::InitTargetedError, C::ConstraintError>;
    type InitialTargetedNodes<'a> = impl Iterator<Item=Result<Arc<Self::Node>, Self::InitTargetedError>> + 'a where Self: 'a, S: 'a, G: 'a;

    fn start<'a>(&'a self, start: &'a S, goal: &'a G) -> Self::InitialTargetedNodes<'a> {
        self.base
            .start(start, goal)
            .into_iter()
            .map(|r| r.map_err(ConstrainErr::Base))
            .map(move |r| {
                r.and_then(|n| {
                    self.constrain_with
                        .constrain(n, goal)
                        .map_err(ConstrainErr::Constraint)
                })
            })
            .filter_map(|r| r.transpose())
    }
}

impl<E: Solvable, C> Solvable for Constrain<E, C> {
    type Solution = E::Solution;
    type SolveError = E::SolveError;
    fn make_solution(
        &self,
        solution_node: &Arc<Self::Node>,
    ) -> Result<Self::Solution, Self::SolveError> {
        self.base.make_solution(solution_node)
    }
}

impl<E: Closable, C> Closable for Constrain<E, C> {
    type ClosedSet = E::ClosedSet;
}

impl<E: Reversible, C: ReversibleConstraint> Reversible for Constrain<E, C> {
    type ReversalError = ConstrainErr<E::ReversalError, C::ReversalError>;
    type Reverse = Constrain<E::Reverse, C::Reverse>;

    fn reverse(&self) -> Result<ReverseOf<Self>, Self::ReversalError> {
        let base = self.base.reverse().map_err(ConstrainErr::Base)?;
        let constrain_with = self
            .constrain_with
            .reverse()
            .map_err(ConstrainErr::Constraint)?;
        Ok(Constrain {
            base,
            constrain_with,
        })
    }
}

impl<E: BidirSolvable, C: ReversibleConstraint> BidirSolvable for Constrain<E, C> {
    type BidirSolveError = E::BidirSolveError;

    fn make_bidirectional_solution(
        &self,
        forward_solution_node: &Arc<Self::Node>,
        reverse_solution_node: &Arc<ReverseNodeOf<E>>,
    ) -> Result<Self::Solution, Self::BidirSolveError> {
        self.base
            .make_bidirectional_solution(forward_solution_node, reverse_solution_node)
    }
}

// TODO(MXG): We should be able to get rid of these repetitive alternative
// constraint closure types when [impl specialization](https://rust-lang.github.io/rfcs/1210-impl-specialization.html)
// is available
pub struct AimlessConstraintClosure<F: Fn(Arc<N>) -> Result<Option<Arc<N>>, Err>, N, Err> {
    closure: F,
    _ignore: PhantomData<(N, Err)>,
}

impl<F, N, Err> AimlessConstraintClosure<F, N, Err>
where
    F: Fn(Arc<N>) -> Result<Option<Arc<N>>, Err>,
{
    pub fn new(closure: F) -> Self {
        Self {
            closure,
            _ignore: Default::default(),
        }
    }
}

impl<F, N, Err> AimlessConstraint<N> for AimlessConstraintClosure<F, N, Err>
where
    Err: Error,
    F: Fn(Arc<N>) -> Result<Option<Arc<N>>, Err>,
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node)
    }
}

pub struct TargetedConstraintClosure<F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>, N, G, Err> {
    closure: F,
    _ignore: PhantomData<(N, G, Err)>,
}

impl<F, N, G, Err> TargetedConstraintClosure<F, N, G, Err>
where
    F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>,
{
    pub fn new(closure: F) -> Self {
        Self {
            closure,
            _ignore: Default::default(),
        }
    }
}

impl<F, N, G, Err> TargetedConstraint<N, G> for TargetedConstraintClosure<F, N, G, Err>
where
    G: Goal<N>,
    Err: Error,
    F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>,
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>, goal: &G) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node, goal)
    }
}

pub struct ConstraintClosure<F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>, N, G, Err> {
    closure: F,
    _ignore: PhantomData<(N, G, Err)>,
}

impl<F, N, G, Err> ConstraintClosure<F, N, G, Err>
where
    F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>,
{
    pub fn new(closure: F) -> Self {
        Self {
            closure,
            _ignore: Default::default(),
        }
    }
}

impl<F, N, G, Err> AimlessConstraint<N> for ConstraintClosure<F, N, G, Err>
where
    G: Goal<N>,
    Err: Error,
    F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>,
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node, None)
    }
}

impl<F, N, G, Err> TargetedConstraint<N, G> for ConstraintClosure<F, N, G, Err>
where
    G: Goal<N>,
    Err: Error,
    F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>,
{
    type ConstraintError = Err;
    fn constrain(&self, node: Arc<N>, goal: &G) -> Result<Option<Arc<N>>, Self::ConstraintError> {
        (self.closure)(node, Some(goal))
    }
}

pub trait Constrainable: Sized {
    type Base: Expander;
    fn constrain<C>(self, constrain_with: C) -> Constrain<Self::Base, C>;

    fn constrain_fn<F, N, G, Err>(
        self,
        closure: F,
    ) -> Constrain<Self::Base, ConstraintClosure<F, N, G, Err>>
    where
        F: Fn(Arc<N>, Option<&G>) -> Result<Option<Arc<N>>, Err>,
    {
        self.constrain(ConstraintClosure::new(closure))
    }

    fn aimless_constrain_fn<F, N, Err>(
        self,
        closure: F,
    ) -> Constrain<Self::Base, AimlessConstraintClosure<F, N, Err>>
    where
        F: Fn(Arc<N>) -> Result<Option<Arc<N>>, Err>,
    {
        self.constrain(AimlessConstraintClosure::new(closure))
    }

    fn targeted_constrain_fn<F, N, G, Err>(
        self,
        closure: F,
    ) -> Constrain<Self::Base, TargetedConstraintClosure<F, N, G, Err>>
    where
        F: Fn(Arc<N>, &G) -> Result<Option<Arc<N>>, Err>,
    {
        self.constrain(TargetedConstraintClosure::new(closure))
    }
}

impl<E: Expander> Constrainable for E {
    type Base = E;
    fn constrain<C>(self, constrain_with: C) -> Constrain<Self::Base, C> {
        Constrain {
            base: self,
            constrain_with,
        }
    }
}
