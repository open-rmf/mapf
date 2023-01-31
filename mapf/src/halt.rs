/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

use crate::algorithm::{Algorithm, Measure, MinimumCostBound};
use std::{ops::Fn, sync::Arc};

/// A trait to define Options for how progression should be performed. The
/// settings for the Options can be changed in between calls to
/// Progress::solve().
pub trait Halt<A: Algorithm>: Clone {
    /// Check whether the current progress should be interrupted according to
    /// the current set of options.
    fn halt(
        &mut self,
        memory: &A::Memory,
    ) -> bool;
}

/// If an empty tuple is given for the options then we treat that as an
/// indication that we should let the solver continue without halting for any
/// reason.
impl<A: Algorithm> Halt<A> for () {
    fn halt(
        &mut self,
        _: &A::Memory,
    ) -> bool {
        false
    }
}

/// Tell the planner to interrupt its attempt to solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Interruption {
    Continue,
    Stop,
}

/// Signature for an object that can interrupt the planner.
/// Use Progress::with_interrupter to set this.
pub type Interrupter = Arc<dyn Fn() -> Interruption>;

/// This option allows the user to specify a callback to indicate whether the
/// progress should continue.
#[derive(Debug, Clone)]
pub struct Interruptible(pub Option<Interrupter>);

impl Interruptible {
    pub fn new<F: Fn() -> Interruption>(f: F) -> Self {
        Self(Some(f))
    }

    pub fn none() -> Self {
        Self(None)
    }
}

impl<A: Algorithm> Halt<A> for Interruptible {
    fn halt(
        &mut self,
        progress: &A::Memory,
    ) -> bool {
        if let Some(interrupter) = &self.0 {
            return Interruption::Stop == interrupter();
        }

        return false;
    }
}

/// This option sets a maximum number of steps that can be taken before the
/// planner is told to halt.
pub struct StepLimit {
    steps: usize,
    pub limit: Option<usize>,
}

impl StepLimit {
    pub fn new(limit: Option<usize>) -> Self {
        Self { steps: 0, limit }
    }

    pub fn reset(&mut self) {
        self.steps = 0;
    }
}

impl<A> Halt<A> for StepLimit {
    fn halt(
        &mut self,
        progress: &<A as Algorithm>::Memory,
    ) -> bool {
        self.steps += 1;
        if let Some(limit) = self.limit {
            return self.steps > self.limit;
        }

        false
    }
}

/// The maximum size that the Memory's Measure can reach before the
/// solve attempt quits. For example, this might put a limit on how large
/// the search queue can get.
#[derive(Default, Clone)]
pub struct MeasureLimit(pub Option<usize>);

impl<A: Algorithm> Halt<A> for MeasureLimit
where
    A::Memory: Measure,
{
    fn halt(
        &mut self,
        memory: &A::Memory,
    ) -> bool {
        if let Some(limit) = self.0 {
            return memory.size() > limit;
        }

        false
    }
}

/// If the lower cost bound of the search exceeds this cost limit then the
/// search will halt.
#[derive(Default, Clone)]
pub struct CostLimit<C>(pub Option<C>);

impl<A: Algorithm> Halt<A> for CostLimit<<A::Memory as MinimumCostBound>::Cost>
where
    A::Memory: MinimumCostBound,
{
    fn halt(
        &mut self,
        memory: &A::Memory,
    ) -> bool {
        if let Some(limit) = self.0 {
            return memory.minimum_cost_bound() > limit;
        }

        false
    }
}

/// Tuples of Halt<A> will also implement Halt<A> with a boolean-and
/// combination of each tuple element's result. We currently limit the tuple
/// size to 8 elements, but that limit can be overcome using nested tuples.
macro_rules! and_tuple_halt {
    ( $( $name:ident )+ ) => {
        impl<Alg: Algorithm, $($name: Halt<Alg>),+> Halt<Alg> for ($($name,)+) {
            fn halt(
                &mut self,
                memory: &Alg::Memory,
            ) -> bool {
                let ($($name,)+) = self;
                false $(|| $name.halt(memory))+
            }
        }
    };
}

and_tuple_halt! { A }
and_tuple_halt! { A B }
and_tuple_halt! { A B C }
and_tuple_halt! { A B C D }
and_tuple_halt! { A B C D E }
and_tuple_halt! { A B C D E F }
and_tuple_halt! { A B C D E F G }
and_tuple_halt! { A B C D E F G H }

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Algorithm,
        algorithm::Status,
        error::NoError,
    };

    struct FakeMem;

    struct FakeAlgo;
    impl Algorithm for FakeAlgo {
        type Memory = FakeMem;
        type Solution = ();
        type StepError = NoError;

        fn step(
            &self,
            memory: &mut Self::Memory,
        ) -> Result<Status<Self::Solution>, Self::StepError> {
            Ok(Status::Incomplete)
        }
    }

    #[test]
    fn test_tuple_options() {
        let options = (
            Interruptible::new(|| Interruption::Continue),
            StepLimit::new(Some(10)),
        );

        assert!(options.halt(&FakeMem));
    }
}
