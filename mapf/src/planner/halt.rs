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

use crate::algorithm::{Measure, MinimumCostBound};
use std::{ops::Fn, sync::Arc};

/// A trait to define conditions in which a search should be halted. The
/// settings can be changed in between calls to  Search::solve().
pub trait Halt<Mem>: Clone {
    /// Check whether the current search should be interrupted.
    fn halt(&mut self, memory: &Mem) -> bool;
}

/// If an empty tuple is given for the options then we treat that as an
/// indication that we should let the solver continue without halting for any
/// reason.
impl<Mem> Halt<Mem> for () {
    fn halt(&mut self, _: &Mem) -> bool {
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
#[derive(Clone)]
pub struct Interruptible(pub Option<Interrupter>);

impl Interruptible {
    pub fn new<F: Fn() -> Interruption + 'static>(f: F) -> Self {
        Self(Some(Arc::new(f)))
    }

    pub fn none() -> Self {
        Self(None)
    }
}

impl<Mem> Halt<Mem> for Interruptible {
    fn halt(&mut self, _: &Mem) -> bool {
        if let Some(interrupter) = &self.0 {
            return Interruption::Stop == interrupter();
        }

        return false;
    }
}

/// This option sets a maximum number of steps that can be taken before the
/// planner is told to halt.
#[derive(Debug, Clone)]
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

impl<Mem> Halt<Mem> for StepLimit {
    fn halt(&mut self, _: &Mem) -> bool {
        self.steps += 1;
        if let Some(limit) = self.limit {
            return self.steps > limit;
        }

        false
    }
}

/// The maximum size that the Memory's Measure can reach before the
/// solve attempt quits. For example, this might put a limit on how large
/// the search queue can get.
#[derive(Default, Clone)]
pub struct MeasureLimit(pub Option<usize>);

impl<Mem> Halt<Mem> for MeasureLimit
where
    Mem: Measure,
{
    fn halt(&mut self, memory: &Mem) -> bool {
        if let Some(limit) = self.0 {
            return dbg!(dbg!(memory.size()) > dbg!(limit));
        }

        false
    }
}

/// If the lower cost bound of the search exceeds this cost limit then the
/// search will halt.
#[derive(Default, Clone)]
pub struct CostLimit<C>(pub Option<C>);

impl<Mem: MinimumCostBound> Halt<Mem> for CostLimit<Mem::Cost>
where
    Mem::Cost: Clone + PartialOrd,
{
    fn halt(&mut self, memory: &Mem) -> bool {
        if let Some(limit) = &self.0 {
            match memory.minimum_cost_bound() {
                Some(bound) => return bound > *limit,
                None => return true,
            }
        }

        false
    }
}

use paste;
/// Tuples of Halt<M> will also implement Halt<M> with a boolean-and
/// combination of each tuple element's result. We currently limit the tuple
/// size to 8 elements, but that limit can be overcome using nested tuples.
macro_rules! and_tuple_halt {
    ( $( $name:ident )+ ) => {
        paste::item! {
            impl<Mem, $($name: Halt<Mem>),+> Halt<Mem> for ($($name,)+) {
                fn halt(
                    &mut self,
                    memory: &Mem,
                ) -> bool {
                    let ($([<$name:lower>],)+) = self;
                    false $(|| [<$name:lower>].halt(memory))+
                }
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

    struct FakeMem;

    impl Measure for FakeMem {
        fn size(&self) -> usize {
            0
        }
    }

    #[test]
    fn test_tuple_options() {
        let mut halting = (
            Interruptible::new(|| Interruption::Continue),
            StepLimit::new(Some(10)),
        );

        assert!(!halting.halt(&FakeMem));

        let mut halting = (
            Interruptible::new(|| Interruption::Continue),
            StepLimit::new(Some(5)),
            MeasureLimit(Some(100)),
        );

        for _ in 0..5 {
            assert!(!halting.halt(&FakeMem));
        }
        assert!(halting.halt(&FakeMem));
    }
}
