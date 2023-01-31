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

use anyhow;
use std::{cell::RefCell, sync::Arc};

use crate::{
    algorithm::{Algorithm, Status},
    halt::Halt,
};

/// Progress manages the progress of a planning effort.
pub struct Search<A: Algorithm, H: Halt<A>> {
    /// Storage container for the progress of the search algorithm
    memory: A::Memory,

    /// The object which determines the search pattern
    algorithm: Arc<A>,

    /// The options that moderate the progress of the solving
    halting: H,
}

impl<A: Algorithm, H: Halt<A>> Search<A, H> {
    pub fn new(memory: A::Memory, algorithm: Arc<A>, halting: H) -> Self {
        Self {
            memory,
            algorithm,
            halting,
        }
    }

    pub fn into_abstract(self) -> Abstract<A::Solution>
    where
        A: 'static,
        H: 'static,
    {
        Abstract {
            implementation: Box::new(RefCell::new(self)),
        }
    }

    /// Tell the planner to attempt to solve the problem. This will run the
    /// step() function until a solution is found, the progress gets
    /// interrupted, or the algorithm determines that the problem is impossible
    /// to solve.
    pub fn solve(
        &mut self,
    ) -> Result<Status<A::Solution>, A::StepError>
    {
        loop {
            if self.halting.halt(self) {
                return Ok(Status::Incomplete);
            }

            let result = self.step()?;
            if let Status::Incomplete = result {
                continue;
            }

            return Ok(result);
        }
    }

    pub fn step(
        &mut self,
    ) -> Result<Status<A::Solution>, A::StepError>
    {
        self.algorithm.step(&mut self.memory)
    }

    pub fn memory(&self) -> &A::Memory {
        &self.memory
    }

    pub fn memory_mut(&mut self) -> &mut A::Memory {
        &mut self.memory
    }

    /// Change the halting behavior for this progress.
    pub fn with_halting<NewHalt: Halt<A>>(
        self,
        halting: NewHalt
    ) -> Search<A, NewHalt> {
        Search {
            memory: self.memory,
            algorithm: self.algorithm,
            halting,
        }
    }
}

pub trait Interface<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>>;

    fn step(&mut self) -> anyhow::Result<Status<Solution>>;
}

impl<A, H> Interface<A::Solution> for Search<A, H>
where
    A: Algorithm,
    H: Halt<A>,
{
    fn solve(&mut self) -> anyhow::Result<Status<A::Solution>> {
        Search::solve(self).map_err(anyhow::Error::new)
    }

    fn step(&mut self) -> anyhow::Result<Status<A::Solution>> {
        Search::step(self).map_err(anyhow::Error::new)
    }
}

trait WithHalting<H> {
    fn halting(&self) -> &H;

    fn halting_mut(&mut self) -> &mut H;

    /// Modify the existing options and return the progress
    fn tweak_halting<F: FnOnce(&mut H)>(mut self, tweak: F) -> Self {
        tweak(&mut self.halting_mut());
        self
    }
}

impl<A, H: Halt<A>> WithHalting<H> for Search<A, H> {
    fn halting(&self) -> &H {
        &self.halting
    }

    fn halting_mut(&mut self) -> &mut H {
        &mut self.halting
    }
}

pub trait InterfaceWithHalting<Solution, Halting>:
    Interface<Solution> + WithHalting<Halting>
{
}

/// Everything that implements Interface<Solution> and WithOptions<Options>
/// automatically implements InterfaceWithOptions<Solution, Options>
impl<Solution, Halting, T: Interface<Solution> + WithHalting<Halting>>
    InterfaceWithHalting<Solution, Halting> for T
{
}

pub struct Abstract<Solution> {
    implementation: Box<RefCell<dyn Interface<Solution>>>,
}

impl<Solution> Interface<Solution> for Abstract<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().solve()
    }

    fn step(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().step()
    }
}
