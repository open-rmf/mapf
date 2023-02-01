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
    algorithm::{Algorithm, Solvable, Status},
    halt::Halt,
    error::Error,
};

/// Progress manages the progress of a planning effort.
pub struct Search<Algo: Algorithm, Goal, Halting> {
    /// Storage container for the progress of the search algorithm
    memory: Algo::Memory,

    /// The object which determines the search pattern
    algorithm: Arc<Algo>,

    /// The goal that the search is trying to reach
    goal: Goal,

    /// The options that moderate the progress of the solving
    halting: Halting,
}

impl<Algo: Algorithm, Goal, Halting> Search<Algo, Goal, Halting> {
    pub fn new(
        memory: Algo::Memory,
        algorithm: Arc<Algo>,
        goal: Goal,
        halting: Halting,
    ) -> Self {
        Self {
            memory,
            algorithm,
            goal,
            halting,
        }
    }

    /// Tell the planner to attempt to solve the problem. This will run the
    /// step() function until a solution is found, the progress gets
    /// interrupted, or the algorithm determines that the problem is impossible
    /// to solve.
    pub fn solve(
        &mut self,
    ) -> Result<Status<Algo::Solution>, Algo::StepError>
    where
        Algo: Solvable<Goal>,
        Halting: Halt<Algo::Memory>,
    {
        loop {
            if self.halting.halt(&self.memory) {
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
    ) -> Result<Status<Algo::Solution>, Algo::StepError>
    where
        Algo: Solvable<Goal>,
    {
        self.algorithm.step(&mut self.memory, &self.goal)
    }

    pub fn memory(&self) -> &Algo::Memory {
        &self.memory
    }

    pub fn memory_mut(&mut self) -> &mut Algo::Memory {
        &mut self.memory
    }

    /// Change the halting behavior for this progress.
    pub fn with_halting<NewHalt>(
        self,
        halting: NewHalt
    ) -> Search<Algo, Goal, NewHalt> {
        Search {
            memory: self.memory,
            algorithm: self.algorithm,
            goal: self.goal,
            halting,
        }
    }

    /// Modify the existing options and return the progress
    fn tweak_halting<F: FnOnce(&mut Halting)>(mut self, tweak: F) -> Self {
        tweak(&mut self.halting);
        self
    }
}

pub trait Interface<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>>;

    fn step(&mut self) -> anyhow::Result<Status<Solution>>;
}

impl<A, G, H> Interface<A::Solution> for Search<A, G, H>
where
    A: Solvable<G>,
    H: Halt<A::Memory>,
    A::StepError: Error,
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
}

impl<A: Algorithm, G, H: Halt<A::Memory>> WithHalting<H> for Search<A, G, H> {
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

impl<A, G, H> From<Search<A, G, H>> for Abstract<A::Solution>
where
    A: Solvable<G>,
    A::StepError: Error,
    H: Halt<A::Memory>,
{
    fn from(value: Search<A, G, H>) -> Self {
        Abstract { implementation: Box::new(RefCell::new(value)) }
    }
}

impl<Solution> Interface<Solution> for Abstract<Solution> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().solve()
    }

    fn step(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().step()
    }
}

pub struct AbstractWithHalting<Solution, Halting> {
    implementation: Box<RefCell<dyn InterfaceWithHalting<Solution, Halting>>>,
}

impl<A, G, H> From<Search<A, G, H>> for AbstractWithHalting<A::Solution, H>
where
    A: Solvable<G> + 'static,
    H: Halt<A::Memory> + 'static,
    A::StepError: Error,
    G: 'static,
{
    fn from(value: Search<A, G, H>) -> Self {
        AbstractWithHalting { implementation: Box::new(RefCell::new(value)) }
    }
}

impl<Solution, Halting> Interface<Solution> for AbstractWithHalting<Solution, Halting> {
    fn solve(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().solve()
    }

    fn step(&mut self) -> anyhow::Result<Status<Solution>> {
        self.implementation.borrow_mut().step()
    }
}

impl<Solution, Halting> WithHalting<Halting> for AbstractWithHalting<Solution, Halting> {
    fn halting(&self) -> &Halting {
        self.implementation.borrow().halting()
    }

    fn halting_mut(&mut self) -> &mut Halting {
        self.implementation.borrow_mut().halting_mut()
    }
}
