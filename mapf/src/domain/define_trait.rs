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

use super::*;

use anyhow::Error as AnyError;

/// To begin defining a trait for a domain, use [`DefineTrait::new()`].
pub struct DefineTrait<State, Error=AnyError> {
    _ignore: std::marker::PhantomData<(State, Error)>
}

impl<State, Error> DefineTrait<State, Error> {
    /// This function begins defining a domain by indicating what the top-level
    /// state and action representation is.
    ///
    /// Add properties to the domain using `.with(~)` from [`Incorporate`],
    /// `.chain(~)` from [`Chain`], and `.map(~)` from [`Map`].
    ///
    /// Properties that are incorporated into this domain must be compatible
    /// with the State and Action types that you specify here. Make sure that
    /// the State and Action have whatever traits are required by the domain
    /// properties that you add.
    pub fn new() -> Self {
        DefineTrait { _ignore: Default::default() }
    }
}

impl<State, Error> Domain for DefineTrait<State, Error> {
    type State = State;
    type Error = Error;
}

impl<State, Error> Clone for DefineTrait<State, Error> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

impl<State, Error> Default for DefineTrait<State, Error> {
    fn default() -> Self {
        Self::new()
    }
}
/// Incorporates a property into a domain. Use `.with(~)` on a domain to create
/// this struct.
#[derive(Debug, Clone)]
pub struct Incorporated<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Incorporated<Base, Prop> {
    type State = Base::State;
    type Error = Base::Error;
}

pub trait Incorporate {
    /// Provides the `.with(~)` function for domains so you can incorporate new
    /// properties into them.
    ///
    /// When you incorporate a property into a domain, any traits that the
    /// property implements must not be implemented by any properties that were
    /// already incorporated into the domain, otherwise there will be ambiguity
    /// about which property's implementation to use for the trait, and the
    /// compiler will refuse to allow that trait to be used in an algorithm.
    ///
    /// If you want to mix the trait implementations of different properties
    /// into one domain then you may want to use the `.chain(~)` function
    /// instead. Or some traits come with modifier traits that can be
    /// incorporated into a domain to modify a property's trait implementation,
    /// such as how ActionMap can modify the choices of an Activity.
    fn with<Prop>(self, prop: Prop) -> Incorporated<Self, Prop> where Self: Sized;
}

impl<D: Domain> Incorporate for D {
    fn with<Prop>(self, prop: Prop) -> Incorporated<Self, Prop> where Self: Sized {
        Incorporated { base: self, prop }
    }
}

#[derive(Debug, Clone)]
pub struct Chained<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Chained<Base, Prop> {
    type State = Base::State;
    type Error = Base::Error;
}

pub trait Chain {
    /// Chains the trait implementation of a property with the trait
    /// implementation of a different property that already existed in the
    /// domain. The exact meaning of chaining may vary between traits.
    ///
    /// For example if you chain two properties that implement Activity then
    /// the domain's overall implementation of Activity::choices will iterate
    /// over all choices available from all of the chained activities.
    fn chain<Prop>(self, prop: Prop) -> Chained<Self, Prop> where Self: Sized;
}

impl<D: Domain> Chain for D {
    fn chain<Prop>(self, prop: Prop) -> Chained<Self, Prop> where Self: Sized {
        Chained { base: self, prop }
    }
}

#[derive(Debug, Clone)]
pub struct Mapped<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Mapped<Base, Prop> {
    type State = Base::State;
    type Error = Base::Error;
}

pub trait Map {
    /// Maps (modifies) some trait(s) of a domain based on the traits of the
    /// provided property. This can be used to apply constraints to a domain
    /// or to project/lift a sub/super domain.
    ///
    /// Each domain trait may a modifier trait that can map its behavior.
    /// Usually the modifying trait is a different trait from the one that is
    /// being modified. For example the Activity trait can be mapped by the
    /// ActionMap trait.
    fn map<Prop>(self, prop: Prop) -> Mapped<Self, Prop> where Self: Sized;
}

impl<D: Domain> Map for D {
    fn map<Prop>(self, prop: Prop) -> Mapped<Self, Prop> where Self: Sized  {
        Mapped { base: self, prop }
    }
}

#[derive(Debug, Clone)]
pub struct Lifted<Base, Lifter, Prop> {
    pub base: Base,
    pub lifter: Lifter,
    pub prop: Prop,
}
impl<Base: Domain, Lifter, Prop> Domain for Lifted<Base, Lifter, Prop> {
    type State = Base::State;
    type Error = Base::Error;
}

pub trait Lift {
    /// Lifts from the domain of property into the base domain using Lifter.
    ///
    /// This can be used to incorporate subspace behaviors or projected domains
    /// into your domain.
    fn lift<Lifter, Prop>(
        self,
        lifter: Lifter,
        prop: Prop,
    ) -> Lifted<Self, Lifter, Prop> where Self: Sized;

    /// Chains a lifted property.
    fn chain_lift<Lifter, Prop>(
        self,
        lifter: Lifter,
        prop: Prop
    ) -> ChainedLift<Self, Lifter, Prop>
    where
        Self: Sized + Domain;
}

type ChainedLift<Base, Lifter, Prop> = Chained<
    Base,
    Lifted<
        DefineTrait<<Base as Domain>::State, <Base as Domain>::Error>,
        Lifter,
        Prop,
    >,
>;

impl<D: Domain> Lift for D {
    fn lift<Lifter, Prop>(
        self,
        lifter: Lifter,
        prop: Prop,
    ) -> Lifted<Self, Lifter, Prop> where Self: Sized {
        Lifted { base: self, lifter, prop }
    }

    fn chain_lift<Lifter, Prop>(
        self,
        lifter: Lifter,
        prop: Prop,
    ) -> ChainedLift<Self, Lifter, Prop> where Self: Sized {
        self.chain(
            DefineTrait::<D::State, D::Error>::new()
            .lift(lifter, prop)
        )
    }
}
