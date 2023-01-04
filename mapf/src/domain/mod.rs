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

use anyhow::Error as AnyError;

/// A domain that is being planned over must, at a minimum, specify a type for
/// its state representation and what kind of actions can be performed.
///
/// Use DefineDomain<S, A> to compose a domain using the properties provided by
/// mapf or using your own custom defined properties.
///
/// Domain properties that are provided by mapf out-of-the-box can be found in
/// the sub-modules of this domain module.
pub trait Domain {
    type State;
    type Action;
    type Error;
}

/// To begin defining a domain, use [`DefineDomain::new()`].
pub struct DefineDomain<State, Action, Error=AnyError> {
    _ignore: std::marker::PhantomData<(State, Action, Error)>
}

impl<State, Action, Error> DefineDomain<State, Action, Error> {
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
        DefineDomain { _ignore: Default::default() }
    }
}

impl<State, Action, Error> Domain for DefineDomain<State, Action, Error> {
    type State = State;
    type Action = Action;
    type Error = Error;
}

/// Incorporates a property into a domain. Use `.with(~)` on a domain to create
/// this struct.
pub struct Incorporated<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Incorporated<Base, Prop> {
    type State = Base::State;
    type Action = Base::Action;
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

pub struct Chained<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Chained<Base, Prop> {
    type State = Base::State;
    type Action = Base::Action;
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
    fn chain<Prop>(self, prop: Prop) -> Chained<Self, Prop> where Self: Sized {
        Chained { base: self, prop }
    }
}

pub struct Mapped<Base, Prop> {
    pub base: Base,
    pub prop: Prop,
}
impl<Base: Domain, Prop> Domain for Mapped<Base, Prop> {
    type State = Base::State;
    type Action = Base::Action;
    type Error = Base::Error;
}

pub trait Map {
    /// Maps (modifies) some trait(s) of a domain based on the traits of the
    /// provided property. This can be used to apply constraints to a domain
    /// or to project/lift a sub/super domain.
    ///
    /// Each domain trait may have one, many, or no modifier traits that can map
    /// its behavior. Usually the modifying trait is a different trait from the
    /// one that is being modified. For example the Activity trait can be mapped
    /// by the ActionMap trait.
    fn map<Prop>(self, prop: Prop) -> Mapped<Self, Prop> where Self: Sized {
        Mapped { base: self, prop }
    }
}

pub mod action_map;
pub mod activity;
pub mod dynamics;

pub mod prelude {
    pub use super::*;
    pub use super::activity::*;
    pub use super::action_map::*;
    pub use super::dynamics::*;
}
