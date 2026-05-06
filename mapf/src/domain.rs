/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

/// A domain that is being planned over must, at a minimum, specify a type for
/// its state representation and a type for errors that may occur while using
/// the domain.
///
/// Domains may also implement various traits like [`Activity`], [`Weighted`],
/// and [`Informed`] which can be used by planners to search the domain. You can
/// easily gather implementations for these traits into a domain using the
/// `#[derive(Domain)]` macro.
///
///
pub trait Domain {
    /// Data structure that represents a state within this domain.
    type State;

    /// The error type that this domain may produce from its various operations.
    type Error;
}

pub use mapf_derive::Domain;

pub mod action_map;
pub use action_map::*;

pub mod activity;
pub use activity::*;

pub mod closable;
pub use closable::*;

pub mod configurable;
pub use configurable::*;

pub mod conflict;
pub use conflict::*;

pub mod connectable;
pub use connectable::*;

pub mod cost;
pub use cost::Cost;

pub mod define_trait;
pub use define_trait::*;

pub mod domain_map;
pub use domain_map::*;

pub mod extrapolator;
pub use extrapolator::*;

pub mod informed;
pub use informed::*;

pub mod initializable;
pub use initializable::*;

pub mod keyed;
pub use keyed::*;

pub mod reversible;
pub use reversible::*;

pub mod satisfiable;
pub use satisfiable::*;

pub mod space;
pub use space::*;

pub mod state_map;
pub use state_map::*;

pub mod weighted;
pub use weighted::*;
