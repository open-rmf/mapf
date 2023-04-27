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

//! [Multi-Agent Pathfinding (mapf)](https://en.wikipedia.org/wiki/Multi-agent_pathfinding)
//! deals with the problem of finding collision-free paths for multiple mobile
//! agents that need to operate within the same space. This is often treated as
//! an optimization problem where the goal is to minimize the overall cost of
//! the agents' motions. The cost may be related to time spent travelling,
//! distance travelled, battery consumed, priority of each agent, or other factors.
//!
//! There are a variety of techniques and algorithms for solving mapf problems,
//! each with its own trade-offs between optimality, performance, and scalability.
//! Due to the NP-Hard nature of finding optimal solutions for mapf problems (as
//! well as differing ideas of what "optimal" means), it is natural that there
//! is not a one-size-fits-all approach that will be best suited to every use
//! case.
//!
//! For that reason, instead of providing a single solver for the multi-agent
//! pathfinding problem, the mapf crate is implemented as a modular framework
//! for defining and customizing different kinds of solvers. It also provides
//! some "premade" solvers that should address the most common use cases.
//!
//! # Premade Solvers
//!
//! In the [`premade`] module you will find premade solvers that are ready to be
//! applied to common use cases. If you have a typical single- or multi-agent
//! planning problem and you want a quick and easy way to solve it, see the docs
//! of that module for guidance.
//!
//! # Implementing Custom Planners
//!
//! ## Top-Down View
//!
//! To understand how modularity is achieved in the `mapf` crate, it is helpful
//! to take a top-down view of the [`planner`] > [`algorithm`] > [`domain`]
//! architecture. These different modules are related to each other through
//! [traits](https://doc.rust-lang.org/book/ch10-02-traits.html), which is the
//! language feature of Rust that allows programmers to define abstract interfaces.
//! Each module defines traits that can be used by dependent modules so that end
//! users of `mapf` can swap out or customize implementations of critical
//! components at any level of the hierarchy.
//!
//! ### Planner
//!
//! At the top of the hierarchy is the [`planner`] module whose central struct
//! is [`Planner`]. [`Planner`] is the struct that encapsulates all the complex
//! details of initializing a solver, configuring it, and asking it to find a
//! solution to a problem. [`Planner`] is a generic struct that needs to be
//! provided with an "algorithm". There are three crucial traits that a struct
//! needs to implement in order to be a useful algorithm for the [`Planner`]:
//! * [`algorithm::Algorithm`] - marks a struct as implementing an algorithm and
//! specifies a `Memory` struct for storing the data used by the algorithm when
//! performing a search.
//! * [`algorithm::Coherent`] - determines whether a `Start` and `Goal` given
//! to the [`Planner::plan`] function can be used to define a planning problem
//! for the algorithm and then initializes the `Memory` of the algorithm based
//! on the provided `Start` and `Goal` conditions.
//! * [`algorithm::Solvable`] - determines whether a `Goal` given by the user
//! is compatible with the algorithm and implements one incremental step in the
//! algorithm's search procedure. You should make sure that any `Goal` which can
//! be supported by [`algorithm::Coherent`] can also be supported by
//! [`algorithm::Solvable`] or else users of your algorithm will be met with
//! very confusing compilation errors.
//!
//! ### Algorithm
//!
//! In the middle of the hierarchy is the [`algorithm`] module which defines
//! the traits that algorithms need to implement, and also provides some premade
//! algorithm implementations such as [`algorithm::AStar`] and [`algorithm::Dijkstra`].
//! When defining a custom algorithm, the only hard requirement is that it
//! implements the traits required by the [`Planner`] struct, as listed in the
//! [planner subsection](#planner) above.
//!
//! However, most search algorithms are broadly applicable to a wide variety of
//! domains. And there are many domains that could be searched by a variety of
//! algorithms, with various trade-offs between the different algorithms. The
//! choice of which algorithm to use for searching a domain might vary between
//! use cases and applications, depending on factors like the scale of the
//! domain or the importance of a fast vs optimal solution for the specific
//! application.
//!
//! With that in mind, the algorithms implemented in `mapf` are made to be
//! [generic](https://doc.rust-lang.org/book/ch10-01-syntax.html) with respect
//! to their search domains. Each algorithm specifies
//! [trait bounds](https://doc.rust-lang.org/rust-by-example/generics/bounds.html)
//! that must be satisfied by any domain that a user wants the algorithm to
//! search. If a user attempts to pass a domain into an algorithm without the
//! trait bounds being satisfied, Rust will issue a compilation error. In some
//! cases the compilation error might not appear until the user attempts to
//! generate a plan with the incompatible algorithm+domain combination.
//!
//! Different algorithms may be compatible with different types of domains. This
//! can be reflected by carefully selecting the trait bounds that your algorithm
//! requires from the domains it is given. For example, [`algorithm::AStar`]
//! requires its domains to implement the [`domain::Informed`] trait, but
//! [`algorithm::Dijkstra`] does not require that trait.
//!
//! ### Domain
//!
//! Algorithm implementations determine _how_ a domain is searched, but domain
//! implementations determine _what_ is being explored. Each algorithm has a
//! set of traits that its supported domains need to implement. Implementing all
//! of those traits from scratch for each new domain can be overwhelming. Many
//! of the domain traits are boilerplate that won't vary much between different
//! domains. To minimize the boilerplate of creating custom domains, we provide
//! templates that take care of much of the boilerplate:
//! * [`templates::InformedSearch`] - A generic struct for defining a domain
//! that is suitable for performing informed searches. Fill its generic parameters
//! with structs that implement the relevant traits to get a domain that can be
//! used with informed search algorithms.
//! * [`templates::GraphMotion`] - A generic struct for describing motion that
//! can be extrapolated ([`domain::Extrapolator`]) through space, guided by a
//! discrete spatial [`Graph`]. This implements several domain traits that are
//! often coupled to each other, and is suitable for the `Activity` field of
//! [`templates::InformedSearch`].
//!
//! For examples of constructing your own domain using `InformedSearch`, you can
//! refer to the source code of the premade domains:
//! * [`premade::SearchR2`]
//! * [`premade::SearchSE2`]
//! * [`premade::SippSE2`]

#![feature(
    associated_type_bounds,
    type_alias_impl_trait,
    impl_trait_in_assoc_type,
    result_flattening
)]

pub mod domain;

pub mod planner;
pub use planner::Planner;

pub mod graph;
pub use graph::Graph;

pub mod algorithm;

pub mod templates;

pub mod motion;

pub mod negotiation;

pub mod error;

pub mod premade;

mod util;

pub mod prelude {
    pub use super::algorithm::*;
    pub use super::domain::*;
    pub use super::graph::*;
    pub use super::planner::*;
    pub use super::premade::*;
}
