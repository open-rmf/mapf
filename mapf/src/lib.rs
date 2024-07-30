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
#![feature(
    associated_type_bounds,
    type_alias_impl_trait,
    impl_trait_in_assoc_type,
    result_flattening
)]
// TODO(@mxgrey): Eliminate the need for this by reducing the complexity of
// struct names, e.g. the typename for the complex chain that implements Lifted
// in activity.rs. This will probably go hand-in-hand with removing the need for
// nightly features, and improve compile times all at once.
#![type_length_limit = "157633981"]

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
