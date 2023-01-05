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

/// A domain that is being planned over must, at a minimum, specify a type for
/// its state representation and what kind of actions can be performed.
///
/// Use DefineTrait<S, A> to construct the traits of a domain using the trait
/// impls provided by mapf or by using your own custom defined trait impls.
///
/// Domain trait impls that are provided by mapf out-of-the-box can be found in
/// the sub-modules of this domain module, such as Activity and Dynamics.
pub trait Domain {
    type State;
    type Action;
    type Error;
}

pub mod action_map;
pub mod activity;
pub mod define_trait;
pub mod dynamics;

pub mod prelude {
    pub use super::*;
    pub use super::activity::*;
    pub use super::action_map::*;
    pub use super::define_trait::*;
    pub use super::dynamics::*;
}
