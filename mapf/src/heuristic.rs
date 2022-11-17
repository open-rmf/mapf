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

use crate::{
    error::{Error, NoError},
    node::Cost,
};

pub trait Heuristic<Start, Goal, C: Cost> {
    type Error: Error;

    fn estimate_cost(&self, from_state: &Start, to_goal: &Goal) -> Result<Option<C>, Self::Error>;
}

/// In cases where a heuristic needs to be specified for a generic argument but
/// you know that it won't actually be used (e.g. you'll only be using aimless
/// expansion), then you can pass in an Uninformed heuristic as a placeholder.
/// This could also be used as a reference point for benchmarking heuristic
/// performance.
pub struct Uninformed;
impl<S, G, C: Cost> Heuristic<S, G, C> for Uninformed {
    type Error = NoError;
    fn estimate_cost(&self, from_state: &S, to_goal: &G) -> Result<Option<C>, Self::Error> {
        Ok(Some(C::zero()))
    }
}
