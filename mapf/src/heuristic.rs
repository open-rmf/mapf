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
    node::Cost,
    error::Error,
};

pub trait Heuristic<Start, Goal, C: Cost>: std::fmt::Debug {
    type Error: Error;

    fn estimate_cost(
        &self,
        from_state: &Start,
        to_goal: &Goal,
    ) -> Result<Option<C>, Self::Error>;
}
