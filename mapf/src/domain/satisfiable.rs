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

use crate::error::NoError;

/// The `Satisfiable` trait allows search algorithms to recognize when a state
/// has reached a goal.
pub trait Satisfiable<State, Goal> {
    type SatisfactionError;

    fn is_satisfied(
        &self,
        by_state: &State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError>;
}

// Implement Satisfiable for an empty tuple by accepting a state as the goal and
// doing an equality comparison.
impl<State: PartialEq> Satisfiable<State, State> for () {
    type SatisfactionError = NoError;
    fn is_satisfied(
        &self,
        by_state: &State,
        for_goal: &State,
    ) -> Result<bool, Self::SatisfactionError> {
        Ok(by_state.eq(for_goal))
    }
}