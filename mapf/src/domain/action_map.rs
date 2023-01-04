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

/// The ActionMap trait describes a domain property that can modify the
/// choices produced by activities. This can be used to apply constraints or
/// transformations to activities within a domain.
pub trait ActionMap<State, FromAction, FromError> {
    /// What kind of action is produced after mapping this activity
    type ToAction;

    /// What kind of error can happen if a bad state or action is provided
    type Error;

    /// Conrete type for the returned container of actions
    type IterActions<'a>: IntoIterator<Item = Result<Self::ToAction, Self::Error>>
    where
        Self: 'a,
        Self::ToAction: 'a,
        Self::Error: 'a,
        State: 'a;

    /// Implement this function to modify a set of actions.
    fn map_actions<'a>(
        &'a self,
        from_state: &'a State,
        with_actions: impl IntoIterator<Item=Result<FromAction, FromError>>,
    ) -> Self::IterActions<'a>;
}
