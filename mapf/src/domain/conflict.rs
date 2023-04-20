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

/// Create an action that extrapolates from an initial state to a target while
/// avoiding conflicts with the environment.
pub trait ConflictAvoider<State, Target, Guidance, Key, Environment> {
    type AvoidanceAction;
    type AvoidanceError;

    type AvoidanceActionIter<'a>: IntoIterator<Item = Result<(Self::AvoidanceAction, State), Self::AvoidanceError>>
        + 'a
    where
        Self: 'a,
        Self::AvoidanceAction: 'a,
        Self::AvoidanceError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
        Environment: 'a;

    fn avoid_conflicts<'a>(
        &'a self,
        from_state: &State,
        to_target: &Target,
        with_guidance: &Guidance,
        for_keys: (Option<&Key>, Option<&Key>),
        in_environment: &Environment,
    ) -> Self::AvoidanceActionIter<'a>
    where
        Self: 'a,
        Self::AvoidanceAction: 'a,
        Self::AvoidanceError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Environment: 'a,
        Key: 'a;
}
