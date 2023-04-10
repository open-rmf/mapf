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

use crate::domain::{ConflictAvoider, Extrapolator};
use std::sync::Arc;

#[derive(Debug)]
pub struct ConflictAvoidance<Avoider, Environment> {
    pub avoider: Avoider,
    pub environment: Arc<Environment>,
}

impl<Avoider: Clone, Environment> Clone for ConflictAvoidance<Avoider, Environment> {
    fn clone(&self) -> Self {
        Self {
            avoider: self.avoider.clone(),
            environment: self.environment.clone(),
        }
    }
}

impl<Avoider, Env, State, Target, Guidance, Key> Extrapolator<State, Target, Guidance, Key>
    for ConflictAvoidance<Avoider, Env>
where
    Avoider: ConflictAvoider<State, Target, Guidance, Key, Env>,
{
    type Extrapolation = Avoider::AvoidanceAction;
    type ExtrapolationError = Avoider::AvoidanceError;
    type ExtrapolationIter<'a> = impl Iterator<Item=Result<(Avoider::AvoidanceAction, State), Self::ExtrapolationError>> + 'a
    where
        Self: 'a,
        Self::Extrapolation: 'a,
        Self::ExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    fn extrapolate<'a>(
        &'a self,
        from_state: &State,
        to_target: &Target,
        with_guidance: &Guidance,
        for_keys: (Option<&Key>, Option<&Key>),
    ) -> Self::ExtrapolationIter<'a>
    where
        Self: 'a,
        Self::Extrapolation: 'a,
        Self::ExtrapolationError: 'a,
        State: 'a,
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
    {
        self.avoider
            .avoid_conflicts(from_state, to_target, with_guidance, for_keys, &*self.environment)
            .into_iter()
    }
}
