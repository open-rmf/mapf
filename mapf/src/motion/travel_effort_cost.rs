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

use crate::{
    domain::{Weighted, Cost, Reversible},
    motion::{Timed, Measurable},
    error::NoError,
};

#[derive(Debug, Clone, Copy)]
pub struct TravelEffortCost {
    pub time: f64,
    pub translation: f64,
    pub rotation: f64,
}

impl TravelEffortCost {
    /// Use this function to intuitively set the weights for travel effort.
    /// Allow the agent to arrive at its goal one second later than the
    /// time-optimal path if doing so will avoid a detour of `meters` of
    /// translation and `radians` of spinning. This is proportional, so an
    /// arrival delay of `t` seconds will be permitted to avoid a detour of
    /// `t * meters` translation and `t * radians` rotation.
    pub fn delay_one_second_instead_of_detouring_by(
        meters: f64,
        radians: f64,
    ) -> Self {
        Self {
            time: 1.0,
            translation: 1.0/meters,
            rotation: 1.0/radians,
        }
    }
}

/// Allow 1s of delay to avoid 1m of detour.
const DEFAULT_TRANSLATION_EFFORT_ALLOWANCE: f64 = 1.0;
/// Allow 1s of delay to avoid 180-degrees of excess rotation.
const DEFAULT_ROTATION_EFFORT_ALLOWANCE: f64 = 180.0 * std::f64::consts::PI / 180.0;

impl Default for TravelEffortCost {
    fn default() -> Self {
        TravelEffortCost::delay_one_second_instead_of_detouring_by(
            DEFAULT_TRANSLATION_EFFORT_ALLOWANCE,
            DEFAULT_ROTATION_EFFORT_ALLOWANCE,
        )
    }
}

impl<State: Timed, Action: Measurable<State>> Weighted<State, Action> for TravelEffortCost {
    type Cost = Cost<f64>;
    type WeightedError = NoError;

    fn cost(
        &self,
        from_state: &State,
        action: &Action,
        to_state: &State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        // Using the absolute value of the difference allows this same
        // implementation to work both forwards and backwards in time. We are
        // assuming that any case where time is decreasing in the child state,
        // a reverse search is being performed.
        let duration = (*to_state.time() - *from_state.time()).as_secs_f64().abs();
        let arclength = action.arclength(from_state, to_state);
        let cost = self.time * duration
            + self.translation * arclength.translational
            + self.rotation * arclength.rotational;

        Ok(Some(Cost(cost)))
    }

    fn initial_cost(
        &self,
        _: &State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        Ok(Some(Cost(0.0)))
    }
}

impl Reversible for TravelEffortCost {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> where Self: Sized {
        Ok(*self)
    }
}
