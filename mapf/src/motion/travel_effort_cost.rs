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
    /// Allow the agent to take a detour of up to `meters` translation and
    /// `radians` spinning from the shortest path in order to arrive at the goal
    /// one second earlier. These measures are proportional, so in general a
    /// detour of `t * meters` translation and `t * radians` spinning will be
    /// allowed to arrive at the goal `t` seconds earlier.
    ///
    /// "Detour" in this context means taking a path which is longer than the
    /// shortest available path to the goal. This can be advantageous if the
    /// shortest possible path is crowded with moving obstacles that will slow
    /// down the agent's ability to make progress. Therefore the `meters` and
    /// `radians` weights will only have an effect in the presence of moving
    /// obstacles which would delay the agent if it were to move along the
    /// shortest possible path to its goal.
    ///
    /// Broadly speaking, larger values tend to result in longer planning times
    /// in environments cluttered with dynamic obstacles or other moving agents.
    /// That's because they allow for longer detours and finding those detours
    /// requires more exploration. Using small values here can make planning
    /// times faster, but then the agent is less likely to search for time-saving
    /// detours, for better or worse.
    ///
    /// # Arguments
    ///
    /// * `meters` - The agent will accept a detour of up to `t * meters` where
    /// `t` is how many seconds the agent can arrive earlier by detouring.
    ///
    /// * `radians` - The agent will accept a detour that involves `t * radians`
    /// spinning where `t` is how many seconds the agent can arrive earlier by
    /// allowing the spinning.
    pub fn save_one_second_with_detour_up_to(
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

/// Detour by up to 5m to arrive one second earlier.
const DEFAULT_TRANSLATION_EFFORT_ALLOWANCE: f64 = 5.0;
/// Spin by up to 360 degrees to arrive one second earlier.
const DEFAULT_ROTATION_EFFORT_ALLOWANCE: f64 = 360.0 * std::f64::consts::PI / 180.0;

impl Default for TravelEffortCost {
    fn default() -> Self {
        TravelEffortCost::save_one_second_with_detour_up_to(
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
