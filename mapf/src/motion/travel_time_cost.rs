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
    motion::Timed,
    error::NoError,
};

#[derive(Debug, Clone, Copy)]
pub struct TravelTimeCost(pub f64);

impl Default for TravelTimeCost {
    fn default() -> Self {
        TravelTimeCost(1.0)
    }
}

impl<State: Timed, Action> Weighted<State, Action> for TravelTimeCost {
    type Cost = Cost<f64>;
    type WeightedError = NoError;

    fn cost(
        &self,
        from_state: &State,
        _: &Action,
        to_state: &State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        // Using the absolute value of the difference allows this same
        // implementation to work both forwards and backwards in time. We are
        // assuming that any case where time is decreasing in the child state,
        // a reverse search is being performed.
        let duration = (*to_state.time() - *from_state.time()).as_secs_f64().abs();
        Ok(Some(Cost(duration*self.0)))
    }

    fn initial_cost(
        &self,
        _: &State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        Ok(Some(Cost(0.0)))
    }
}

impl Reversible for TravelTimeCost {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> where Self: Sized {
        Ok(*self)
    }
}
