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
    domain::Weighted,
    motion::{Timed, Duration},
    error::NoError,
    Cost,
};

pub struct TravelTimeCost(pub f64);

impl<State: Timed, Action> Weighted<State, Action> for TravelTimeCost {
    type Cost = Cost<f64>;
    type WeightedError = NoError;

    fn cost(
        &self,
        from_state: &State,
        _: &Action,
        to_state: &State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let duration = (*to_state.time() - *from_state.time()).as_secs_f64();
        Ok(Some(Cost(duration*self.0)))
    }

    fn initial_cost(
        &self,
        _: &State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        Ok(Some(Cost(0.0)))
    }
}
