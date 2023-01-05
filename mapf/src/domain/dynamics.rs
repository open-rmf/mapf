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

use super::prelude::*;

/// The Dynamics trait describes how a state gets transformed by an action.
///
/// In graph theory terms, this produces the next vertex of a graph, given the
/// current vertex and an outgoing edge.
///
/// Note that when you chain Dynamics properties, the implementation of advance
/// will be called on each chained property in the order that they are chained
/// in. When chaining Dynamics properties, make sure the implementations are not
/// doubling up their effects on the state.
pub trait Dynamics<State, Action> {
    /// What kind of error can happen if a bad state and/or action is provided
    type Error;

    /// Advance the given state based on the given action.
    ///
    /// If the dynamics are constrained and the action would lead to a state
    /// that violates the constraint, return Ok(None).
    fn advance(&self, state: State, action: &Action) -> Result<Option<State>, Self::Error>;
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::Error: Into<Base::Error>,
{
    type Error = Base::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::Error> {
        self.prop.advance(state, action).map_err(Into::into)
    }
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Chained<Base, Prop>
where
    Base: Domain + Dynamics<Base::State, Base::Action>,
    <Base as Dynamics<Base::State, Base::Action>>::Error: Into<<Base as Domain>::Error>,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::Error: Into<<Base as Domain>::Error>,
{
    type Error = <Base as Domain>::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::Error> {
        // NOTE: The base dynamics are being applied before the chained prop dynamics
        self.base.advance(state, action)
            .map_err(Into::into)
            .and_then(|state|
                state
                    .map(|state| self.prop.advance(state, action).map_err(Into::into))
                    .transpose()
                    .map(|r| r.flatten())
            )
    }
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Mapped<Base, Prop>
where
    Base: Domain + ActionMap<Base::State, <Base as Domain>::Action>,
    <Base as ActionMap<Base::State, Base::Action>>::Error: Into<<Base as Domain>::Error>,
    Base::State: Clone,
    Base::Action: Clone,
    Prop: Dynamics<Base::State, Base::ToAction>,
    Prop::Error: Into<<Base as Domain>::Error>,
{
    type Error = <Base as Domain>::Error;
    fn advance(&self, mut state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::Error> {
        let input_state = state.clone();
        for action in self.base.map_actions(&input_state, action.clone()) {
            let action = action.map_err(Into::into)?;
            state = match self.prop.advance(state, &action).map_err(Into::into)? {
                Some(state) => state,
                None => return Ok(None),
            };
        }

        Ok(Some(state))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::NoError;
    use approx::assert_relative_eq;

    #[derive(Debug, PartialEq)]
    struct TimeDelta(f64);

    #[derive(Debug, Clone, PartialEq)]
    struct TestState {
        time: f64,
        battery: f64,
    }

    trait AdvanceTime {
        fn advance_time(&mut self, delta: f64);
    }
    impl AdvanceTime for TestState {
        fn advance_time(&mut self, delta: f64) {
            self.time += delta;
        }
    }

    trait BatteryPowered {
        fn charge_battery(&mut self, delta: f64);
        fn drain_battery(&mut self, delta: f64) {
            self.charge_battery(-delta);
        }
    }
    impl BatteryPowered for TestState {
        fn charge_battery(&mut self, delta: f64) {
            self.battery += delta;
        }
    }

    struct TimePassage;
    impl<S: AdvanceTime> Dynamics<S, TimeDelta> for TimePassage {
        type Error = NoError;
        fn advance(&self, mut state: S, action: &TimeDelta) -> Result<Option<S>, Self::Error> {
            state.advance_time(action.0);
            Ok(Some(state))
        }
    }

    struct DrainBattery(f64 /* charge per second */);
    impl<S: BatteryPowered> Dynamics<S, TimeDelta> for DrainBattery {
        type Error = NoError;
        fn advance(&self, mut state: S, action: &TimeDelta) -> Result<Option<S>, Self::Error> {
            state.drain_battery(action.0 * self.0);
            Ok(Some(state))
        }
    }

    #[test]
    fn test_composed_dynamics() {
        let domain = DefineTrait::<TestState, TimeDelta>::new()
            .with(TimePassage)
            .chain(DrainBattery(0.02));

        let initial_state = TestState{ time: 0.1, battery: 1.0 };
        let action = TimeDelta(4.0);
        let next_state = domain.advance(initial_state.clone(), &action).unwrap().unwrap();
        assert_ne!(initial_state, next_state);
        assert_relative_eq!(next_state.time, initial_state.time + action.0);
        assert_relative_eq!(next_state.battery, initial_state.battery - action.0 * 0.02);
    }


}
