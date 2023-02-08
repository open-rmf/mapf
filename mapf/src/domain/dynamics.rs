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

use super::*;

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
    type DynamicsError;

    /// Advance the given state based on the given action.
    ///
    /// If the dynamics are constrained and the action would lead to a state
    /// that violates the constraint, return Ok(None).
    fn advance(&self, state: State, action: &Action) -> Result<Option<State>, Self::DynamicsError>;
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::DynamicsError: Into<Base::Error>,
{
    type DynamicsError = Base::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::DynamicsError> {
        self.prop.advance(state, action).map_err(Into::into)
    }
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Chained<Base, Prop>
where
    Base: Domain + Dynamics<Base::State, Base::Action>,
    <Base as Dynamics<Base::State, Base::Action>>::DynamicsError: Into<<Base as Domain>::Error>,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::DynamicsError: Into<<Base as Domain>::Error>,
{
    type DynamicsError = <Base as Domain>::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::DynamicsError> {
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
    Base: Domain + Dynamics<Base::State, Prop::ToAction>,
    Prop: ActionMap<Base::State, Base::Action>,
    Prop::ActionMapError: Into<<Base as Domain>::Error>,
    Base::State: Clone,
    Base::Action: Clone,
    Base::DynamicsError: Into<<Base as Domain>::Error>,
    Prop::ActionMapError: Into<<Base as Domain>::Error>,
{
    type DynamicsError = <Base as Domain>::Error;
    fn advance(&self, mut state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::DynamicsError> {
        let input_state = state.clone();
        for action in self.prop.map_action(input_state, action.clone()) {
            let action = action.map_err(Into::into)?;
            state = match self.base.advance(state, &action).map_err(Into::into)? {
                Some(state) => state,
                None => return Ok(None),
            };
        }

        Ok(Some(state))
    }
}

impl<Base, Lifter, Prop> Dynamics<Base::State, Base::Action> for Lifted<Base, Lifter, Prop>
where
    Base: Domain,
    Lifter: StateMap<Base::State> + ActionMap<Base::State, Base::Action>,
    Lifter::ActionMapError: Into<Base::Error>,
    Lifter::ProjectionError: Into<Base::Error>,
    Lifter::LiftError: Into<Base::Error>,
    Prop: Dynamics<Lifter::ProjectedState, Lifter::ToAction>,
    Base::State: Clone,
    Base::Action: Clone,
    Prop::DynamicsError: Into<Base::Error>,
{
    type DynamicsError = Base::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Option<Base::State>, Self::DynamicsError> {
        let original_state = state.clone();
        let mut projected_state = match self.lifter.project(state).map_err(Into::into)? {
            Some(s) => s,
            None => return Ok(None),
        };

        for action in self.lifter.map_action(original_state.clone(), action.clone()) {
            let action = action.map_err(Into::into)?;
            projected_state = match self.prop.advance(projected_state, &action).map_err(Into::into)? {
                Some(state) => state,
                None => return Ok(None),
            };
        }

        self.lifter.lift(original_state, projected_state).map_err(Into::into)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::NoError;
    use approx::assert_relative_eq;

    #[derive(Debug, Clone, PartialEq)]
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
        fn battery_level(&self) -> f64;
    }
    impl BatteryPowered for TestState {
        fn charge_battery(&mut self, delta: f64) {
            self.battery += delta;
        }
        fn battery_level(&self) -> f64 {
            self.battery
        }
    }

    struct TimePassage;
    impl<S: AdvanceTime> Dynamics<S, TimeDelta> for TimePassage {
        type DynamicsError = NoError;
        fn advance(&self, mut state: S, action: &TimeDelta) -> Result<Option<S>, Self::DynamicsError> {
            state.advance_time(action.0);
            Ok(Some(state))
        }
    }

    struct TimeBatteryDrain(f64 /* charge per second */);
    impl<S: BatteryPowered> Dynamics<S, TimeDelta> for TimeBatteryDrain {
        type DynamicsError = NoError;
        fn advance(&self, mut state: S, action: &TimeDelta) -> Result<Option<S>, Self::DynamicsError> {
            state.drain_battery(action.0 * self.0);
            if state.battery_level() < 0.0 {
                Ok(None)
            } else {
                Ok(Some(state))
            }
        }
    }

    #[test]
    fn test_composed_dynamics() {
        let domain = DefineTrait::<TestState, TimeDelta>::new()
            .with(TimePassage)
            .chain(TimeBatteryDrain(0.02));

        let initial_state = TestState{ time: 0.1, battery: 1.0 };
        let action = TimeDelta(4.0);
        let next_state = domain.advance(initial_state.clone(), &action).unwrap().unwrap();
        assert_ne!(initial_state, next_state);
        assert_relative_eq!(next_state.time, initial_state.time + action.0);
        assert_relative_eq!(next_state.battery, initial_state.battery - action.0 * 0.02);

        // Drain the battery past its limit
        let invalid_state = domain.advance(initial_state, &TimeDelta(100.0));
        assert!(invalid_state.is_ok());
        if let Ok(invalid_state) = invalid_state {
            assert!(invalid_state.is_none());
        }
    }

    #[derive(Clone)]
    struct Move(f64 /* distance in meters */);

    struct MoveBatteryDrain(f64 /* charge per meter */);
    impl Dynamics<Battery, Move> for MoveBatteryDrain {
        type DynamicsError = NoError;
        fn advance(&self, mut state: Battery, action: &Move) -> Result<Option<Battery>, Self::DynamicsError> {
            state.0 -= self.0 * action.0;
            if state.0 < 0.0 {
                Ok(None)
            } else {
                Ok(Some(state))
            }
        }
    }

    #[derive(Clone)]
    enum HybridAction {
        Time(TimeDelta),
        Move(Move),
    }

    impl From<HybridAction> for Option<TimeDelta> {
        fn from(value: HybridAction) -> Self {
            match value {
                HybridAction::Time(time) => Some(time),
                _ => None,
            }
        }
    }

    impl From<HybridAction> for Option<Move> {
        fn from(value: HybridAction) -> Self {
            match value {
                HybridAction::Move(moving) => Some(moving),
                _ => None,
            }
        }
    }

    #[derive(Clone)]
    struct Battery(f64);
    struct BatterySubspace;
    impl StateSubspace for BatterySubspace {
        type ProjectedState = Battery;
    }
    impl ProjectState<TestState> for BatterySubspace {
        type ProjectionError = NoError;
        fn project(
            &self,
            state: TestState
        ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
            Ok(Some(Battery(state.battery)))
        }
    }
    impl LiftState<TestState> for BatterySubspace {
        type LiftError = NoError;
        fn lift(
            &self,
            original: TestState,
            projection: Self::ProjectedState
        ) -> Result<Option<TestState>, Self::LiftError> {
            Ok(Some(TestState {
                battery: projection.0,
                ..original
            }))
        }
    }

    struct ChargeBattery(f64 /* charge per second */);
    impl Dynamics<Battery, TimeDelta> for ChargeBattery {
        type DynamicsError = NoError;
        fn advance(&self, mut state: Battery, action: &TimeDelta) -> Result<Option<Battery>, Self::DynamicsError> {
            state.0 += action.0 * self.0;
            state.0 = state.0.max(1.0);
            Ok(Some(state))
        }
    }

    impl Dynamics<Battery, HybridAction> for ChargeBattery {
        type DynamicsError = NoError;
        fn advance(&self, mut state: Battery, action: &HybridAction) -> Result<Option<Battery>, Self::DynamicsError> {
            match action {
                HybridAction::Time(time) => {
                    state.0 += time.0 * self.0;
                    state.0 = state.0.min(1.0);
                }
                _ => { }
            }
            Ok(Some(state))
        }
    }

    #[test]
    fn test_lifted_dynamics() {
        // TODO(MXG): Figure out how to make action mapping work for dynamics
        let domain = DefineTrait::<TestState, HybridAction>::new()
            .lift(
                DefineDomainMap::for_subspace(BatterySubspace),
                ChargeBattery(0.2),
            )
            .chain_lift(
                DefineDomainMap::<TestState>::for_actions(
                    MaybeActionInto::<TimeDelta>::new()
                ),
                DefineTrait::<TestState, TimeDelta>::new()
                    .with(TimeBatteryDrain(0.1))
                    .chain(TimePassage),
            )
            .chain_lift(
                DefineDomainMap::with(
                    BatterySubspace,
                    MaybeActionInto::<Move>::new(),
                ),
                MoveBatteryDrain(0.02),
            );

        let initial_time = 0.1;
        let initial_battery = 0.3;
        let initial_state = TestState {
            time: initial_time,
            battery: initial_battery
        };

        let action = HybridAction::Time(TimeDelta(0.1));
        let next_state = domain.advance(initial_state.clone(), &action).unwrap().unwrap();
        let expected_time = initial_time + 0.1;
        let expected_battery = initial_battery + 0.1 * (0.2 - 0.1);
        assert_relative_eq!(next_state.battery, expected_battery);
        assert_relative_eq!(next_state.time, expected_time);

        let action = HybridAction::Move(Move(10.0));
        let next_state = domain.advance(next_state, &action).unwrap().unwrap();
        let expected_battery = expected_battery - 0.02 * 10.0;
        assert_relative_eq!(next_state.battery, expected_battery);
    }
}
