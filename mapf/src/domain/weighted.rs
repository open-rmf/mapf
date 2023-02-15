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
use crate::error::NoError;
use num::traits::Zero;

/// The `Weighted` trait describes how to calculate the cost of an action.
pub trait Weighted<State, Action> {
    /// How is cost represented. E.g. f32, f64, or u64
    type Cost;

    /// What kind of error can happen if a bad state or activity is provided
    type WeightedError;

    /// Calculate the cost for performing `action` which transitions `from_state`
    /// to `to_state`.
    ///
    /// When cost returns `None` that implies the state is too expensive for
    /// consideration.
    fn cost(
        &self,
        from_state: &State,
        action: &Action,
        to_state: &State
    ) -> Result<Option<Self::Cost>, Self::WeightedError>;

    /// Calculate the cost of an initial state.
    fn initial_cost(
        &self,
        for_state: &State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError>;
}

/// The `CostModifier` trait can be used with `.map` to modify the output of a
/// `Weighted` trait.
pub trait CostModifier<State, Action, Cost> {
    /// What kind of error can happen if a bad input is provided
    type CostModifierError;

    /// Modified the incoming cost. The output of this function will be used
    /// as the cost instead of the input.
    fn modify_cost(
        &self,
        from_state: &State,
        action: &Action,
        to_state: &State,
        original_cost: Cost,
    ) -> Result<Option<Cost>, Self::CostModifierError>;

    /// Modify the cost of an initial state. By default this does not modify
    /// anything. Initial state costs are usually zero anyway so this modifier
    /// is typically not needed.
    fn modify_initial_cost(
        &self,
        _: &State,
        original_cost: Cost,
    ) -> Result<Option<Cost>, Self::CostModifierError> {
        Ok(Some(original_cost))
    }
}

/// Implements CostModifier for simple proportional scaling of cost calculation.
///
/// Apply this to a Weighted property using `.map(ScaleWeight(scale))`.
pub struct ScaleWeight<Cost: std::ops::Mul<Cost, Output=Cost> + Clone>(pub Cost);
impl<State, Action, Cost> CostModifier<State, Action, Cost> for ScaleWeight<Cost>
where
    Cost: std::ops::Mul<Cost, Output=Cost> + Clone
{
    type CostModifierError = NoError;
    fn modify_cost(
        &self,
        _: &State,
        _: &Action,
        _: &State,
        original_cost: Cost,
    ) -> Result<Option<Cost>, Self::CostModifierError> {
        Ok(Some(original_cost * self.0.clone()))
    }
}

impl<Base, Prop, Action> Weighted<Base::State, Action> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Weighted<Base::State, Action>,
    Prop::WeightedError: Into<Base::Error>,
{
    type Cost = Prop::Cost;
    type WeightedError = Base::Error;
    fn cost(
        &self,
        from_state: &Base::State,
        action: &Action,
        to_state: &Base::State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.prop.cost(from_state, action, to_state).map_err(Into::into)
    }

    fn initial_cost(
        &self,
        for_state: &Base::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.prop.initial_cost(for_state).map_err(Into::into)
    }
}

impl<Base, Prop, Action> Weighted<Base::State, Action> for Chained<Base, Prop>
where
    Base: Domain + Weighted<Base::State, Action>,
    Base::WeightedError: Into<Base::Error>,
    Prop: Weighted<Base::State, Action, Cost=Base::Cost>,
    Prop::WeightedError: Into<Base::Error>,
    Base::Cost: std::ops::Add<Base::Cost, Output=Base::Cost>,
{
    type Cost = Base::Cost;
    type WeightedError = Base::Error;
    fn cost(
        &self,
        from_state: &Base::State,
        action: &Action,
        to_state: &Base::State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let base_cost = self.base.cost(from_state, action, to_state)
            .map_err(Into::into)?;
        let prop_cost = self.prop.cost(from_state, action, to_state)
            .map_err(Into::into)?;

        let base_cost = match base_cost {
            Some(c) => c,
            None => return Ok(None),
        };
        let prop_cost = match prop_cost {
            Some(c) => c,
            None => return Ok(None),
        };

        Ok(Some(base_cost + prop_cost))
    }

    fn initial_cost(
        &self,
        for_state: &Base::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let base_cost = self.base.initial_cost(for_state).map_err(Into::into)?;
        let prop_cost = self.prop.initial_cost(for_state).map_err(Into::into)?;

        let base_cost = match base_cost {
            Some(c) => c,
            None => return Ok(None),
        };
        let prop_cost = match prop_cost {
            Some(c) => c,
            None => return Ok(None),
        };

        Ok(Some(base_cost + prop_cost))
    }
}

impl<Base, Prop, Action> Weighted<Base::State, Action> for Mapped<Base, Prop>
where
    Base: Domain + Weighted<Base::State, Action>,
    Base::WeightedError: Into<Base::Error>,
    Prop: CostModifier<Base::State, Action, Base::Cost>,
    Prop::CostModifierError: Into<Base::Error>,
{
    type Cost = Base::Cost;
    type WeightedError = Base::Error;
    fn cost(
        &self,
        from_state: &Base::State,
        action: &Action,
        to_state: &Base::State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let base_cost = match self.base.cost(
            from_state, action, to_state
        ).map_err(Into::into)? {
            Some(base_cost) => base_cost,
            None => return Ok(None),
        };

        self.prop.modify_cost(from_state, action, to_state, base_cost)
            .map_err(Into::into)
    }

    fn initial_cost(
        &self,
        for_state: &Base::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let base_cost = match self.base.initial_cost(for_state).map_err(Into::into)? {
            Some(base_cost) => base_cost,
            None => return Ok(None),
        };

        self.prop.modify_initial_cost(for_state, base_cost)
            .map_err(Into::into)
    }
}

impl<Base, Lifter, Prop, Action> Weighted<Base::State, Action> for Lifted<Base, Lifter, Prop>
where
    Base: Domain,
    Base::State: Clone,
    Action: Clone,
    Lifter: ProjectState<Base::State> + ActionMap<Base::State, Action>,
    Lifter::ActionMapError: Into<Base::Error>,
    Lifter::ProjectionError: Into<Base::Error>,
    Prop: Weighted<Lifter::ProjectedState, Lifter::ToAction>,
    Prop::WeightedError: Into<Base::Error>,
    Prop::Cost: std::ops::Add<Prop::Cost, Output=Prop::Cost> + Zero,
{
    type Cost = Prop::Cost;
    type WeightedError = Base::Error;
    fn cost(
        &self,
        from_state: &Base::State,
        action: &Action,
        to_state: &Base::State
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let from_state_proj = match self.lifter.project(from_state)
            .map_err(Into::into)? {
            Some(s) => s,
            None => return Ok(None),
        };

        let to_state_proj = match self.lifter.project(to_state)
            .map_err(Into::into)? {
            Some(s) => s,
            None => return Ok(None),
        };

        let mut cost = Prop::Cost::zero();
        let actions = self.lifter.map_action(
            from_state.clone(), action.clone()
        ).into_iter();
        for action in actions {
            let action = action.map_err(Into::into)?;
            let additional_cost = match self.prop.cost(
                &from_state_proj, &action, &to_state_proj
            ).map_err(Into::into)? {
                Some(c) => c,
                None => return Ok(None),
            };

            cost = cost + additional_cost;
        }

        Ok(Some(cost))
    }

    fn initial_cost(
        &self,
        for_state: &Base::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        let for_state_proj = match self.lifter.project(for_state)
            .map_err(Into::into)? {
            Some(s) => s,
            None => return Ok(None),
        };

        self.prop.initial_cost(&for_state_proj).map_err(Into::into)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::error::NoError;
    use approx::assert_relative_eq;

    pub(crate) type Point = nalgebra::Point2<f64>;

    pub(crate) trait Mobile {
        fn position(&self) -> Point;
        fn distance_traveled(&self, from_other: &Self) -> f64;
    }

    pub(crate) trait BatteryPowered {
        fn battery_level(&self) -> f64;
    }

    #[derive(Clone, Copy)]
    pub(crate) struct TestState {
        pub(crate) position: Point,
        pub(crate) battery: f64,
    }

    impl From<TestState> for Point {
        fn from(value: TestState) -> Self {
            value.position
        }
    }

    pub(crate) struct Battery(f64);
    impl From<TestState> for Battery {
        fn from(value: TestState) -> Self {
            Battery(value.battery)
        }
    }

    impl Mobile for TestState {
        fn position(&self) -> Point {
            self.position
        }

        fn distance_traveled(&self, from_other: &Self) -> f64 {
            (self.position - from_other.position).norm()
        }
    }

    impl Mobile for Point {
        fn position(&self) -> Point {
            *self
        }

        fn distance_traveled(&self, from_other: &Self) -> f64 {
            (*self - *from_other).norm()
        }
    }

    impl BatteryPowered for TestState {
        fn battery_level(&self) -> f64 {
            self.battery
        }
    }

    impl BatteryPowered for Battery {
        fn battery_level(&self) -> f64 {
            self.0
        }
    }

    struct DistanceWeight(f64 /* cost per meter */);
    impl<State: Mobile, Action> Weighted<State, Action> for DistanceWeight {
        type Cost = f64;
        type WeightedError = NoError;
        fn cost(
            &self,
            from_state: &State,
            _: &Action,
            to_state: &State
        ) -> Result<Option<Self::Cost>, Self::WeightedError> {
            Ok(Some(to_state.distance_traveled(from_state) * self.0))
        }

        fn initial_cost(
            &self,
            for_state: &State,
        ) -> Result<Option<Self::Cost>, Self::WeightedError> {
            Ok(Some(0.0))
        }
    }

    struct BatteryLossWeight(f64 /* cost per battery loss */);
    impl<State: BatteryPowered, Action> Weighted<State, Action> for BatteryLossWeight {
        type Cost = f64;
        type WeightedError = NoError;
        fn cost(
            &self,
            from_state: &State,
            _: &Action,
            to_state: &State
        ) -> Result<Option<Self::Cost>, Self::WeightedError> {
            if to_state.battery_level() < 0.0 {
                return Ok(None);
            }

            Ok(Some((from_state.battery_level() - to_state.battery_level()) * self.0))
        }

        fn initial_cost(
            &self,
            for_state: &State,
        ) -> Result<Option<Self::Cost>, Self::WeightedError> {
            Ok(Some(0.0))
        }
    }

    #[test]
    fn test_cost_calculation() {
        let domain = DefineTrait::<TestState>::new()
            .with(
                DefineTrait::<TestState>::new()
                    .with(DistanceWeight(0.1))
                    .map(ScaleWeight(2.0))
            )
            .chain(
                DefineTrait::<TestState>::new()
                    .with(BatteryLossWeight(10.0))
                    .map(ScaleWeight(3.0))
            );

        let from_state = TestState {
            position: Point::new(0.0, 0.0),
            battery: 1.0,
        };
        let to_state = TestState {
            position: Point::new(10.0, 0.0),
            battery: 0.5,
        };

        let cost = domain.cost(&from_state, &(), &to_state).unwrap().unwrap();
        assert_relative_eq!(cost, 10.0*0.1*2.0 + 0.5*10.0*3.0);
    }

    #[test]
    fn test_lifted_weight_calculation() {
        let domain = DefineTrait::<TestState>::new()
            .lift(
                DefineDomainMap::for_subspace(StateInto::<Point>::new()),
                DefineTrait::<Point>::new()
                    .with(DistanceWeight(0.1))
                    .map(ScaleWeight(2.0))
            )
            .chain_lift(
                DefineDomainMap::for_subspace(StateInto::<Battery>::new()),
                DefineTrait::<Battery>::new()
                    .with(BatteryLossWeight(10.0))
                    .map(ScaleWeight(3.0))
            );

        let from_state = TestState {
            position: Point::new(0.0, 0.0),
            battery: 1.0,
        };
        let to_state = TestState {
            position: Point::new(10.0, 0.0),
            battery: 0.5,
        };

        let cost = domain.cost(&from_state, &(), &to_state).unwrap().unwrap();
        assert_relative_eq!(cost, 10.0*0.1*2.0 + 0.5*10.0*3.0);
    }
}
