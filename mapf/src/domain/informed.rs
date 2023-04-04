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

pub trait Informed<State, Goal> {
    /// How is cost represented. E.g. f32, f64, or u64
    type CostEstimate;

    /// What kind of error can happen if a bad state is provided
    type InformedError;

    /// Calculate an estimate for what it will cost to move from `from_state` to
    /// `to_goal_state`. If it is know to be impossible to reach the goal from
    /// `from_state` then return Ok(None). If there is a problem with the input
    /// that makes it impossible to calculate any estimate, return Err(_).
    fn estimate_remaining_cost(
        &self,
        from_state: &State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError>;
}

pub trait EstimateModifier<State, Goal, Cost> {
    /// What kind of error can happen if a bad input is provided
    type EstimateModifierError;

    fn modify_remaining_cost_estimate(
        &self,
        from_state: &State,
        to_goal: &Goal,
        original_estimate: Cost,
    ) -> Result<Option<Cost>, Self::EstimateModifierError>;
}

impl<State, Goal, Cost> EstimateModifier<State, Goal, Cost> for ScaleWeight<Cost>
where
    Cost: std::ops::Mul<Cost, Output = Cost> + Clone,
{
    type EstimateModifierError = NoError;
    fn modify_remaining_cost_estimate(
        &self,
        _: &State,
        _: &Goal,
        original_estimate: Cost,
    ) -> Result<Option<Cost>, Self::EstimateModifierError> {
        Ok(Some(original_estimate * self.0.clone()))
    }
}

impl<Base, Prop, Goal> Informed<Base::State, Goal> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Informed<Base::State, Goal>,
    Prop::InformedError: Into<Base::Error>,
{
    type CostEstimate = Prop::CostEstimate;
    type InformedError = Base::Error;
    fn estimate_remaining_cost(
        &self,
        from_state: &Base::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        self.prop
            .estimate_remaining_cost(from_state, to_goal)
            .map_err(Into::into)
    }
}

impl<Base, Prop, Goal> Informed<Base::State, Goal> for Chained<Base, Prop>
where
    Base: Domain + Informed<Base::State, Goal>,
    Base::InformedError: Into<Base::Error>,
    Prop: Informed<Base::State, Goal, CostEstimate = Base::CostEstimate>,
    Prop::InformedError: Into<Base::Error>,
    Base::CostEstimate: std::ops::Add<Base::CostEstimate, Output = Base::CostEstimate>,
{
    type CostEstimate = Base::CostEstimate;
    type InformedError = Base::Error;
    fn estimate_remaining_cost(
        &self,
        from_state: &Base::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let base_cost_estimate = self
            .base
            .estimate_remaining_cost(from_state, to_goal)
            .map_err(Into::into)?;
        let prop_cost_estimate = self
            .prop
            .estimate_remaining_cost(from_state, to_goal)
            .map_err(Into::into)?;

        let base_cost_estimate = match base_cost_estimate {
            Some(c) => c,
            None => return Ok(None),
        };
        let prop_cost_estimate = match prop_cost_estimate {
            Some(c) => c,
            None => return Ok(None),
        };

        Ok(Some(base_cost_estimate + prop_cost_estimate))
    }
}

impl<Base, Prop, Goal> Informed<Base::State, Goal> for Mapped<Base, Prop>
where
    Base: Domain + Informed<Base::State, Goal>,
    Base::InformedError: Into<Base::Error>,
    Prop: EstimateModifier<Base::State, Goal, Base::CostEstimate>,
    Prop::EstimateModifierError: Into<Base::Error>,
{
    type CostEstimate = Base::CostEstimate;
    type InformedError = Base::Error;
    fn estimate_remaining_cost(
        &self,
        from_state: &Base::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let original_estimate = match self
            .base
            .estimate_remaining_cost(from_state, to_goal)
            .map_err(Into::into)?
        {
            Some(c) => c,
            None => return Ok(None),
        };

        self.prop
            .modify_remaining_cost_estimate(from_state, to_goal, original_estimate)
            .map_err(Into::into)
    }
}

impl<Base, Lifter, Prop, Goal> Informed<Base::State, Goal> for Lifted<Base, Lifter, Prop>
where
    Base: Domain,
    Base::State: Clone,
    Lifter: ProjectState<Base::State>,
    Lifter::ProjectionError: Into<Base::Error>,
    Prop: Informed<Lifter::ProjectedState, Goal>,
    Prop::InformedError: Into<Base::Error>,
    Prop::CostEstimate: std::ops::Add<Prop::CostEstimate, Output = Prop::CostEstimate>,
{
    type CostEstimate = Prop::CostEstimate;
    type InformedError = Base::Error;
    fn estimate_remaining_cost(
        &self,
        from_state: &Base::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        let from_state_proj = match self.lifter.project(from_state).map_err(Into::into)? {
            Some(s) => s,
            None => return Ok(None),
        };

        self.prop
            .estimate_remaining_cost(&from_state_proj, &to_goal)
            .map_err(Into::into)
    }
}

#[cfg(test)]
mod tests {
    use super::weighted::tests::*;
    use super::*;
    use crate::error::NoError;
    use approx::assert_relative_eq;

    struct EuclideanDistanceEstimate;
    impl<State: Mobile, Goal: Mobile> Informed<State, Goal> for EuclideanDistanceEstimate {
        type CostEstimate = f64;
        type InformedError = NoError;
        fn estimate_remaining_cost(
            &self,
            from_state: &State,
            to_goal: &Goal,
        ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
            Ok(Some((from_state.position() - to_goal.position()).norm()))
        }
    }

    struct BatteryLevelCostEstimate;
    impl<State: BatteryPowered, Goal> Informed<State, Goal> for BatteryLevelCostEstimate {
        type CostEstimate = f64;
        type InformedError = NoError;
        fn estimate_remaining_cost(
            &self,
            from_state: &State,
            _: &Goal,
        ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
            if from_state.battery_level() <= 0.0 {
                return Ok(None);
            }

            // We penalize a low battery value
            Ok(Some(1.0 / from_state.battery_level()))
        }
    }

    #[test]
    fn test_cost_estimate() {
        let domain = DefineTrait::<TestState>::new()
            .with(EuclideanDistanceEstimate)
            .map(ScaleWeight(0.1));

        let from_state = TestState {
            position: Point::new(0.0, 0.0),
            battery: 1.0,
        };
        let to_goal_state = TestState {
            position: Point::new(10.0, 0.0),
            battery: 0.5,
        };

        let cost_estimate = domain
            .estimate_remaining_cost(&from_state, &to_goal_state)
            .unwrap()
            .unwrap();
        assert_relative_eq!(cost_estimate, 10.0 * 0.1);
    }

    #[test]
    fn test_lifted_cost_estimate() {
        let domain = DefineTrait::<TestState>::new()
            .lift(
                DefineDomainMap::for_subspace(StateInto::<Point>::new()),
                DefineTrait::<Point>::new()
                    .with(EuclideanDistanceEstimate)
                    .map(ScaleWeight(0.1)),
            )
            .chain_lift(
                DefineDomainMap::for_subspace(StateInto::<Battery>::new()),
                DefineTrait::<Battery>::new()
                    .with(BatteryLevelCostEstimate)
                    .map(ScaleWeight(0.2)),
            );

        let from_state = TestState {
            position: Point::new(0.0, 0.0),
            battery: 0.35,
        };
        let to_goal_state = TestState {
            position: Point::new(10.0, 0.0),
            battery: 0.25,
        };

        let cost_estimate = domain
            .estimate_remaining_cost(&from_state, &to_goal_state)
            .unwrap()
            .unwrap();
        assert_relative_eq!(cost_estimate, 10.0 * 0.1 + 1.0 / 0.35 * 0.2);
    }
}
