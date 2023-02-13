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
    Graph, graph::Edge,
    domain::{
        Domain, Informed, Extrapolator, Activity, ActivityModifier, Weighted,
        Satisfiable, Closable, Space, PartialKeyedSpace, Initializable,
        Connectable, Chained,
    },
    util::FlatResultMapTrait,
    error::Error,
};
use thiserror::Error as ThisError;

/// `InformedGraphMotion` template helps produce a domain for graph-based motion
/// planning that is compatible with the [`AStar`] and [`AStarConnect`] algorithms.
///
/// Fill in this domain with the relevant components and then it can be passed
/// into AStar, AStarConnect, or any other algorithm whose traits it satisfies.
pub struct InformedGraphMotion<M, W, H, C> {
    pub motion: M,
    pub weight: W,
    pub heuristic: H,
    pub connector: C,
}

impl<M: Domain, W, H> InformedGraphMotion<M, W, H, ()> {
    /// Create a new InformedGraphMotion domain with the minimum required
    /// components.
    pub fn new(
        motion: M,
        weight: W,
        heuristic: H,
    ) -> Self {
        Self {
            motion,
            weight,
            heuristic,
            connector: (),
        }
    }
}

impl<M: Domain, W, H> InformedGraphMotion<M, W, H, ()> {
    /// Give a goal connector to the InformedGraphMotion
    pub fn with_connector<C>(self, connector: C) -> InformedGraphMotion<M, W, H, C> {
        InformedGraphMotion {
            connector,
            motion: self.motion,
            weight: self.weight,
            heuristic: self.heuristic,
        }
    }
}

impl<M, W, H, C1> InformedGraphMotion<M, W, H, C1> {
    /// Add another goal connector to the domain
    pub fn chain_connector<C2>(self, connector2: C2) -> InformedGraphMotion<M, W, H, Chained<C1, C2>> {
        InformedGraphMotion {
            connector: Chained {
                base: self.connector,
                prop: connector2,
            },
            motion: self.motion,
            weight: self.weight,
            heuristic: self.heuristic,
        }
    }

    /// Replace the goal connector of the domain
    pub fn replace_connector<C2>(self, new_connector: C2) -> InformedGraphMotion<M, W, H, C2> {
        InformedGraphMotion {
            connector: new_connector,
            motion: self.motion,
            weight: self.weight,
            heuristic: self.heuristic,
        }
    }
}

impl<M: Domain, W, H, C> Domain for InformedGraphMotion<M, W, H, C> {
    type State = M::State;
    type Action = M::Action;
    type Error = anyhow::Error;
}

impl<M, W, H, C, Start> Initializable<Start, M::State> for InformedGraphMotion<M, W, H, C>
where
    M: Domain + Initializable<Start, M::State>,
    M::InitialError: Error,
{
    type InitialError = M::InitialError;
    type InitialStates<'a> = M::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        M::State: 'a,
        M: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        M::State: 'a,
    {
        self.motion.initialize(from_start)
    }
}

impl<M, W, H, C> Closable<M::State> for InformedGraphMotion<M, W, H, C>
where
    M: Domain + Closable<M::State>,
{
    type ClosedSet<T> = M::ClosedSet<T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        self.motion.new_closed_set()
    }
}

impl<M, W, H, C> Activity<M::State> for InformedGraphMotion<M, W, H, C>
where
    M: Domain + Activity<M::State>,
{
    type ActivityAction = M::ActivityAction;
    type ActivityError = M::ActivityError;
    type Choices<'a> = impl IntoIterator<Item = Result<(Self::ActivityAction, M::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        M::State: 'a;

    fn choices<'a>(&'a self, from_state: M::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        M::State: 'a,
    {
        self.motion.choices(from_state)
    }
}

impl<M, W, H, C> Weighted<M::State, M::Action> for InformedGraphMotion<M, W, H, C>
where
    M: Domain,
    W: Weighted<M::State, M::Action>,
    W::WeightedError: Error,
{
    type Cost = W::Cost;
    type WeightedError = W::WeightedError;
    fn cost(
        &self,
        from_state: &M::State,
        action: &M::Action,
        to_state: &M::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.cost(from_state, action, to_state)
    }

    fn initial_cost(
        &self,
        for_state: &M::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.initial_cost(for_state)
    }
}

impl<M, W, H, C, Goal> Informed<M::State, Goal> for InformedGraphMotion<M, W, H, C>
where
    M: Domain,
    H: Informed<M::State, Goal>,
    H::InformedError: Error,
{
    type CostEstimate = H::CostEstimate;
    type InformedError = H::InformedError;
    fn remaining_cost_estimate(
        &self,
        from_state: &M::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        self.heuristic.remaining_cost_estimate(from_state, to_goal)
    }
}

impl<M, W, H, C, Goal> Satisfiable<M::State, Goal> for InformedGraphMotion<M, W, H, C>
where
    M: Domain + Satisfiable<M::State, Goal>,
    M::SatisfactionError: Error,
{
    type SatisfactionError = M::SatisfactionError;
    fn is_satisfied(
        &self,
        by_state: &M::State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError> {
        self.motion.is_satisfied(by_state, for_goal)
    }
}

impl<M, W, H, C, Goal> Connectable<M::State, M::Action, Goal> for InformedGraphMotion<M, W, H, C>
where
    M: Domain + 'static,
    H: 'static,
    W: 'static,
    M::State: Clone,
    C: Connectable<M::State, M::Action, Goal> + 'static,
    C::ConnectionError: Error,
{
    type ConnectionError = anyhow::Error;
    type Connections<'a> = impl IntoIterator<Item=Result<(M::Action, M::State), Self::ConnectionError>> + 'a
    where
        Self::ConnectionError: 'a,
        M::State: 'a,
        M::Action: 'a,
        Goal: 'a;

    fn connect<'a>(
        &'a self,
        from_state: M::State,
        to_target: &'a Goal,
    ) -> Self::Connections<'a>
    where
        Self::ConnectionError: 'a,
        M::State: 'a,
        M::Action: 'a,
        Goal: 'a,
    {
        self
        .connector
        .connect(from_state.clone(), to_target)
        .into_iter()
        .map(|r| r.map_err(Into::into))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn motion_planning_se2() {

    }
}
