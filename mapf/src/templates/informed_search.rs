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
        Satisfiable, Closable, Space, Initializable,
        Connectable, Chained,
    },
    util::FlatResultMapTrait,
    error::Error,
};
use thiserror::Error as ThisError;

/// `InformedSearch` template helps produce a domain for graph-based motion
/// planning that is compatible with the [`AStar`] and [`AStarConnect`] algorithms.
///
/// Fill in this domain with the relevant components and then it can be passed
/// into AStar, AStarConnect, or any other algorithm whose traits it satisfies.
pub struct InformedSearch<A, W, H, X, I, S, C> {
    /// Describe what kind of activity is being searched. This field must
    /// implement [`Activity`] and [`Domain`].
    ///
    /// The following templates can be used here:
    /// * [`crate::templates::graph_motion::GraphMotion`]
    pub activity: A,
    /// Calculate the cost of each choice that is searched. This field must
    /// implement [`Weighted<A::State, A::Action>`].
    pub weight: W,
    /// Calculate an estimate of the remaining cost from a state. This field
    /// must implemented [`Informed<A::State, Goal>`].
    pub heuristic: H,
    /// Provide the [`ClosedSet`] that should be used during the search. This
    /// field must implement [`Closable<A::State>`].
    pub closer: X,
    /// Determine what kind of Start conditions the search can accept. This
    /// field must implement [`Initializable<Start, A::State>`]. By default this
    /// field will be an empty tuple `()`, which means the `Start` type must
    /// match `A::State`.
    pub initializer: I,
    /// Determine when the search is solved/satisfied and what type of `Goal`
    /// conditions the search can accept. This field must implement
    /// [`Satisfiable<A::State, Goal>`]. By default this field will be an empty
    /// tuple `()`, which means the `Goal` type must match `A::State` and the
    /// search will be satisfied when a candidate state matches the goal state
    /// using the `PartialEq` trait.
    pub satisfier: S,
    /// Provide an action to connect search states directly to the goal. This
    /// can be helpful or even necessary for search spaces where simply
    /// expanding the usual `activity` is not guaranteed to directly reach the
    /// goal. This field must implement [`Connectable<A::State, A::Action, Goal>`].
    /// By default this field will be an empty tuple `()` which means there will
    /// be no attempt to connect to the goal.
    pub connector: C,
}

impl<A: Domain, W, H, X> InformedSearch<A, W, H, X, (), (), ()> {
    /// Create a new InformedSearch domain with the minimum required
    /// components.
    pub fn new(
        motion: A,
        weight: W,
        heuristic: H,
        closer: X,
    ) -> Self {
        Self {
            activity: motion,
            weight,
            heuristic,
            closer,
            initializer: (),
            satisfier: (),
            connector: (),
        }
    }
}

impl<A: Domain, W, H, X, I, S> InformedSearch<A, W, H, X, I, S, ()> {
    /// Give a goal connector to the InformedSearch
    pub fn with_connector<C>(self, connector: C) -> InformedSearch<A, W, H, X, I, S, C> {
        InformedSearch {
            connector,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            satisfier: self.satisfier,
        }
    }
}

impl<A, W, H, X, I, S, C1> InformedSearch<A, W, H, X, I, S, C1> {
    /// Add another goal connector to the domain
    pub fn chain_connector<C2>(self, connector2: C2) -> InformedSearch<A, W, H, X, I, S, Chained<C1, C2>> {
        InformedSearch {
            connector: Chained {
                base: self.connector,
                prop: connector2,
            },
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            satisfier: self.satisfier,
        }
    }

    /// Replace the goal connector of the domain
    pub fn replace_connector<C2>(self, new_connector: C2) -> InformedSearch<A, W, H, X, I, S, C2> {
        InformedSearch {
            connector: new_connector,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            satisfier: self.satisfier,
        }
    }
}

impl<A: Domain, W, H, X, I, S, C> Domain for InformedSearch<A, W, H, X, I, S, C> {
    type State = A::State;
    type Error = anyhow::Error;
}

impl<A, W, H, X, I, S, C, Start> Initializable<Start, A::State> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain,
    I: Initializable<Start, A::State>,
{
    type InitialError = I::InitialError;
    type InitialStates<'a> = I::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        A::State: 'a,
        A: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        A::State: 'a,
    {
        self.initializer.initialize(from_start)
    }
}

impl<A, W, H, X, I, S, C> Closable<A::State> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain,
    X: Closable<A::State>,
{
    type ClosedSet<T> = X::ClosedSet<T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        self.closer.new_closed_set()
    }
}

impl<A, W, H, X, I, S, C> Activity<A::State> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State>,
{
    type ActivityAction = A::ActivityAction;
    type ActivityError = A::ActivityError;
    type Choices<'a> = impl IntoIterator<Item = Result<(Self::ActivityAction, A::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        A::State: 'a;

    fn choices<'a>(&'a self, from_state: A::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        A::State: 'a,
    {
        self.activity.choices(from_state)
    }
}

impl<A, W, H, X, I, S, C> Weighted<A::State, A::ActivityAction> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State>,
    W: Weighted<A::State, A::ActivityAction>,
    W::WeightedError: Error,
{
    type Cost = W::Cost;
    type WeightedError = W::WeightedError;
    fn cost(
        &self,
        from_state: &A::State,
        action: &A::ActivityAction,
        to_state: &A::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.cost(from_state, action, to_state)
    }

    fn initial_cost(
        &self,
        for_state: &A::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.initial_cost(for_state)
    }
}

impl<A, W, H, X, I, S, C, Goal> Informed<A::State, Goal> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain,
    H: Informed<A::State, Goal>,
    H::InformedError: Error,
{
    type CostEstimate = H::CostEstimate;
    type InformedError = H::InformedError;
    fn estimate_remaining_cost(
        &self,
        from_state: &A::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        self.heuristic.estimate_remaining_cost(from_state, to_goal)
    }
}

impl<A, W, H, X, I, S, C, Goal> Satisfiable<A::State, Goal> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain,
    S: Satisfiable<A::State, Goal>,
    S::SatisfactionError: Error,
{
    type SatisfactionError = S::SatisfactionError;
    fn is_satisfied(
        &self,
        by_state: &A::State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError> {
        self.satisfier.is_satisfied(by_state, for_goal)
    }
}

impl<A, W, H, X, I, S, C, Goal> Connectable<A::State, A::ActivityAction, Goal> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State> + 'static,
    W: 'static,
    H: 'static,
    X: 'static,
    I: 'static,
    S: 'static,
    A::State: Clone,
    C: Connectable<A::State, A::ActivityAction, Goal> + 'static,
    C::ConnectionError: Error,
{
    type ConnectionError = anyhow::Error;
    type Connections<'a> = impl IntoIterator<Item=Result<(A::ActivityAction, A::State), Self::ConnectionError>> + 'a
    where
        Self::ConnectionError: 'a,
        A::State: 'a,
        A::ActivityAction: 'a,
        Goal: 'a;

    fn connect<'a>(
        &'a self,
        from_state: A::State,
        to_target: &'a Goal,
    ) -> Self::Connections<'a>
    where
        Self::ConnectionError: 'a,
        A::State: 'a,
        A::ActivityAction: 'a,
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
