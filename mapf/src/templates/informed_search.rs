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
    domain::{
        Activity, ArrivalKeyring, AsTimeInvariant, AsTimeVariant, Backtrack, Chained, Closable,
        Connectable, Domain, Informed, Initializable, Keyed, Keyring, PartialKeyed, Reversible,
        Satisfiable, Weighted,
    },
    error::{Anyhow, ThisError},
};

/// `InformedSearch` template helps produce a domain for graph-based motion
/// planning that is compatible with the [`AStar`] and [`AStarConnect`] algorithms.
///
/// Fill in this domain with the relevant components and then it can be passed
/// into AStar, AStarConnect, or any other algorithm whose traits it satisfies.
#[derive(Debug, Clone)]
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

impl<A, W, H, X> InformedSearch<A, W, H, X, (), (), ()> {
    /// Create a new InformedSearch domain with the minimum required
    /// components.
    pub fn new(activity: A, weight: W, heuristic: H, closer: X) -> Self {
        Self {
            activity,
            weight,
            heuristic,
            closer,
            initializer: (),
            satisfier: (),
            connector: (),
        }
    }
}

impl<A, W, H, X, S, C> InformedSearch<A, W, H, X, (), S, C> {
    pub fn with_initializer<I>(self, initializer: I) -> InformedSearch<A, W, H, X, I, S, C> {
        InformedSearch {
            initializer,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            satisfier: self.satisfier,
            connector: self.connector,
        }
    }
}

impl<A, W, H, X, I, C> InformedSearch<A, W, H, X, I, (), C> {
    pub fn with_satisfier<S>(self, satisfier: S) -> InformedSearch<A, W, H, X, I, S, C> {
        InformedSearch {
            satisfier,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            connector: self.connector,
        }
    }
}

impl<A, W, H, X, I, S> InformedSearch<A, W, H, X, I, S, ()> {
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

impl<A, W, H, X, I, S, C> InformedSearch<A, W, H, X, I, S, C> {
    /// Consume this informed search, modify its activity, and return a new
    /// informed search with the modified activity.
    pub fn map_activity<A2, F: FnOnce(A) -> A2>(
        self,
        f: F,
    ) -> InformedSearch<A2, W, H, X, I, S, C> {
        InformedSearch {
            activity: f(self.activity),
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            satisfier: self.satisfier,
            connector: self.connector,
        }
    }
}

impl<A, W, H, X, I, S, C> AsTimeInvariant for InformedSearch<A, W, H, X, I, S, C>
where
    X: AsTimeInvariant,
{
    type TimeInvariantClosable = InformedSearch<A, W, H, X::TimeInvariantClosable, I, S, C>;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable {
        InformedSearch {
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer.as_time_invariant(),
            initializer: self.initializer,
            satisfier: self.satisfier,
            connector: self.connector,
        }
    }
}

impl<A, W, H, X, I, S, C> AsTimeVariant for InformedSearch<A, W, H, X, I, S, C>
where
    X: AsTimeVariant,
{
    type TimeVariantClosable = InformedSearch<A, W, H, X::TimeVariantClosable, I, S, C>;
    fn as_time_variant(self) -> Self::TimeVariantClosable {
        InformedSearch {
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer.as_time_variant(),
            initializer: self.initializer,
            satisfier: self.satisfier,
            connector: self.connector,
        }
    }
}

impl<A, W, H, X, I, S, C> InformedSearch<A, W, H, X, I, S, C> {
    /// Replace the initializer of the domain
    pub fn replace_initializer<I2>(
        self,
        new_initializer: I2,
    ) -> InformedSearch<A, W, H, X, I2, S, C> {
        InformedSearch {
            initializer: new_initializer,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            satisfier: self.satisfier,
            connector: self.connector,
        }
    }

    /// Replace the satisfier of the domain
    pub fn replace_satisfier<S2>(self, new_satisfier: S2) -> InformedSearch<A, W, H, X, I, S2, C> {
        InformedSearch {
            satisfier: new_satisfier,
            activity: self.activity,
            weight: self.weight,
            heuristic: self.heuristic,
            closer: self.closer,
            initializer: self.initializer,
            connector: self.connector,
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

    /// Add another goal connector to the domain
    pub fn chain_connector<C2>(
        self,
        connector2: C2,
    ) -> InformedSearch<A, W, H, X, I, S, Chained<C, C2>> {
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
}

impl<A: Domain, W, H, X, I, S, C> Domain for InformedSearch<A, W, H, X, I, S, C> {
    type State = A::State;
    type Error = anyhow::Error;
}

impl<A, W, H, X, I, S, C, Start, Goal> Initializable<Start, Goal, A::State>
    for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain,
    I: Initializable<Start, Goal, A::State>,
{
    type InitialError = I::InitialError;
    type InitialStates<'a>
        = I::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        A::State: 'a,
        A: 'a;

    fn initialize<'a>(&'a self, from_start: Start, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        A::State: 'a,
    {
        self.initializer.initialize(from_start, to_goal)
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
    type Action = A::Action;
    type ActivityError = A::ActivityError;
    type Choices<'a>
        =
        impl IntoIterator<Item = Result<(Self::Action, A::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        A::State: 'a;

    fn choices<'a>(&'a self, from_state: A::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        A::State: 'a,
    {
        self.activity.choices(from_state)
    }
}

impl<A, W, H, X, I, S, C> Weighted<A::State, A::Action>
    for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State>,
    W: Weighted<A::State, A::Action>,
    W::WeightedError: Into<Anyhow>,
{
    type Cost = W::Cost;
    type WeightedError = W::WeightedError;
    fn cost(
        &self,
        from_state: &A::State,
        action: &A::Action,
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
    H::InformedError: Into<Anyhow>,
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
    S::SatisfactionError: Into<Anyhow>,
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

impl<A, W, H, X, I, S, C, Goal> Connectable<A::State, A::Action, Goal>
    for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State>,
    A::State: Clone,
    C: Connectable<A::State, A::Action, Goal>,
    C::ConnectionError: Into<Anyhow>,
{
    type ConnectionError = anyhow::Error;
    type Connections<'a>
        =
        impl IntoIterator<Item = Result<(A::Action, A::State), Self::ConnectionError>> + 'a
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        A::State: 'a,
        A::Action: 'a,
        Goal: 'a;

    fn connect<'a>(&'a self, from_state: A::State, to_target: &'a Goal) -> Self::Connections<'a>
    where
        Self::ConnectionError: 'a,
        A::State: 'a,
        A::Action: 'a,
        Goal: 'a,
    {
        self.connector
            .connect(from_state.clone(), to_target)
            .into_iter()
            .map(|r| r.map_err(Into::into))
    }
}

impl<A: Keyed, W, H, X, I, S, C> Keyed for InformedSearch<A, W, H, X, I, S, C> {
    type Key = A::Key;
}

impl<A: PartialKeyed, W, H, X, I, S, C> PartialKeyed for InformedSearch<A, W, H, X, I, S, C> {
    type PartialKey = A::PartialKey;
}

impl<A, W, H, X, I, S, C> Keyring<A::State> for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Keyring<A::State>,
{
    type KeyRef<'a>
        = A::KeyRef<'a>
    where
        Self: 'a,
        A::State: 'a;

    fn key_for<'a>(&'a self, state: &'a A::State) -> Self::KeyRef<'a>
    where
        Self: 'a,
        A::State: 'a,
    {
        self.activity.key_for(state)
    }
}

impl<A, W, H, X, I, S, C, Start, Goal> ArrivalKeyring<A::Key, Start, Goal>
    for InformedSearch<A, W, H, X, I, S, C>
where
    A: Keyed,
    S: ArrivalKeyring<A::Key, Start, Goal>,
{
    type ArrivalKeyError = S::ArrivalKeyError;
    type ArrivalKeys<'a>
        = S::ArrivalKeys<'a>
    where
        Self: 'a,
        Start: 'a,
        Goal: 'a,
        S::ArrivalKeyError: 'a,
        A::Key: 'a;

    fn get_arrival_keys<'a>(&'a self, start: &Start, goal: &Goal) -> Self::ArrivalKeys<'a>
    where
        Self: 'a,
        Self::ArrivalKeyError: 'a,
        A::Key: 'a,
        Start: 'a,
        Goal: 'a,
    {
        self.satisfier.get_arrival_keys(start, goal)
    }
}

impl<A, W, H, X, I, S, C> Reversible for InformedSearch<A, W, H, X, I, S, C>
where
    A: Reversible,
    W: Reversible,
    H: Reversible,
    C: Reversible,
    X: Reversible,
    I: Reversible,
    S: Reversible,
{
    type ReversalError = InformedSearchReversalError<
        A::ReversalError,
        W::ReversalError,
        H::ReversalError,
        X::ReversalError,
        I::ReversalError,
        S::ReversalError,
        C::ReversalError,
    >;

    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(InformedSearch {
            activity: self
                .activity
                .reversed()
                .map_err(InformedSearchReversalError::Activity)?,
            weight: self
                .weight
                .reversed()
                .map_err(InformedSearchReversalError::Weight)?,
            heuristic: self
                .heuristic
                .reversed()
                .map_err(InformedSearchReversalError::Heuristic)?,
            connector: self
                .connector
                .reversed()
                .map_err(InformedSearchReversalError::Connector)?,
            closer: self
                .closer
                .reversed()
                .map_err(InformedSearchReversalError::Closer)?,
            initializer: self
                .initializer
                .reversed()
                .map_err(InformedSearchReversalError::Initializer)?,
            satisfier: self
                .satisfier
                .reversed()
                .map_err(InformedSearchReversalError::Satisfier)?,
        })
    }
}

impl<A, W, H, X, I, S, C> Backtrack<A::State, A::Action>
    for InformedSearch<A, W, H, X, I, S, C>
where
    A: Domain + Activity<A::State> + Backtrack<A::State, A::Action>,
{
    type BacktrackError = A::BacktrackError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &A::State,
        final_reverse_state: &A::State,
    ) -> Result<(A::State, A::State), Self::BacktrackError> {
        self.activity
            .flip_endpoints(initial_reverse_state, final_reverse_state)
    }

    fn backtrack(
        &self,
        parent_forward_state: &A::State,
        parent_reverse_state: &A::State,
        reverse_action: &A::Action,
        child_reverse_state: &A::State,
    ) -> Result<(A::Action, A::State), Self::BacktrackError> {
        self.activity.backtrack(
            parent_forward_state,
            parent_reverse_state,
            reverse_action,
            child_reverse_state,
        )
    }
}

#[derive(Debug, ThisError)]
pub enum InformedSearchReversalError<A, W, H, X, I, S, C> {
    #[error("An error happened while reversing the activity:\n{0}")]
    Activity(A),
    #[error("An error happened while reversing the weight:\n{0}")]
    Weight(W),
    #[error("An error happened while reversing the heuristic:\n{0}")]
    Heuristic(H),
    #[error("An error happened while reversing the closer:\n{0}")]
    Closer(X),
    #[error("An error happened while reversing the initializer:\n{0}")]
    Initializer(I),
    #[error("An error happened while reversing the satisfier:\n{0}")]
    Satisfier(S),
    #[error("An error happened while reversing the connector:\n{0}")]
    Connector(C),
}

#[cfg(test)]
mod tests {
    #[test]
    fn motion_planning_se2() {}
}
