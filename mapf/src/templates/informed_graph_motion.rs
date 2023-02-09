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
        Connectable,
    },
    util::FlatResultMapTrait,
    error::Error,
};
use thiserror::Error as ThisError;

/// `InformedGraphMotion` template helps produce a domain for graph-based motion
/// planning that is compatible with the [`AStar`] and [`AStarConnect`] algorithms.
pub struct InformedGraphMotion<S, G, E, W, H, C, M> {
    pub space: S,
    pub graph: G,
    pub extrapolator: E,
    pub weight: W,
    pub heuristic: H,
    pub connector: C,
    pub modifier: M,
}

impl<S, G, E, W, H, C, M> Domain for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space,
    G: Graph,
    E: Extrapolator<S::Waypoint, G::Vertex, G::Edge>,
    W: Weighted<S::Waypoint, E::Extrapolation>,
    M: ActivityModifier<S::State, E::Extrapolation>,
{
    type State = S::State;
    type Action = M::ModifiedAction;
    type Error = anyhow::Error;
}

impl<S, G, E, W, H, C, M, Start> Initializable<Start, S::State> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space + Initializable<Start, S::State>,
    S::InitialError: Error,
{
    type InitialError = S::InitialError;
    type InitialStates<'a> = S::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        S::State: 'a,
        S: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        S::State: 'a,
    {
        self.space.initialize(from_start)
    }
}

impl<S, G, E, W, H, C, M> Closable<S::State> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space + Closable<S::State>,
{
    type ClosedSet<T> = S::ClosedSet<T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        self.space.new_closed_set()
    }
}

impl<S, G, E, W, H, C, M> Activity<S::State> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: PartialKeyedSpace<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone + 'static,
    G::Edge: Clone + 'static,
    E: Extrapolator<S::Waypoint, G::Vertex, G::Edge>,
    E::ExtrapolationError: Error,
    M: ActivityModifier<S::State, E::Extrapolation>,
    M::ModifiedActionError: Error,
{
    type ActivityAction = M::ModifiedAction;
    type ActivityError = InformedGraphMotionError<G::Key, E::ExtrapolationError, M::ModifiedActionError>;
    type Choices<'a> = impl IntoIterator<Item = Result<(Self::ActivityAction, S::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::Edge: 'a,
        S::State: 'a;

    fn choices<'a>(&'a self, from_state: S::State) -> Self::Choices<'a> {
        self
        .space
        .partial_key(&from_state)
        .into_iter()
        .flat_map(move |k| {
            let from_state = from_state.clone();
            self
            .graph
            .edges_from_vertex(k.clone())
            .into_iter()
            .flat_map(move |edge| {
                let to_vertex = edge.to_vertex().clone();
                let from_state = from_state.clone();
                let edge = edge.clone();
                self
                .graph
                .vertex(&to_vertex)
                .ok_or_else(|| InformedGraphMotionError::MissingVertex(to_vertex.clone()))
                .flat_result_map(move |v| {
                    let from_state = from_state.clone();
                    let to_vertex = to_vertex.clone();
                    self
                    .extrapolator
                    .extrapolate(&self.space.waypoint(&from_state), &v, &edge)
                    .map_err(InformedGraphMotionError::Extrapolator)
                    .flat_result_map(|r| r.into_iter())
                    .flat_map(move |r: Result<(E::Extrapolation, S::Waypoint), Self::ActivityError>| {
                        let from_state = from_state.clone();
                        let to_vertex = to_vertex.clone();
                        r
                        .flat_result_map(move |(action, waypoint)| {
                            let state = self.space.make_partial_keyed_state(
                                Some(to_vertex.clone()), waypoint
                            );
                            self
                            .modifier
                            .modify_action(from_state.clone(), action, state)
                            .into_iter()
                            .map(move |r: Result<(Self::ActivityAction, S::State), M::ModifiedActionError>| r.map_err(InformedGraphMotionError::Modifier))
                        })
                        .map(move |r: Result<Result<(Self::ActivityAction, S::State), Self::ActivityError>, Self::ActivityError>| r.flatten())
                    })
                })
                .map(move |r: Result<Result<(Self::ActivityAction, S::State), Self::ActivityError>, Self::ActivityError>| r.flatten())
            })
        })
    }
}

impl<S, G, E, W, H, C, M> Weighted<S::State, M::ModifiedAction> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space,
    G: Graph,
    E: Extrapolator<S::Waypoint, G::Vertex, G::Edge>,
    W: Weighted<S::Waypoint, M::ModifiedAction>,
    W::WeightedError: Error,
    M: ActivityModifier<S::State, E::Extrapolation>,
{
    type Cost = W::Cost;
    type WeightedError = W::WeightedError;
    fn cost(
        &self,
        from_state: &S::State,
        action: &M::ModifiedAction,
        to_state: &S::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.cost(&self.space.waypoint(from_state), action, &self.space.waypoint(to_state))
    }

    fn initial_cost(
        &self,
        for_state: &S::State,
    ) -> Result<Option<Self::Cost>, Self::WeightedError> {
        self.weight.initial_cost(&self.space.waypoint(for_state))
    }
}

impl<S, G, E, W, H, C, M, Goal> Informed<S::State, Goal> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space,
    H: Informed<S::Waypoint, Goal>,
    H::InformedError: Error,
{
    type CostEstimate = H::CostEstimate;
    type InformedError = H::InformedError;
    fn remaining_cost_estimate(
        &self,
        from_state: &S::State,
        to_goal: &Goal,
    ) -> Result<Option<Self::CostEstimate>, Self::InformedError> {
        self.heuristic.remaining_cost_estimate(&self.space.waypoint(from_state), to_goal)
    }
}

impl<S, G, E, W, H, C, M, Goal> Satisfiable<S::State, Goal> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: Space + Satisfiable<S::State, Goal>,
    S::SatisfactionError: Error,
{
    type SatisfactionError = S::SatisfactionError;
    fn is_satisfied(
        &self,
        by_state: &S::State,
        for_goal: &Goal,
    ) -> Result<bool, Self::SatisfactionError> {
        self.space.is_satisfied(by_state, for_goal)
    }
}

impl<S, G, E, W, H, C, M, Goal> Connectable<S::State, Goal> for InformedGraphMotion<S, G, E, W, H, C, M>
where
    S: PartialKeyedSpace<G::Key> + 'static,
    S::State: Clone,
    G: Graph + 'static,
    G::Key: 'static,
    E: Extrapolator<S::Waypoint, G::Vertex, G::Edge> + 'static,
    C: Connectable<S::State, Goal> + 'static,
    C::Connection: Into<E::Extrapolation>,
    C::ConnectionError: Error,
    M: ActivityModifier<S::State, E::Extrapolation> + 'static,
    M::ModifiedActionError: Error,
    H: 'static,
    W: 'static,
{
    type Connection = M::ModifiedAction;
    type ConnectionError = InformedGraphMotionError<G::Key, C::ConnectionError, M::ModifiedActionError>;
    type Connections<'a> = impl IntoIterator<Item=Result<(Self::Connection, S::State), Self::ConnectionError>> + 'a
    where
        Self::Connection: 'a,
        Self::ConnectionError: 'a,
        S::State: 'a,
        Goal: 'a;

    fn connect<'a>(
        &'a self,
        from_state: S::State,
        to_target: &'a Goal,
    ) -> Self::Connections<'a>
    where
        Self::Connection: 'a,
        Self::ConnectionError: 'a,
        S::State: 'a,
        Goal: 'a,
    {
        self
        .connector
        .connect(from_state.clone(), to_target)
        .into_iter()
        .map(|r| r.map_err(InformedGraphMotionError::Extrapolator))
        .flat_map(move |r| {
            let from_state = from_state.clone();
            r.flat_result_map(move |(action, child_state)|
                self
                .modifier
                .modify_action(from_state.clone(), action.into(), child_state)
                .into_iter()
                .map(|r| r.map_err(InformedGraphMotionError::Modifier))
            )
            .map(|x| x.flatten())
        })
    }
}

#[derive(ThisError, Debug)]
pub enum InformedGraphMotionError<K, E, M> {
    #[error("The graph is missing the requested vertex [{0}]")]
    MissingVertex(K),
    #[error("The extrapolator experienced an error:\n{0}")]
    Extrapolator(E),
    #[error("The action modifier experienced an error:\n{0}")]
    Modifier(M),
}

#[cfg(test)]
mod tests {
    #[test]
    fn motion_planning_se2() {

    }
}
