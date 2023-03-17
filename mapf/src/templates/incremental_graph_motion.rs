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
    graph::{Graph, Edge},
    domain::{
        Domain, IncrementalExtrapolator, Activity, Reversible,
        KeyedSpace, Keyed, PartialKeyed, Keyring,
    },
    templates::graph_motion::{GraphMotionError, GraphMotionReversalError},
    util::{FlatResultMapTrait, ForkIter},
    error::StdError,
};
use std::borrow::Borrow;

/// `IncrementalGraphMotion` defines a domain and activity that moves over a
/// spatial graph as thoroughly as possible. This is generally used for motion
/// planning problems that cache their search results.
///
/// The activity of `IncrementalGraphMotion` is meant to thoroughly cover every
/// state that the search is passing through. This leads to more search
/// iterations for the algorithm operating on this activity, but it also allows
/// more thorough caching of search results which may be desirable or even
/// necessary for some algorithms, like the Dijkstra algorithms.
///
/// For a more efficient activity representation, use [`super::GraphMotion`].
pub struct IncrementalGraphMotion<S, G, E> {
    pub space: S,
    pub graph: G,
    pub extrapolator: E,
}

impl<S, G, E> Domain for IncrementalGraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
{
    type State = IncrementalState<S::State, G>;
    type Error = GraphMotionError<G::Key, E::IncrementalExtrapolationError>;
}

impl<S, G, E> Activity<IncrementalState<S::State, G>> for IncrementalGraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone + 'static,
    G::EdgeAttributes: Clone + 'static,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
    E::IncrementalExtrapolationError: StdError,
{
    type ActivityAction = E::IncrementalExtrapolation;
    type ActivityError = GraphMotionError<G::Key, E::IncrementalExtrapolationError>;
    type Choices<'a> = impl IntoIterator<Item = Result<(Self::ActivityAction, IncrementalState<S::State, G>), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::EdgeAttributes: 'a;

    fn choices<'a>(&'a self, from_state: IncrementalState<S::State, G>) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        IncrementalState<S::State, G>: 'a,
    {
        if let Some((to_key, with_guidance)) = from_state.target.clone() {
            return ForkIter::Left(
                [self.graph.vertex(&to_key).ok_or_else(|| GraphMotionError::MissingVertex(to_key.clone()))]
                .into_iter()
                .flat_map(move |r| {
                    let from_state = from_state.clone();
                    let to_key = to_key.clone();
                    let with_guidance = with_guidance.clone();
                    r.flat_result_map(move |to_v_ref| {
                        let extrapolation = self.extrapolator.incremental_extrapolate(
                            self.space.waypoint(&from_state.base_state).borrow(),
                            to_v_ref.borrow(),
                            &with_guidance,
                        );

                        let from_state = from_state.clone();
                        let to_key = to_key.clone();
                        let with_guidance = with_guidance.clone();
                        extrapolation
                        .map_err(GraphMotionError::Extrapolator)
                        .flat_result_map(move |r| {
                            let from_state = from_state.clone();
                            let to_key = to_key.clone();
                            let with_guidance = with_guidance.clone();
                            r
                            .into_iter()
                            .map(move |(action, waypoint, progress)| {
                                let to_key = to_key.clone();
                                let with_guidance = with_guidance.clone();
                                if progress.incomplete() {
                                    let key = self.space.key_for(
                                        &from_state.base_state
                                    ).borrow().borrow().clone();
                                    let state = self.space.make_keyed_state(
                                        key, waypoint,
                                    );
                                    let state = IncrementalState {
                                        target: Some((to_key, with_guidance)),
                                        base_state: state,
                                    };
                                    (action, state)
                                } else {
                                    let state = self.space.make_keyed_state(
                                        to_key, waypoint
                                    );
                                    let state = IncrementalState {
                                        target: None,
                                        base_state: state,
                                    };
                                    (action, state)
                                }
                            })
                        })
                    })
                })
                .map(|x| x.flatten())
            )
        }

        ForkIter::Right(
            self
            .graph
            .edges_from_vertex(self.space.key_for(&from_state.base_state.clone()).borrow().borrow())
            .into_iter()
            .flat_map(move |edge| {
                let from_state = from_state.clone();
                let to_key = edge.to_vertex().clone();
                let with_guidance = edge.attributes().clone();
                self
                .graph
                .vertex(&to_key)
                .ok_or_else(|| GraphMotionError::MissingVertex(to_key.clone()))
                .flat_result_map(move |v| {
                    let extrapolation = self.extrapolator.incremental_extrapolate(
                        self.space.waypoint(&from_state.base_state).borrow(),
                        v.borrow(),
                        &with_guidance,
                    );

                    let from_state = from_state.clone();
                    let to_key = to_key.clone();
                    let with_guidance = with_guidance.clone();
                    extrapolation
                    .map_err(GraphMotionError::Extrapolator)
                    .flat_result_map(move |r| {
                        let from_state = from_state.clone();
                        let to_key = to_key.clone();
                        let with_guidance = with_guidance.clone();
                        r
                        .into_iter()
                        .map(move |(action, waypoint, progress)| {
                            if progress.incomplete() {
                                let key = self.space.key_for(
                                    &from_state.base_state
                                ).borrow().borrow().clone();
                                let state = self.space.make_keyed_state(
                                    key, waypoint
                                );
                                let state = IncrementalState {
                                    target: Some((to_key.clone(), with_guidance.clone())),
                                    base_state: state,
                                };
                                (action, state)
                            } else {
                                let state = self.space.make_keyed_state(
                                    to_key.clone(), waypoint,
                                );
                                let state = IncrementalState {
                                    target: None,
                                    base_state: state
                                };
                                (action, state)
                            }
                        })
                    })
                })
                .map(|x| x.flatten())
            })
        )
    }
}

impl<S, G, E> Keyed for IncrementalGraphMotion<S, G, E>
where
    S: Keyed,
{
    type Key = S::Key;
}

impl<S, G, E> PartialKeyed for IncrementalGraphMotion<S, G, E>
where
    S: PartialKeyed,
{
    type PartialKey = S::PartialKey;
}

impl<S, G, E> Keyring<IncrementalState<S::State, G>> for IncrementalGraphMotion<S, G, E>
where
    S: Domain + Keyring<S::State>,
    G: Graph,
{
    type KeyRef<'a> = S::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a;

    fn key_for<'a>(&'a self, state: &'a IncrementalState<S::State, G>) -> Self::KeyRef<'a>
    where
        Self: 'a,
        IncrementalState<S::State, G>: 'a
    {
        self.space.key_for(&state.base_state)
    }
}

impl<S, G, E> Reversible for IncrementalGraphMotion<S, G, E>
where
    S: Reversible,
    G: Reversible,
    E: Reversible,
{
    type Reverse = IncrementalGraphMotion<S::Reverse, G::Reverse, E::Reverse>;
    type ReversalError = GraphMotionReversalError<S::ReversalError, G::ReversalError, E::ReversalError>;
    fn reversed(&self) -> Result<Self::Reverse, Self::ReversalError> {
        Ok(IncrementalGraphMotion {
            space: self.space.reversed().map_err(GraphMotionReversalError::Space)?,
            graph: self.graph.reversed().map_err(GraphMotionReversalError::Graph)?,
            extrapolator: self.extrapolator.reversed().map_err(GraphMotionReversalError::Extrapolator)?,
        })
    }
}

#[derive(Debug)]
pub struct IncrementalState<BaseState, G: Graph> {
    target: Option<(G::Key, G::EdgeAttributes)>,
    base_state: BaseState,
}

impl<BaseState: Clone, G: Graph> Clone for IncrementalState<BaseState, G>
where
    G::Key: Clone,
    G::EdgeAttributes: Clone,
{
    fn clone(&self) -> Self {
        Self {
            target: self.target.clone(),
            base_state: self.base_state.clone(),
        }
    }
}
