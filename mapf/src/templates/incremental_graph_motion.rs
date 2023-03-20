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
        KeyedSpace, Keyed, PartialKeyed, Keyring, SelfKey, Backtrack,
    },
    motion::Timed,
    templates::graph_motion::{GraphMotionError, GraphMotionReversalError},
    util::{FlatResultMapTrait, ForkIter},
    error::StdError,
};
use std::{
    borrow::Borrow,
    fmt::Debug,
};

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
#[derive(Debug, Clone)]
pub struct IncrementalGraphMotion<S, G, E> {
    pub space: S,
    pub graph: G,
    pub extrapolator: E,
}

impl<S, G, E> IncrementalGraphMotion<S, G, E> {
    fn update_state_waypoint(
        &self,
        waypoint: S::Waypoint,
        original_state: &IncrementalState<S::State, G>,
    ) -> IncrementalState<S::State, G>
    where
        S: KeyedSpace<G::Key>,
        G: Graph,
        G::Key: Clone,
        G::EdgeAttributes: Clone,
    {
        let vertex = self.space.vertex_of(
            self.space.key_for(&original_state.base_state).borrow()
        ).clone();
        IncrementalState {
            target: original_state.target.clone(),
            base_state: self.space.make_keyed_state(vertex, waypoint)
        }
    }
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
    S: KeyedSpace<G::Key>,
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
    type ReversalError = GraphMotionReversalError<S::ReversalError, G::ReversalError, E::ReversalError>;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(IncrementalGraphMotion {
            space: self.space.reversed().map_err(GraphMotionReversalError::Space)?,
            graph: self.graph.reversed().map_err(GraphMotionReversalError::Graph)?,
            extrapolator: self.extrapolator.reversed().map_err(GraphMotionReversalError::Extrapolator)?,
        })
    }
}

impl<S, G, E> Backtrack<IncrementalState<S::State, G>, E::IncrementalExtrapolation> for IncrementalGraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes> + Backtrack<S::Waypoint, E::IncrementalExtrapolation>,
{
    type BacktrackError = E::BacktrackError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &IncrementalState<S::State, G>,
        final_reverse_state: &IncrementalState<S::State, G>,
    ) -> Result<(IncrementalState<S::State, G>, IncrementalState<S::State, G>), Self::BacktrackError> {
        self
        .extrapolator
        .flip_endpoints(
            self.space.waypoint(&initial_reverse_state.base_state).borrow(),
            self.space.waypoint(&final_reverse_state.base_state).borrow(),
        )
        .map(|(initial_forward_waypoint, final_forward_waypoint)| {
            (
                self.update_state_waypoint(initial_forward_waypoint, final_reverse_state),
                self.update_state_waypoint(final_forward_waypoint, initial_reverse_state),
            )
        })
    }

    fn backtrack(
        &self,
        parent_forward_state: &IncrementalState<S::State, G>,
        parent_reverse_state: &IncrementalState<S::State, G>,
        reverse_action: &E::IncrementalExtrapolation,
        child_reverse_state: &IncrementalState<S::State, G>,
    ) -> Result<(E::IncrementalExtrapolation, IncrementalState<S::State, G>), Self::BacktrackError> {
        self.extrapolator.backtrack(
            self.space.waypoint(&parent_forward_state.base_state).borrow(),
            self.space.waypoint(&parent_reverse_state.base_state).borrow(),
            reverse_action,
            self.space.waypoint(&child_reverse_state.base_state).borrow(),
        )
        .map(|(action, waypoint)| {
            let state = self.update_state_waypoint(waypoint, child_reverse_state);
            (action, state)
        })
    }
}

pub struct IncrementalState<BaseState, G: Graph> {
    target: Option<(G::Key, G::EdgeAttributes)>,
    base_state: BaseState,
}

impl<BaseState, G: Graph> Debug for IncrementalState<BaseState, G>
where
    BaseState: Debug,
    G::Key: Debug,
    G::EdgeAttributes: Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IncrementalState")
            .field("target", &self.target)
            .field("base_state", &self.base_state)
            .finish()
    }
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

impl<BaseState, G: Graph> From<BaseState> for IncrementalState<BaseState, G> {
    fn from(base_state: BaseState) -> Self {
        IncrementalState { target: None, base_state }
    }
}

impl<BaseState, G: Graph> Borrow<BaseState> for IncrementalState<BaseState, G> {
    fn borrow(&self) -> &BaseState {
        &self.base_state
    }
}

impl<BaseState: Keyed, G: Graph> Keyed for IncrementalState<BaseState, G> {
    type Key = BaseState::Key;
}

impl<BaseState: SelfKey, G: Graph> SelfKey for IncrementalState<BaseState, G> {
    type KeyRef<'a> = BaseState::KeyRef<'a> where BaseState: 'a, G: 'a;
    fn key<'a>(&'a self) -> Self::KeyRef<'a> where Self: 'a {
        self.base_state.key()
    }
}

impl<BaseState: Timed, G: Graph> Timed for IncrementalState<BaseState, G> {
    fn set_time(&mut self, new_time: time_point::TimePoint) {
        self.base_state.set_time(new_time)
    }

    fn time(&self) -> &time_point::TimePoint {
        self.base_state.time()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::SimpleGraph,
        domain::Initializable,
        motion::se2::{Point, StarburstSE2, StateSE2},
    };

    #[test]
    fn test_incremental_initialization() {
        /*
         *      5
         *      |
         *      |
         * 0----1----2
         *     / \
         *   /     \
         * 3         4
         *
         */

        let graph = SimpleGraph::from_iters(
            [
                Point::new(-1.0, 0.0), // 0
                Point::new(0.0, 0.0), // 1
                Point::new(1.0, 0.0), // 2
                Point::new(-1.0, -1.0), // 3
                Point::new(1.0, -1.0), // 4
                Point::new(0.0, 1.0), // 5
            ],
            [
                (0, 1, ()), (1, 0, ()),
                (2, 1, ()), (1, 2, ()),
                (3, 1, ()), (1, 3, ()),
                (4, 1, ()), (1, 4, ()),
                (5, 1, ()), (1, 5, ()),
            ]
        );

        const R: u32 = 100;
        let init = StarburstSE2::for_start(graph);
        let states: Result<Vec<IncrementalState<StateSE2<usize, R>, SimpleGraph<Point, ()>>>, _> = init.initialize(1).collect();
        let states = states.unwrap();
        assert!(states.len() == 5);
    }
}
