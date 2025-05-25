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
        Activity, Backtrack, Domain, IncrementalExtrapolator, Keyed, KeyedSpace, Keyring,
        PartialKeyed, Reversible, SelfKey,
    },
    error::StdError,
    graph::{Edge, Graph},
    motion::{MaybeTimed, TimePoint, Timed},
    templates::graph_motion::{GraphMotionError, GraphMotionReversalError},
};
use std::{borrow::Borrow, fmt::Debug};

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
        let vertex = self
            .space
            .vertex_of(self.space.key_for(&original_state.base_state).borrow())
            .clone();
        IncrementalState {
            target: original_state.target.clone(),
            base_state: self.space.make_keyed_state(vertex, waypoint),
        }
    }
}

impl<S, G, E> Domain for IncrementalGraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
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
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::IncrementalExtrapolationError: StdError,
{
    type Action = E::IncrementalExtrapolation;
    type ActivityError = GraphMotionError<G::Key, E::IncrementalExtrapolationError>;
    type Choices<'a> = IncrementalGraphMotionChoices<'a, S, G, E>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::EdgeAttributes: 'a;

    fn choices<'a>(&'a self, from_state: IncrementalState<S::State, G>) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        IncrementalState<S::State, G>: 'a,
    {
        if let Some(target) = from_state.target {
            // Just keep moving this state towards its target.
            IncrementalGraphMotionChoices {
                current_iter: None,
                next_target: Some(target),
                edges_from_vertex: None,
                motion: &self,
                from_base_state: from_state.base_state,
            }
        } else {
            // No specific target, so we want to expand in every direction from
            // this vertex.
            let edges_from_vertex = self.graph.edges_from_vertex(
                self.space
                    .key_for(&from_state.base_state)
                    .borrow()
                    .borrow()
            )
            .into_iter();

            IncrementalGraphMotionChoices {
                current_iter: None,
                next_target: None,
                edges_from_vertex: Some(edges_from_vertex),
                motion: &self,
                from_base_state: from_state.base_state,
            }
        }
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
    type KeyRef<'a>
        = S::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a;

    fn key_for<'a>(&'a self, state: &'a IncrementalState<S::State, G>) -> Self::KeyRef<'a>
    where
        Self: 'a,
        IncrementalState<S::State, G>: 'a,
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
    type ReversalError =
        GraphMotionReversalError<S::ReversalError, G::ReversalError, E::ReversalError>;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(IncrementalGraphMotion {
            space: self
                .space
                .reversed()
                .map_err(GraphMotionReversalError::Space)?,
            graph: self
                .graph
                .reversed()
                .map_err(GraphMotionReversalError::Graph)?,
            extrapolator: self
                .extrapolator
                .reversed()
                .map_err(GraphMotionReversalError::Extrapolator)?,
        })
    }
}

impl<S, G, E> Backtrack<IncrementalState<S::State, G>, E::IncrementalExtrapolation>
    for IncrementalGraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>
        + Backtrack<S::Waypoint, E::IncrementalExtrapolation>,
{
    type BacktrackError = E::BacktrackError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &IncrementalState<S::State, G>,
        final_reverse_state: &IncrementalState<S::State, G>,
    ) -> Result<(IncrementalState<S::State, G>, IncrementalState<S::State, G>), Self::BacktrackError>
    {
        self.extrapolator
            .flip_endpoints(
                self.space
                    .waypoint(&initial_reverse_state.base_state)
                    .borrow(),
                self.space
                    .waypoint(&final_reverse_state.base_state)
                    .borrow(),
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
    ) -> Result<(E::IncrementalExtrapolation, IncrementalState<S::State, G>), Self::BacktrackError>
    {
        self.extrapolator
            .backtrack(
                self.space
                    .waypoint(&parent_forward_state.base_state)
                    .borrow(),
                self.space
                    .waypoint(&parent_reverse_state.base_state)
                    .borrow(),
                reverse_action,
                self.space
                    .waypoint(&child_reverse_state.base_state)
                    .borrow(),
            )
            .map(|(action, waypoint)| {
                let state = self.update_state_waypoint(waypoint, child_reverse_state);
                (action, state)
            })
    }
}

pub struct IncrementalGraphMotionChoices<'a, S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
{
    current_iter: Option<Extrapolations<G::Key, G::EdgeAttributes, <E::IncrementalExtrapolationIter<'a> as IntoIterator>::IntoIter>>,
    next_target: Option<(G::Key, G::EdgeAttributes)>,
    edges_from_vertex: Option<<G::EdgeIter<'a> as IntoIterator>::IntoIter>,
    motion: &'a IncrementalGraphMotion<S, G, E>,
    from_base_state: S::State,
}

impl<'a, S, G, E> Iterator for IncrementalGraphMotionChoices<'a, S, G, E>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: IncrementalExtrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::IncrementalExtrapolationError: StdError,
{
    type Item = Result<
        (E::IncrementalExtrapolation, IncrementalState<S::State, G>),
        GraphMotionError<G::Key, E::IncrementalExtrapolationError>,
    >;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(current_iter) = self.current_iter.as_mut() {
                if let Some(result) = current_iter.extrapolations.next() {
                    match result {
                        Ok((action, waypoint, progress)) => {
                            let target = &current_iter.target;
                            if progress.incomplete() {
                                let key = self
                                    .motion
                                    .space
                                    .key_for(&self.from_base_state)
                                    .borrow()
                                    .borrow()
                                    .clone();
                                let state = IncrementalState {
                                    target: Some(current_iter.target.clone()),
                                    base_state: self.motion.space.make_keyed_state(key, waypoint),
                                };
                                return Some(Ok((action, state)));
                            } else {
                                let key = target.0.clone();
                                let state = IncrementalState {
                                    target: None,
                                    base_state: self.motion.space.make_keyed_state(key, waypoint),
                                };
                                return Some(Ok((action, state)));
                            }
                        }
                        Err(err) => {
                            return Some(Err(GraphMotionError::Extrapolator(err)));
                        }
                    }
                }
            }
            self.current_iter = None;

            if let Some((to_key, with_guidance)) = Option::take(&mut self.next_target) {
                let Some(to_v_ref) = self.motion.graph.vertex(&to_key) else {
                    return Some(Err(GraphMotionError::MissingVertex(to_key)));
                };

                let from_waypoint_ref = self.motion.space.waypoint(&self.from_base_state);
                let from_waypoint = from_waypoint_ref.borrow();
                let extrapolations = self.motion.extrapolator.incremental_extrapolate(
                    from_waypoint,
                    to_v_ref.borrow(),
                    &with_guidance,
                    (
                        Some(self.motion.space.key_for(&self.from_base_state).borrow().borrow()),
                        Some(&to_key),
                    ),
                );

                self.current_iter = Some(Extrapolations {
                    target: (to_key, with_guidance),
                    extrapolations: extrapolations.into_iter(),
                });
                // Return to the start of the loop to begin pulling items out
                // of the extrapolation iterator.
                continue;
            }

            if let Some(edges_from_vertex) = self.edges_from_vertex.as_mut() {
                if let Some(edge) = edges_from_vertex.next() {
                    let to_vertex = edge.to_vertex().clone();
                    let with_guidance = edge.attributes().clone();
                    self.next_target = Some((to_vertex, with_guidance));
                    // Return to the start of the loop to extrapolate toward
                    // this new target.
                    continue;
                }
            }

            return None;
        }
    }
}

struct Extrapolations<Key, Guidance, E> {
    target: (Key, Guidance),
    extrapolations: E,
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
        IncrementalState {
            target: None,
            base_state,
        }
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
    type KeyRef<'a>
        = BaseState::KeyRef<'a>
    where
        BaseState: 'a,
        G: 'a;
    fn key<'a>(&'a self) -> Self::KeyRef<'a>
    where
        Self: 'a,
    {
        self.base_state.key()
    }
}

impl<BaseState: Timed, G: Graph> Timed for IncrementalState<BaseState, G> {
    fn set_time(&mut self, new_time: TimePoint) {
        self.base_state.set_time(new_time)
    }

    fn time(&self) -> TimePoint {
        self.base_state.time()
    }
}

impl<BaseState: Timed, G: Graph> MaybeTimed for IncrementalState<BaseState, G> {
    fn maybe_time(&self) -> Option<TimePoint> {
        Some(self.base_state.time())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        domain::Initializable,
        graph::SimpleGraph,
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
                Point::new(-1.0, 0.0),  // 0
                Point::new(0.0, 0.0),   // 1
                Point::new(1.0, 0.0),   // 2
                Point::new(-1.0, -1.0), // 3
                Point::new(1.0, -1.0),  // 4
                Point::new(0.0, 1.0),   // 5
            ],
            [
                (0, 1, ()),
                (1, 0, ()),
                (2, 1, ()),
                (1, 2, ()),
                (3, 1, ()),
                (1, 3, ()),
                (4, 1, ()),
                (1, 4, ()),
                (5, 1, ()),
                (1, 5, ()),
            ],
        );

        const R: u32 = 100;
        let init = StarburstSE2::for_start(graph);
        let states: Result<Vec<IncrementalState<StateSE2<usize, R>, SimpleGraph<Point, ()>>>, _> =
            init.initialize(1, &4).collect();
        let states = states.unwrap();
        assert!(states.len() == 5);
    }
}
