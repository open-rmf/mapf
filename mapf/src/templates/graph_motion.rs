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
        Activity, Backtrack, Domain, Extrapolator, Keyed, KeyedSpace, Keyring, PartialKeyed,
        Reversible,
    },
    error::{StdError, ThisError},
    graph::{Edge, Graph},
};
use std::borrow::Borrow;

#[derive(Debug, Clone)]
pub struct GraphMotion<S, G, E> {
    pub space: S,
    pub graph: G,
    pub extrapolator: E,
}

impl<S, G, E> GraphMotion<S, G, E> {
    fn update_state_waypoint(&self, waypoint: S::Waypoint, original_state: &S::State) -> S::State
    where
        S: KeyedSpace<G::Key>,
        G: Graph,
        G::Key: Clone,
    {
        let vertex = self
            .space
            .vertex_of(self.space.key_for(&original_state).borrow())
            .clone();
        self.space.make_keyed_state(vertex, waypoint)
    }
}

/// [`GraphMotion`] defines a domain and activity that efficiently moves over a
/// spatial graph. This is generally used for single-shot motion planning
/// problems.
///
/// The activity of `GraphMotion` is meant to be efficient, generating as few
/// search states as possible while traversing the graph. This is usually
/// desirable since it keeps the memory footprint down. However, searches that
/// cache search results like Dijkstra would become ineffective because it won't
/// be able to cache all the relevant states that the search passes through.
///
/// For search algorithms that benefit from more state coverage, use
/// [`super::IncrementalGraphMotion`].
impl<S, G, E> Domain for GraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
{
    type State = S::State;
    type Error = GraphMotionError<G::Key, E::ExtrapolationError>;
}

impl<S, G, E> Activity<S::State> for GraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::ExtrapolationError: StdError,
{
    type Action = E::Extrapolation;
    type ActivityError = GraphMotionError<G::Key, E::ExtrapolationError>;
    type Choices<'a> = GraphMotionChoices<'a, S, G, E>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::EdgeAttributes: 'a,
        G::Key: 'a,
        S::State: 'a;

    fn choices<'a>(&'a self, from_state: S::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::Key: 'a,
    {
        let from_vertex = self.space.key_for(&from_state).borrow().borrow().clone();
        let edges_from_vertex = self.graph.edges_from_vertex(&from_vertex).into_iter();
        GraphMotionChoices {
            current_iter: None,
            edges_from_vertex,
            motion: &self,
            from_state,
        }
    }
}

impl<S, G, E> Keyed for GraphMotion<S, G, E>
where
    S: Keyed,
{
    type Key = S::Key;
}

impl<S, G, E> PartialKeyed for GraphMotion<S, G, E>
where
    S: PartialKeyed,
{
    type PartialKey = S::PartialKey;
}

impl<S, G, E> Keyring<S::State> for GraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
{
    type KeyRef<'a>
        = S::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a;

    fn key_for<'a>(&'a self, state: &'a S::State) -> Self::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a,
    {
        self.space.key_for(state)
    }
}

impl<S, G, E> Reversible for GraphMotion<S, G, E>
where
    S: Reversible,
    G: Reversible,
    E: Reversible,
{
    type ReversalError =
        GraphMotionReversalError<S::ReversalError, G::ReversalError, E::ReversalError>;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(GraphMotion {
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

impl<S, G, E> Backtrack<S::State, E::Extrapolation> for GraphMotion<S, G, E>
where
    S: KeyedSpace<G::Key>,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>
        + Backtrack<S::Waypoint, E::Extrapolation>,
{
    type BacktrackError = E::BacktrackError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &S::State,
        final_reverse_state: &S::State,
    ) -> Result<(S::State, S::State), Self::BacktrackError> {
        self.extrapolator
            .flip_endpoints(
                self.space.waypoint(&initial_reverse_state).borrow(),
                self.space.waypoint(&final_reverse_state).borrow(),
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
        parent_forward_state: &S::State,
        parent_reverse_state: &S::State,
        reverse_action: &E::Extrapolation,
        child_reverse_state: &S::State,
    ) -> Result<(E::Extrapolation, S::State), Self::BacktrackError> {
        self.extrapolator
            .backtrack(
                self.space.waypoint(&parent_forward_state).borrow(),
                self.space.waypoint(&parent_reverse_state).borrow(),
                reverse_action,
                self.space.waypoint(&child_reverse_state).borrow(),
            )
            .map(|(action, waypoint)| {
                let state = self.update_state_waypoint(waypoint, child_reverse_state);
                (action, state)
            })
    }
}

pub struct GraphMotionChoices<'a, S, G, E>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::ExtrapolationError: StdError,
{
    current_iter: Option<Extrapolations<G::Key, <E::ExtrapolationIter<'a> as IntoIterator>::IntoIter>>,
    edges_from_vertex: <G::EdgeIter<'a> as IntoIterator>::IntoIter,
    motion: &'a GraphMotion<S, G, E>,
    from_state: S::State,
}

impl<'a, S, G, E> Iterator for GraphMotionChoices<'a, S, G, E>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::ExtrapolationError: StdError,
{
    type Item = Result<(E::Extrapolation, S::State), GraphMotionError<G::Key, E::ExtrapolationError>>;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(current_iter) = self.current_iter.as_mut() {
                if let Some(result) = current_iter.extrapolations.next() {
                    match result {
                        Ok((action, waypoint)) => {
                            let to_vertex = current_iter.to_vertex.clone();
                            let state = self.motion.space.make_keyed_state(to_vertex, waypoint);
                            return Some(Ok((action, state)));
                        }
                        Err(err) => {
                            return Some(Err(GraphMotionError::Extrapolator(err)));
                        }
                    }
                }
            }
            self.current_iter = None;

            if let Some(edge) = self.edges_from_vertex.next() {
                let to_vertex = edge.to_vertex();
                let Some(v) = self.motion.graph.vertex(to_vertex) else {
                    return Some(Err(GraphMotionError::MissingVertex(to_vertex.clone())));
                };

                let extrapolations = self.motion.extrapolator.extrapolate(
                    self.motion.space.waypoint(&self.from_state).borrow(),
                    v.borrow(),
                    &edge.attributes(),
                    (
                        Some(self.motion.space.key_for(&self.from_state).borrow().borrow()),
                        Some(&to_vertex),
                    ),
                );

                self.current_iter = Some(Extrapolations {
                    to_vertex: to_vertex.clone(),
                    extrapolations: extrapolations.into_iter(),
                });
                // Move back to the start of this loop to immediately iterate
                // over the new set of extrapolations.
                continue;
            }

            return None;
        }

    }
}

struct Extrapolations<K, E> {
    to_vertex: K,
    extrapolations: E,
}

#[derive(ThisError, Debug)]
pub enum GraphMotionError<K, E> {
    #[error("The graph is missing the requested vertex [{0:?}]")]
    MissingVertex(K),
    #[error("The extrapolator experienced an error:\n{0:?}")]
    Extrapolator(E),
}

#[derive(ThisError, Debug)]
pub enum GraphMotionReversalError<S, G, E> {
    #[error("The space had a reversal error:\n{0:?}")]
    Space(S),
    #[error("The graph had a reversal error:\n{0:?}")]
    Graph(G),
    #[error("The extrapolator had a reversal error:\n{0:?}")]
    Extrapolator(E),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::SimpleGraph,
        motion::r2::{DiscreteSpaceTimeR2, LineFollow, Position},
    };
    use std::sync::Arc;

    #[test]
    fn test_graph_motion_construction() {
        let graph = SimpleGraph::from_iters(
            [
                Position::new(0.0, 0.0),
                Position::new(1.0, 0.0),
                Position::new(3.0, -1.0),
            ],
            [(0, 1, ()), (1, 0, ()), (0, 2, ()), (2, 1, ())],
        );
        let graph = Arc::new(graph);

        let motion = GraphMotion {
            space: DiscreteSpaceTimeR2::<usize>::default(),
            graph,
            extrapolator: LineFollow::new(1.0),
        };
        println!("{motion:?}");
    }
}
