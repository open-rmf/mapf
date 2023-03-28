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
        Domain, Extrapolator, Activity, Reversible,
        KeyedSpace, Keyed, PartialKeyed, Keyring,
    },
    util::FlatResultMapTrait,
    error::{StdError, ThisError},
};
use std::borrow::Borrow;

#[derive(Debug, Clone)]
pub struct GraphMotion<S, G, E> {
    pub space: S,
    pub graph: G,
    pub extrapolator: E,
}

/// `GraphMotion` defines a domain and activity that efficiently moves over a
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
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
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
    G::Key: Clone + 'static,
    G::EdgeAttributes: Clone + 'static,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
    E::ExtrapolationError: StdError,
{
    type ActivityAction = E::Extrapolation;
    type ActivityError = GraphMotionError<G::Key, E::ExtrapolationError>;
    type Choices<'a> = impl IntoIterator<Item = Result<(Self::ActivityAction, S::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
        G::EdgeAttributes: 'a,
        S::State: 'a;

    fn choices<'a>(&'a self, from_state: S::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        S::State: 'a,
    {
        self
        .graph
        .edges_from_vertex(self.space.key_for(&from_state.clone()).borrow().borrow())
        .into_iter()
        .flat_map(move |edge| {
            let from_state = from_state.clone();
            let to_vertex = edge.to_vertex().clone();
            let edge = edge.attributes().clone();
            self
            .graph
            .vertex(&to_vertex)
            .ok_or_else(|| GraphMotionError::MissingVertex(to_vertex.clone()))
            .flat_result_map(move |v| {
                let from_state = from_state.clone();
                let to_vertex = to_vertex.clone();
                let extrapolations = self.extrapolator.extrapolate(
                    self.space.waypoint(&from_state).borrow(), v.borrow(), &edge
                );

                extrapolations
                .into_iter()
                .map(move |r| {
                    let to_vertex = to_vertex.clone();
                    r
                    .map_err(GraphMotionError::Extrapolator)
                    .map(move |(action, waypoint)| {
                        let to_vertex = to_vertex.clone();
                        let state = self.space.make_keyed_state(
                            to_vertex.clone(), waypoint
                        );
                        (action, state)
                    })
                })
            })
            .map(|x| x.flatten())
        })
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
    S: Domain + Keyring<S::State>,
{
    type KeyRef<'a> = S::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a;

    fn key_for<'a>(&'a self, state: &'a S::State) -> Self::KeyRef<'a>
    where
        Self: 'a,
        S::State: 'a
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
    type ReversalError = GraphMotionReversalError<S::ReversalError, G::ReversalError, E::ReversalError>;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(GraphMotion {
            space: self.space.reversed().map_err(GraphMotionReversalError::Space)?,
            graph: self.graph.reversed().map_err(GraphMotionReversalError::Graph)?,
            extrapolator: self.extrapolator.reversed().map_err(GraphMotionReversalError::Extrapolator)?,
        })
    }
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
    #[error("The space had a reversal error:\n{0}")]
    Space(S),
    #[error("The graph had a reversal error:\n{0}")]
    Graph(G),
    #[error("The extrapolator had a reversal error:\n{0}")]
    Extrapolator(E),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        graph::SimpleGraph,
        motion::r2::{Position, DiscreteSpaceTimeR2, LineFollow},
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
            [
                (0, 1, ()),
                (1, 0, ()),
                (0, 2, ()),
                (2, 1, ()),
            ],
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
