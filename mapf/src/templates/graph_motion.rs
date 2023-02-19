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
    domain::{Domain, Extrapolator, Activity, KeyedSpace},
    util::FlatResultMapTrait,
    error::StdError,
};
use thiserror::Error as ThisError;
use std::{
    sync::Arc,
    borrow::Borrow,
};

pub struct GraphMotion<S, G, E> {
    pub space: S,
    pub graph: Arc<G>,
    pub extrapolator: E,
}

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
                let extrapolation = self.extrapolator.extrapolate(
                    self.space.waypoint(&from_state).borrow(), v.borrow(), &edge
                );

                extrapolation
                .map_err(GraphMotionError::Extrapolator)
                .flat_result_map(move |r| {
                    let to_vertex = to_vertex.clone();
                    r
                    .into_iter()
                    .map(move |(action, waypoint)| {
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

#[derive(ThisError, Debug)]
pub enum GraphMotionError<K, E> {
    #[error("The graph is missing the requested vertex [{0:?}]")]
    MissingVertex(K),
    #[error("The extrapolator experienced an error:\n{0:?}")]
    Extrapolator(E),
    #[error("An action modifier experienced an error:\n{0}")]
    Modifier(anyhow::Error),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        directed::SimpleGraph,
        motion::r2::{
            Position, DiscreteSpaceTimeR2,
            timed_position::LineFollow,
        },
    };
    use std::sync::Arc;

    #[test]
    fn test_motion_activity_map() {
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
    }
}
