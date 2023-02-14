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
        Domain, Space, Extrapolator, Activity, ActivityModifier,
        Initializable, Satisfiable, Closable, PartialKeyedSpace, Chained,
    },
    util::FlatResultMapTrait,
    error::Error,
};
use thiserror::Error as ThisError;
use std::sync::Arc;

pub struct GraphMotion<S, G, E> {
    pub space: S,
    pub graph: Arc<G>,
    pub extrapolator: E,
}

impl<S, G, E> Activity<S::State> for GraphMotion<S, G, E>
where
    S: PartialKeyedSpace<G::Key>,
    S::State: Clone,
    G: Graph,
    G::Key: Clone + 'static,
    G::EdgeAttributes: Clone + 'static,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
    E::ExtrapolationError: Error,
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
                let edge = edge.attributes().clone();
                self
                .graph
                .vertex(&to_vertex)
                .ok_or_else(|| GraphMotionError::MissingVertex(to_vertex.clone()))
                .flat_result_map(move |v| {
                    let from_state = from_state.clone();
                    let to_vertex = to_vertex.clone();
                    self
                    .extrapolator
                    .extrapolate(&self.space.waypoint(&from_state), &v, &edge)
                    .map_err(GraphMotionError::Extrapolator)
                    .flat_result_map(move |r| {
                        let to_vertex = to_vertex.clone();
                        r
                        .into_iter()
                        .map(move |(action, waypoint)| {
                            let state = self.space.make_partial_keyed_state(
                                Some(to_vertex.clone()), waypoint
                            );
                            (action, state)
                        })
                    })
                })
                .map(|x| x.flatten())
            })
        })
    }
}

#[derive(ThisError, Debug)]
pub enum GraphMotionError<K, E> {
    #[error("The graph is missing the requested vertex [{0}]")]
    MissingVertex(K),
    #[error("The extrapolator experienced an error:\n{0}")]
    Extrapolator(E),
    #[error("An action modifier experienced an error:\n{0}")]
    Modifier(anyhow::Error),
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_motion_activity_map() {

    }
}
