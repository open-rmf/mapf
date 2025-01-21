/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
    domain::Reversible,
    error::NoError,
    graph::{Edge, Graph},
};

#[derive(Debug, Clone, Default)]
pub struct SimpleGraph<V, E> {
    pub vertices: Vec<V>,
    pub edges: Vec<Vec<(usize, E)>>,
}

impl<V, E> SimpleGraph<V, E> {
    pub fn new(vertices: Vec<V>, edges: Vec<Vec<(usize, E)>>) -> Self {
        Self { vertices, edges }
    }

    pub fn from_iters(
        vertices: impl IntoIterator<Item = V>,
        input_edges: impl IntoIterator<Item = (usize, usize, E)>,
    ) -> Self {
        let mut edges = Vec::new();
        for edge in input_edges {
            if edges.len() <= edge.0 {
                edges.resize_with(edge.0 + 1, || Vec::new());
            }

            edges.get_mut(edge.0).unwrap().push((edge.1, edge.2));
        }

        Self {
            vertices: Vec::from_iter(vertices),
            edges,
        }
    }

    // TODO(@mxgrey): We could make an into_reverse(self) for cases where V and E
    // cannot be cloned.
}

impl<V: Clone, E: Clone> Reversible for SimpleGraph<V, E> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        let mut r_edges = Vec::new();
        r_edges.resize(self.edges.len(), Vec::new());
        for (r_v_to, edges) in self.edges.iter().enumerate() {
            for (r_v_from, e) in edges {
                r_edges
                    .get_mut(*r_v_from)
                    .unwrap()
                    .push((r_v_to, e.clone()));
            }
        }

        Ok(Self {
            vertices: self.vertices.clone(),
            edges: r_edges,
        })
    }
}

impl<E> Edge<usize, E> for (usize, usize, &E) {
    fn from_vertex(&self) -> &usize {
        &self.0
    }

    fn to_vertex(&self) -> &usize {
        &self.1
    }

    fn attributes(&self) -> &E {
        &self.2
    }
}

impl<V, E> Graph for SimpleGraph<V, E> {
    type Key = usize;
    type Vertex = V;
    type EdgeAttributes = E;

    type VertexRef<'a>
        = &'a Self::Vertex
    where
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    type Edge<'a>
        = (usize, usize, &'a E)
    where
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    type EdgeIter<'a>
        = impl Iterator<Item = (usize, usize, &'a E)> + 'a
    where
        V: 'a,
        E: 'a;

    fn vertex<'a>(&'a self, key: &usize) -> Option<&'a V> {
        self.vertices.get(*key)
    }

    fn edges_from_vertex<'a>(&'a self, from_key: &usize) -> Self::EdgeIter<'a>
    where
        V: 'a,
        E: 'a,
    {
        let from_key = *from_key;
        self.edges
            .get(from_key)
            .into_iter()
            .flat_map(|outgoing| outgoing.iter())
            .map(move |(to_key, attr)| (from_key, *to_key, attr))
    }

    type LazyEdgeIter<'a>
        = [(usize, usize, &'a E); 0]
    where
        V: 'a,
        E: 'a;

    fn lazy_edges_between<'a>(&'a self, _: &Self::Key, _: &Self::Key) -> Self::LazyEdgeIter<'a>
    where
        V: 'a,
        E: 'a,
    {
        []
    }
}
