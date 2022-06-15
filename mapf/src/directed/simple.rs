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

use std::vec::Vec;

#[derive(Debug, Clone, Default)]
pub struct SimpleGraph<Vertex: std::fmt::Debug + Clone> {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Vec<usize>>,

    /// A user may call edges_from_vertex with an invalid key, in which case we
    /// will return a reference to this always-empty vector.
    _placeholder: Vec<usize>,
}

impl<Vertex: std::fmt::Debug + Clone> SimpleGraph<Vertex> {

    pub fn new(vertices: Vec<Vertex>, edges: Vec<Vec<usize>>) -> Self {
        Self{vertices, edges, _placeholder: Vec::new()}
    }

    pub fn from_iters(
        vertices: impl IntoIterator<Item=Vertex>,
        input_edges: impl IntoIterator<Item=(usize, usize)>,
    ) -> Self {
        let mut edges = Vec::new();
        for edge in input_edges {
            if edges.len() <= edge.0 {
                edges.resize(edge.0 + 1, Vec::new());
            }

            edges.get_mut(edge.0).unwrap().push(edge.1);
        }

        Self{vertices: Vec::from_iter(vertices), edges, _placeholder: Vec::new()}
    }

    pub fn reverse(&self) -> Self {
        let mut r_edges = Vec::new();
        r_edges.resize(self.edges.len(), Vec::new());
        for (r_v_to, edges) in self.edges.iter().enumerate() {
            for r_v_from in edges {
                r_edges.get_mut(*r_v_from).unwrap().push(r_v_to);
            }
        }

        Self{
            vertices: self.vertices.clone(),
            edges: r_edges,
            _placeholder: Vec::new(),
        }
    }
}

impl crate::graph::Edge<usize> for usize {
    fn endpoint_key(&self) -> &usize {
        self
    }
}

impl<V: std::fmt::Debug + Clone> crate::Graph for SimpleGraph<V> {
    type Key = usize;
    type Vertex = V;
    type Edge = usize;

    type EdgeIter<'a> where Self: 'a = impl Iterator<Item=&'a usize> + 'a;

    fn vertex (&self, key: usize) -> Option<&V> {
        self.vertices.get(key)
    }

    fn edges_from_vertex<'a>(&'a self, key: usize) -> Self::EdgeIter<'a> {
        if let Some(to_vertices) = self.edges.get(key) {
            return to_vertices.iter();
        }

        return self._placeholder.iter();
    }
}
