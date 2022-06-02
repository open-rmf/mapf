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

#[derive(Debug, Clone)]
pub struct Graph<Vertex: Clone> {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Vec<usize>>,
}

impl<Vertex: Clone> Graph<Vertex> {
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
        }
    }
}
