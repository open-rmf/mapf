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

pub trait Edge<Key, Attributes> {
    fn from_vertex(&self) -> &Key;
    fn to_vertex(&self) -> &Key;
    fn attributes(&self) -> &Attributes;
}

pub trait Graph {
    type Vertex;
    type Key;
    type EdgeAttributes;
    type Edge: Edge<Self::Key, Self::EdgeAttributes>;

    type EdgeIter<'a>: IntoIterator<Item=Self::Edge> + 'a
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
        Self::Edge: 'a;

    /// Get the vertex associated with `key`. If no such vertex exists, this
    /// will return None.
    fn vertex(&self, key: &Self::Key) -> Option<Self::Vertex>;

    /// Get the edges that originate from the given vertex.
    fn edges_from_vertex<'a>(&'a self, key: Self::Key) -> Self::EdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
        Self::Edge: 'a;
}
