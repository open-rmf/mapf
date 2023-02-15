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

use std::borrow::Borrow;

pub trait Edge<Key, Attributes> {
    fn from_vertex(&self) -> &Key;
    fn to_vertex(&self) -> &Key;
    fn attributes(&self) -> &Attributes;
}

// TODO(MXG): Consider if it there is a way to assign a lifetime bound to Vertex
// and Edge so that vertex() and edges_from_vertex() could choose between
// returning a concrete value or a reference. For some types of Graph it would
// be more efficient to return a reference type, but for other types of Graph it
// is not possible to return a reference.
pub trait Graph {
    /// What type of data is stored at each vertex of the graph.
    type Vertex;

    /// What kind of key is used to uniquely access the data of each vertex.
    type Key;

    /// What kind of data is stored along each edge that connects two vertices.
    type EdgeAttributes;

    /// The return type that provides access to vertex data. This can be a
    /// [`Self::Vertex`] value or a reference to a [`Self::Vertex`] value.
    /// Structs that implement `Graph` can choose whichever makes the most sense
    /// for their data structure.
    type VertexRef<'a>: Borrow<Self::Vertex> + 'a
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    /// The return type that provides access to edge data. This must implement
    /// the [`Edge`] trait and provide access to the [`Self::EdgeAttributes`]
    /// for an edge.
    type Edge<'a>: Edge<Self::Key, Self::EdgeAttributes> + 'a
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    /// The return type that allows users to iterate over the edges that come
    /// out of a vertex.
    type EdgeIter<'a>: IntoIterator<Item=Self::Edge<'a>> + 'a
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    /// Get the vertex associated with `key`. If no such vertex exists, this
    /// will return None.
    fn vertex<'a, 'b>(&'a self, key: &'b Self::Key) -> Option<Self::VertexRef<'a>>;

    /// Get the edges that originate from the given vertex.
    fn edges_from_vertex<'a>(&'a self, key: Self::Key) -> Self::EdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;
}
