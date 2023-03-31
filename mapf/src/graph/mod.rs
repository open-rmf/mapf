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

pub mod occupancy;

pub mod simple;
pub use simple::SimpleGraph;

pub mod shared_graph;
pub use shared_graph::SharedGraph;

use std::borrow::Borrow;

pub trait Edge<Key, Attributes> {
    fn from_vertex(&self) -> &Key;
    fn to_vertex(&self) -> &Key;
    fn attributes(&self) -> &Attributes;
}

/// The basic trait for Graphs to implement. Some algorithms might require
/// additional traits, such as [`crate::domain::Reversible`] or [`WithPointsOfInterest`].
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
    fn vertex<'a>(&'a self, key: &Self::Key) -> Option<Self::VertexRef<'a>>;

    /// Get the edges that originate from the given vertex.
    fn edges_from_vertex<'a>(&'a self, key: &Self::Key) -> Self::EdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    type LazyEdgeIter<'a>: IntoIterator<Item=Self::Edge<'a>> + 'a
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;

    /// Get the "lazy" edges that exist between from_key and to_key. Lazy Graphs
    /// ("lazy" in the sense of lazy evaluation) can cover more space with less
    /// branching by not returning all possible edges in the `edges_from_vertex`
    /// function and instead only returning edges that lead to critical vertices.
    /// Then those critical vertices can be supplemented with this
    /// `lazy_edges_between` function to find connections to points of interest
    /// that would not normally be included among the critical vertices.
    ///
    /// A correct implementation of this function should not return any edges
    /// that would normally show up by calling `edges_from_vertex(from_key)`. It
    /// should only return edges that are generated on demand.
    ///
    /// Graphs structures that are not capable of lazy evaluation can correctly
    /// implement this function by simply returning a zero-sized array.
    fn lazy_edges_between<'a>(
        &'a self,
        from_key: &Self::Key,
        to_key: &Self::Key
    ) -> Self::LazyEdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a;
}
