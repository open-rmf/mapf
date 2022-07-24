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

use crate::node::Key as KeyTrait;

pub trait Edge<Key: KeyTrait> {
    fn from_vertex(&self) -> &Key;
    fn to_vertex(&self) -> &Key;
}

pub trait Graph: std::fmt::Debug {
    type Key: KeyTrait;
    type Vertex;
    type Edge: Edge<Self::Key>;

    type EdgeIter<'a>: IntoIterator<Item=Self::Edge> where Self: 'a, Self::Edge: 'a;

    // TODO(MXG): Consider if there's a way we can have this API accept a key by
    // reference instead of by value. The current issue is that Node keys need a
    // way to be mapped (reduced) into Graph keys. When I tried using Into<&Key>
    // there was too much fuss from the borrow checker for me to get it working
    // correctly. We also have to consider: What if the Node key does not contain
    // a Graph key instance? In that case the Node key cannot provide a reference
    // to a Graph key.
    fn vertex(&self, key: Self::Key) -> Option<Self::Vertex>;

    fn edges_from_vertex<'a>(&'a self, key: Self::Key) -> Self::EdgeIter<'a>;
}

pub type KeyOf<G> = <G as Graph>::Key;
pub type VertexOf<G> = <G as Graph>::Vertex;
pub type EdgeOf<G> = <G as Graph>::Edge;
