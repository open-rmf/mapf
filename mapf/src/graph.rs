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

use crate::node::Key;

pub trait Edge<Key> {
    fn endpoint_key(&self) -> &Key;
}

pub trait Graph {
    type Key: Key;
    type Vertex;
    type Edge: Edge<Self::Key>;

    type EdgeIter<'a>: IntoIterator<Item=&'a Self::Edge>
    where
        Self: 'a,
        Self::Edge: 'a;

    fn vertex(&self, key: Self::Key) -> Option<&Self::Vertex>;

    fn edges_from_vertex<'a>(&'a self, key: Self::Key) -> Self::EdgeIter<'a>;
}

pub type KeyOf<G> = <G as Graph>::Key;
pub type VertexOf<G> = <G as Graph>::Vertex;
pub type EdgeOf<G> = <G as Graph>::Edge;
