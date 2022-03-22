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

use std::hash::{Hash, Hasher, BuildHasher};
use std::rc::Rc;
use std::collections::hash_map::Entry;

pub trait Node : Sized {
    type ClosedSet: ClosedSet<Self>;
}

pub enum CloseResult<NodeType: Node> {
    /// The node was successfully closed. No other node was previously closed
    /// with an equivalent state.
    Closed,

    /// A node with an equivalent state was previous closed. This contains a
    /// reference to that node.
    Prior(Rc<NodeType>)
}

pub enum ClosedStatus<NodeType: Node> {
    Open,
    Closed(Rc<NodeType>)
}

pub trait ClosedSet<NodeType: Node> {

    /// Tell the closed set to close this node.
    fn close(&mut self, node: &Rc<NodeType>) -> CloseResult<NodeType>;

    /// Check whether an equivalent node has been closed.
    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType>;
}

pub struct HashClosedSet<NodeType> where
    NodeType: Hash {
    closed_set: std::collections::HashMap<u64, Rc<NodeType>>
}

impl<NodeType> ClosedSet<NodeType> for HashClosedSet<NodeType> where
    NodeType: Node + Hash {

    fn close(&mut self, node: &Rc<NodeType>) -> CloseResult<NodeType> {
        // It would be nice to use a HashSet instead of a HashMap, but the
        // HashSet API does not have an equivalent to the HashMap's Entry API,
        // which makes this function far more efficient.
        let mut hasher = self.closed_set.hasher().build_hasher();
        node.hash(&mut hasher);
        let key = hasher.finish();
        let entry = self.closed_set.entry(key);
        match entry {
            Entry::Occupied(occupied) => {
                return CloseResult::Prior(occupied.get().clone());
            },
            Entry::Vacant(vacant) => {
                vacant.insert(node.clone());
                return CloseResult::Closed;
            }
        }
    }

    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType> {
        let mut hasher = self.closed_set.hasher().build_hasher();
        node.hash(&mut hasher);
        let key = hasher.finish();
        let entry = self.closed_set.get(&key);
        match entry {
            Some(value) => {
                return ClosedStatus::Closed(value.clone());
            },
            None => {
                return ClosedStatus::Open
            }
        }
    }
}
