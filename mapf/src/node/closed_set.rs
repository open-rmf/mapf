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

use crate::node::traits::*;
use std::sync::Arc;
use std::collections::hash_map::{HashMap, Entry};

/// The result of attempting to add a node to the Closed Set.
pub enum CloseResult<N> {
    /// The node was successfully closed. No other node was previously closed
    /// with an equivalent state.
    Closed,

    /// A node with an equivalent state was previous closed. This contains a
    /// reference to that node.
    Prior(Arc<N>)
}

impl<N> CloseResult<N> {
    pub fn accepted(&self) -> bool {
        match self {
            Self::Closed => true,
            Self::Prior(_) => false,
        }
    }

    pub fn rejected(&self) -> bool {
        !self.accepted()
    }
}

/// The result of checking whether an equivalent node is already in the ClosedSet.
// TODO(MXG): Consider returning an &Arc<N> to reduce reference counting overhead
pub enum ClosedStatus<N> {
    Open,
    Closed(Arc<N>)
}

impl<N> ClosedStatus<N> {
    pub fn is_open(&self) -> bool {
        match self {
            Self::Open => true,
            Self::Closed(_) => false,
        }
    }

    pub fn is_closed(&self) -> bool {
        !self.is_open()
    }
}

/// The generic trait of a Closed Set. "Closed Sets" are used to avoid
/// unnecessary search effort. They keep track of the lowest cost node which has
/// visited a certain location.
pub trait ClosedSet<N>: Default {

    type Iter<'a>: IntoIterator<Item=&'a Arc<N>> where Self: 'a, N: 'a;

    /// Tell the closed set to close this node.
    fn close(&mut self, node: &Arc<N>) -> CloseResult<N>;

    /// Check whether an equivalent node has been closed.
    fn status(&self, node: &N) -> ClosedStatus<N>;

    /// Iterate through all the entries
    fn iter<'a>(&'a self) -> Self::Iter<'a>;
}

pub trait KeyedSet<N> {
    /// The key type for the set. This can match the key type of the Node if the
    /// Node has a PartialKeyed trait, but that is not a requirement in general.
    type Key: Key;

    /// Get the node in the closed set that matches this key
    fn get(&self, key: &Self::Key) -> Option<&Arc<N>>;
}

pub struct PartialKeyedClosedSet<N: Weighted + PartialKeyed> {
    closed_set: HashMap<N::Key, Arc<N>>
}

impl<N: Weighted + PartialKeyed> Default for PartialKeyedClosedSet<N> {
    fn default() -> Self {
        return PartialKeyedClosedSet {
            closed_set: Default::default()
        }
    }
}

impl<N: Weighted + PartialKeyed> ClosedSet<N> for PartialKeyedClosedSet<N> {
    type Iter<'a> where N: 'a = impl Iterator<Item=&'a Arc<N>> + 'a;

    fn close(&mut self, node: &Arc<N>) -> CloseResult<N> {
        if let Some(key) = node.partial_key() {
            match self.closed_set.entry(key.clone()) {
                Entry::Occupied(mut occupied) => {
                    if occupied.get().cost() <= node.cost() {
                        return CloseResult::Prior(occupied.get().clone());
                    }

                    occupied.insert(node.clone());
                    return CloseResult::Closed;
                },
                Entry::Vacant(vacant) => {
                    vacant.insert(node.clone());
                    return CloseResult::Closed;
                }
            }
        }

        return CloseResult::Closed;
    }

    fn status(&self, node: &N) -> ClosedStatus<N> {
        if let Some(key) = node.partial_key() {
            match self.closed_set.get(key) {
                Some(value) => {
                    return ClosedStatus::Closed(value.clone());
                },
                None => {
                    return ClosedStatus::Open
                }
            }
        }

        return ClosedStatus::Open;
    }

    fn iter<'a>(&'a self) -> Self::Iter<'a> {
        self.closed_set.iter().map(|(_, n)| n)
    }
}

impl<N: Weighted + PartialKeyed> KeyedSet<N> for PartialKeyedClosedSet<N> {

    type Key = N::Key;

    fn get(&self, key: &Self::Key) -> Option<&Arc<N>> {
        self.closed_set.get(key)
    }
}

pub struct TimeVariantPartialKeyedClosetSet<N: Weighted + PartialKeyed + Timed> {
    closed_set: HashMap<N::Key, HashMap<i64, Arc<N>>>,
    time_thresh: i64,
}

// Note: A user could alternatively incorporate the time value into the key of their node,
// but this alternative closed set provides an easy way to use the same node type with the
// same key type for either time variant or invariant scenarios.
impl<N: Weighted + PartialKeyed + Timed> Default for TimeVariantPartialKeyedClosetSet<N> {
    fn default() -> Self {
        Self{closed_set: Default::default(), time_thresh: 100_000_000}
    }
}

impl<N: Weighted + PartialKeyed + Timed> ClosedSet<N> for TimeVariantPartialKeyedClosetSet<N> {
    type Iter<'a> where N: 'a = impl Iterator<Item=&'a Arc<N>> + 'a;

    fn close(&mut self, node: &Arc<N>) -> CloseResult<N> {
        if let Some(key) = node.partial_key() {
            let t_key = node.time().nanos_since_zero / self.time_thresh;
            match self.closed_set.entry(key.clone()) {
                Entry::Occupied(mut occupied) => {
                    let t_map = occupied.get_mut();
                    match t_map.entry(t_key) {
                        Entry::Occupied(mut occupied) => {
                            if occupied.get().cost() <= node.cost() {
                                return CloseResult::Prior(occupied.get().clone());
                            }

                            occupied.insert(node.clone());
                            return CloseResult::Closed;
                        },
                        Entry::Vacant(vacant) => {
                            vacant.insert(node.clone());
                            return CloseResult::Closed;
                        }
                    }
                },
                Entry::Vacant(vacant) => {
                    let t_map = vacant.insert(Default::default());
                    t_map.insert(t_key, node.clone());
                    return CloseResult::Closed;
                }
            }
        }

        return CloseResult::Closed;
    }

    fn status(&self, node: &N) -> ClosedStatus<N> {
        if let Some(key) = node.partial_key() {
            match self.closed_set.get(key) {
                Some(t_map) => {
                    let t_key = node.time().nanos_since_zero / self.time_thresh;
                    match t_map.get(&t_key) {
                        Some(value) => {
                            return ClosedStatus::Closed(value.clone());
                        },
                        None => {
                            return ClosedStatus::Open;
                        }
                    }
                },
                None => {
                    return ClosedStatus::Open;
                }
            }
        }

        return ClosedStatus::Open;
    }

    fn iter<'a>(&'a self) -> Self::Iter<'a> {
        self.closed_set.iter().flat_map(|(_, t_map)| t_map).map(|(_, n)| n)
    }
}


#[cfg(test)]
mod tests {

    use super::*;

    struct TestNode {
        graph_index: usize,
        cost: u64,
        remaining_cost_estimate: u64,
        parent: Option<Arc<Self>>,
    }

    impl PartialKeyed for TestNode {
        type Key = usize;

        fn partial_key(&self) -> Option<&Self::Key> {
            Some(&self.graph_index)
        }
    }

    impl PathSearch for TestNode {
        fn parent(&self) -> &Option<Arc<Self>> {
            return &self.parent;
        }
    }

    impl Weighted for TestNode {
        type Cost = u64;

        fn cost(&self) -> u64 {
            return self.cost;
        }
    }

    impl Informed for TestNode {
        fn remaining_cost_estimate(&self) -> u64 {
            return self.remaining_cost_estimate;
        }
        fn total_cost_estimate(&self) -> u64 {
            return self.cost + self.remaining_cost_estimate;
        }
    }

    #[test]
    fn hashable_node_can_enter_closed_set() {

        let mut closed_set = PartialKeyedClosedSet::<TestNode>::default();

        let node_1 = Arc::<TestNode>::new(
            TestNode{
                graph_index: 0,
                cost: 10,
                remaining_cost_estimate: 6,
                parent: None
            }
        );

        assert!(matches!(closed_set.status(&node_1), ClosedStatus::Open));
        assert!(matches!(closed_set.close(&node_1), CloseResult::Closed));
        assert!(matches!(closed_set.status(&node_1), ClosedStatus::Closed(_)));

        let node_2 = Arc::<TestNode>::new(
            TestNode {
                graph_index: 0,
                cost: 12,
                remaining_cost_estimate: 6,
                parent: Some(node_1.clone())
            }
        );

        assert!(matches!(closed_set.status(&node_2), ClosedStatus::Closed(_)));
        assert!(matches!(closed_set.close(&node_2), CloseResult::Prior(_)));

        let node_3 = Arc::<TestNode>::new(
            TestNode {
                graph_index: 1,
                cost: 2,
                remaining_cost_estimate: 3,
                parent: Some(node_2.clone())
            }
        );

        assert!(matches!(closed_set.status(&node_3), ClosedStatus::Open));
        assert!(matches!(closed_set.close(&node_3), CloseResult::Closed));
        assert!(matches!(closed_set.status(&node_3), ClosedStatus::Closed(_)));
    }
}
