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

use std::hash::Hash;
use std::sync::Arc;
use std::collections::hash_map::{self, HashMap, Entry};
use std::ops::Add;
use std::cmp::Ord;
use num::traits::Zero;
use std::cmp::Ordering;

/// A trait that describes what is needed to define a cost.
pub trait Cost: Ord + Add<Output=Self> + Sized + Copy + Zero + std::fmt::Debug { }
impl<T: Ord + Add<Output=Self> + Sized + Copy + Zero + std::fmt::Debug> Cost for T { }

/// The generic trait for a Node. This contains the minimal generalized
/// information that most algorithms will need from a search node in order to
/// solve a search problem.
pub trait Node : Sized {
    type Cost: Cost;
    type ClosedSet: ClosedSet<Self>;

    fn cost(&self) -> Self::Cost;
    fn parent(&self) -> &Option<Arc<Self>>;
}

/// A subset of Node which has an informed estimate of how far it is from its goal
pub trait Informed: Node {

    /// Get the estimate of the remaining cost from the goal
    fn remaining_cost_estimate(&self) -> Self::Cost;

    /// Get the estimate of the total cost from the goal. This should always be
    /// equal to cost() + remaining_cost_estimate(), but we make it a separate
    /// function so that implementers can choose to micro-optimize by adding and
    /// saving the result inside of the node itself.
    fn total_cost_estimate(&self) -> Self::Cost {
        self.cost() + self.remaining_cost_estimate()
    }
}

/// Trait for nodes that can expand in reverse. HashOption is required for both
/// Self and Self::Reverse because the keys are used to match up when a forward
/// and reverse node are connected. Nodes that return None for their key cannot
/// connect to a complementary Node.
pub trait Reversible: Node + PartialKeyed {
    /// The reversed type of this Node. Its key type must match the key type of
    /// the forward node so that they can be compared against each other.
    type Reverse: Node<Cost=Self::Cost> + PartialKeyed<Key=<Self as PartialKeyed>::Key>;
}

/// The result of attempting to add a node to the Closed Set.
pub enum CloseResult<N: Node> {
    /// The node was successfully closed. No other node was previously closed
    /// with an equivalent state.
    Closed,

    /// A node with an equivalent state was previous closed. This contains a
    /// reference to that node.
    Prior(Arc<N>)
}

impl<N: Node> CloseResult<N> {
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

/// The result of checking whether an equivalent node is already in the
/// ClosedSet.
pub enum ClosedStatus<N: Node> {
    Open,
    Closed(Arc<N>)
}

impl<N: Node> ClosedStatus<N> {
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

/// The generic trait of a Closed Set. "Closed Sets" are used to keep avoid
/// unnecessary search effort. They keep track of the lowest cost node which has
/// visited a certain location.
pub trait ClosedSet<NodeType: Node>: Default {

    type Iter<'a>: Iterator<Item=&'a Arc<NodeType>> where Self: 'a, NodeType: 'a;

    /// Tell the closed set to close this node.
    fn close(&mut self, node: &Arc<NodeType>) -> CloseResult<NodeType>;

    /// Check whether an equivalent node has been closed.
    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType>;

    fn iter<'a>(&'a self) -> Self::Iter<'a>;
}

/// A trait for nodes that can sometimes provide a unique key but other times
/// cannot.
pub trait PartialKeyed {

    type Key: Hash + Eq + Clone;

    /// Attempt to get a key that uniquely identifies the state of this node.
    /// If the node cannot be uniquely identified by a key, this will return
    /// None.
    #[must_use]
    fn key(&self) -> Option<Self::Key>;
}

/// A trait for nodes that can always provide a unique key. The PartialKeyed
/// trait must be implemented, and its implementation is what will be used. If
/// PartialKeyed ever returns None, then generics which use the Keyed trait may
/// panic.
pub trait Keyed: PartialKeyed { }

pub struct PartialKeyedClosedSet<NodeType: PartialKeyed> {
    closed_set: HashMap<NodeType::Key, Arc<NodeType>>
}

impl<NodeType: PartialKeyed> Default for PartialKeyedClosedSet<NodeType> {
    fn default() -> Self {
        return PartialKeyedClosedSet {
            closed_set: Default::default()
        }
    }
}

impl<NodeType> ClosedSet<NodeType> for PartialKeyedClosedSet<NodeType>
where
    NodeType: Node + PartialKeyed
{
    type Iter<'a> where NodeType: 'a = HashOptionClosedSetIter<'a, NodeType>;

    fn close(&mut self, node: &Arc<NodeType>) -> CloseResult<NodeType> {
        if let Some(key) = node.key() {
            let entry = self.closed_set.entry(key);
            match entry {
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

    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType> {
        if let Some(key) = node.key() {
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

        return ClosedStatus::Open;
    }

    fn iter<'a>(&'a self) -> Self::Iter<'a> {
        HashOptionClosedSetIter{iter: self.closed_set.iter()}
    }
}

pub struct HashOptionClosedSetIter<'a, NodeType: PartialKeyed> {
    iter: hash_map::Iter<'a, NodeType::Key, Arc<NodeType>>,
}

impl<'a, NodeType: PartialKeyed + 'a> Iterator for HashOptionClosedSetIter<'a, NodeType> {
    type Item = &'a Arc<NodeType>;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|(_, n)| n)
    }
}

pub struct CostCmp<N: Node>(pub Arc<N>);

impl<N: Node> Ord for CostCmp<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        return self.0.cost().cmp(&other.0.cost());
    }
}

impl<N: Node> PartialOrd for CostCmp<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        return self.0.cost().partial_cmp(&other.0.cost());
    }
}

impl<N: Node> PartialEq for CostCmp<N> {
    fn eq(&self, other: &Self) -> bool {
        return self.0.cost().eq(&other.0.cost());
    }
}

impl<N: Node> Eq for CostCmp<N> { }

#[derive(Debug, Clone)]
pub struct TotalCostEstimateCmp<N: Informed>(pub Arc<N>);

impl<N: Informed> Ord for TotalCostEstimateCmp<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        return self.0.total_cost_estimate().cmp(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> PartialOrd for TotalCostEstimateCmp<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        return self.0.total_cost_estimate().partial_cmp(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> PartialEq for TotalCostEstimateCmp<N> {
    fn eq(&self, other: &Self) -> bool {
        return self.0.total_cost_estimate().eq(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> Eq for TotalCostEstimateCmp<N> { }

pub type KeyOf<N> = <N as PartialKeyed>::Key;

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

        fn key(&self) -> Option<Self::Key> {
            Some(self.graph_index)
        }
    }

    impl Node for TestNode {
        type ClosedSet = PartialKeyedClosedSet<Self>;
        type Cost = u64;

        fn cost(&self) -> u64 {
            return self.cost;
        }
        fn parent(&self) -> &Option<Arc<Self>> {
            return &self.parent;
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
