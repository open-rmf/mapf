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
use std::ops::Add;
use std::cmp::Ord;
use num::traits::Zero;
use std::cmp::Ordering;

/// A trait that describes what is needed to define a cost.
pub trait Cost: Ord + Add<Output=Self> + Sized + Copy + Zero { }
impl<T: Ord + Add<Output=Self> + Sized + Copy + Zero> Cost for T { }

/// The generic trait for a Node. This contains the minimal generalized
/// information that most algorithms will need from a search node in order to
/// solve a search problem.
pub trait Node : Sized {
    type Cost: Cost;
    type ClosedSet: ClosedSet<Self>;

    fn cost(&self) -> Self::Cost;
    fn parent(&self) -> &Option<Rc<Self>>;
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

/// The result of attempting to add a node to the Closed Set.
pub enum CloseResult<NodeType: Node> {
    /// The node was successfully closed. No other node was previously closed
    /// with an equivalent state.
    Closed,

    /// A node with an equivalent state was previous closed. This contains a
    /// reference to that node.
    Prior(Rc<NodeType>)
}

/// The result of checking whether an equivalent node is already in the
/// ClosedSet.
pub enum ClosedStatus<NodeType: Node> {
    Open,
    Closed(Rc<NodeType>)
}

/// The generic trait of a Closed Set. "Closed Sets" are used to keep avoid
/// unnecessary search effort. They keep track of the lowest cost node which has
/// visited a certain location.
pub trait ClosedSet<NodeType: Node>: Default {

    /// Tell the closed set to close this node.
    fn close(&mut self, node: &Rc<NodeType>) -> CloseResult<NodeType>;

    /// Check whether an equivalent node has been closed.
    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType>;
}

/// An alternative to Hash for objects that can sometimes be hashed but other
/// times cannot.
pub trait HashOption {
    /// Attempt to hash the object. Returns false if the object was not hashable.
    #[must_use]
    fn hash_opt<H: Hasher>(&self, state: &mut H) -> bool;
}

impl<T: Hash> HashOption for T {
    fn hash_opt<H: Hasher>(&self, state: &mut H) -> bool {
        self.hash(state);
        return true;
    }
}

pub struct HashOptionClosedSet<NodeType: HashOption> {
    closed_set: std::collections::HashMap<u64, Rc<NodeType>>
}

impl<NodeType: HashOption> Default for HashOptionClosedSet<NodeType> {
    fn default() -> Self {
        return HashOptionClosedSet {
            closed_set: std::collections::HashMap::<u64, Rc<NodeType>>::default()
        }
    }
}

impl<NodeType> ClosedSet<NodeType> for HashOptionClosedSet<NodeType>
where
    NodeType: Node + HashOption
{
    fn close(&mut self, node: &Rc<NodeType>) -> CloseResult<NodeType> {
        // It would be nice to use a HashSet instead of a HashMap, but the
        // HashSet API does not have an equivalent to the HashMap's Entry API,
        // which makes this function far more efficient.
        let mut hasher = self.closed_set.hasher().build_hasher();
        if !node.hash_opt(&mut hasher) {
            return CloseResult::Closed;
        }

        let key = hasher.finish();
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

    fn status(&self, node: &NodeType) -> ClosedStatus<NodeType> {
        let mut hasher = self.closed_set.hasher().build_hasher();
        if !node.hash_opt(&mut hasher) {
            return ClosedStatus::Open;
        }
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

#[derive(Debug, Clone)]
pub struct TotalCostEstimateCmp<N: Informed>(pub Rc<N>);

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


#[cfg(test)]
mod tests {

    use super::*;

    struct TestNode {
        graph_index: usize,
        cost: u64,
        remaining_cost_estimate: u64,
        parent: Option<Rc<Self>>,
    }

    impl Hash for TestNode {
        fn hash<H: Hasher>(&self, state: &mut H) {
            state.write_usize(self.graph_index);
        }
    }

    impl Node for TestNode {
        type ClosedSet = HashOptionClosedSet<Self>;
        type Cost = u64;

        fn cost(&self) -> u64 {
            return self.cost;
        }
        fn parent(&self) -> &Option<Rc<Self>> {
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

        let mut closed_set = HashOptionClosedSet::<TestNode>::default();

        let node_1 = Rc::<TestNode>::new(
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

        let node_2 = Rc::<TestNode>::new(
            TestNode {
                graph_index: 0,
                cost: 12,
                remaining_cost_estimate: 6,
                parent: Some(node_1.clone())
            }
        );

        assert!(matches!(closed_set.status(&node_2), ClosedStatus::Closed(_)));
        assert!(matches!(closed_set.close(&node_2), CloseResult::Prior(_)));

        let node_3 = Rc::<TestNode>::new(
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
