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

use crate::{
    algorithm::{MinimumCostBound, Path, QueueLength},
    domain::{ClosedSet, ClosedStatus},
    error::ThisError,
};
use std::{
    cmp::{Ordering, Reverse},
    collections::BinaryHeap,
};

/// A data structure for storing, managing, and growing a tree of nodes.
#[derive(Debug)]
pub struct Tree<Closed, Node, Cost> {
    /// The set of tree nodes that have been closed. Closing a node usually
    /// suggests that the node represents the shortest path to the node's state
    pub closed_set: Closed,
    /// The frontier queue for the tree, representing edge nodes of the tree
    /// that should be expanded next, in order from highest to lowest priority.
    pub queue: TreeFrontierQueue<Cost>,
    /// The memory arena for the tree which keeps track of all node data.
    pub arena: Vec<Node>,
}

impl<Closed, Node: TreeNode> Tree<Closed, Node, Node::Cost> {
    pub fn new(closed_set: Closed) -> Self
    where
        Node: TreeNode,
        Node::Cost: Ord,
    {
        Self {
            closed_set,
            queue: Default::default(),
            arena: Default::default(),
        }
    }

    pub fn push_node(&mut self, node: Node) -> Result<(), TreeError>
    where
        Node: TreeNode,
        Closed: ClosedSet<Node::State, usize>,
        Node::Cost: Ord,
    {
        if let ClosedStatus::Closed(prior) = self.closed_set.status(node.state()) {
            if let Some(prior) = self.arena.get(*prior) {
                if prior.cost() <= node.cost() {
                    // The state is already closed with a lower-cost node, so
                    // we should not push this new node.
                    return Ok(());
                }
            } else {
                // The closed set is referencing a node that does not exist in
                // the memory arena. This is a major bug.
                return Err(TreeError::BrokenReference(*prior));
            }
        }

        let node_id = self.arena.len();
        let evaluation = node.queue_evaluation();
        let bias = node.queue_bias();
        self.arena.push(node);
        self.queue.push(Reverse(TreeQueueTicket {
            node_id,
            bias,
            evaluation,
        }));
        Ok(())
    }
}

pub trait TreeNode {
    /// The type used to describe the state of the node.
    type State;

    /// The type of action that can be taken from one node to another.
    type Action;

    /// The type used to decide whether to prefer one node over another. Lower
    /// values are preferable.
    type Cost;

    /// Get the state of the node.
    fn state(&self) -> &Self::State;

    /// If the node has a parent, get the identity of that parent and the action
    /// used to arrive from it.
    fn parent(&self) -> Option<(usize, &Self::Action)>;

    /// Get the actual cost of arriving at this node from its initial state.
    fn cost(&self) -> Self::Cost;

    /// Evaluate this node for its placement in the search queue. For an
    /// uninformed node this would simply return its aggregated cost. For an
    /// informed node this would return its aggregated cost plus its remaining
    /// cost estimate.
    fn queue_evaluation(&self) -> Self::Cost;

    /// Give a bias to this node. When queue_evaluation is exactly equal, the
    /// node will be ordered by this bias instead. Higher bias will push it
    /// later in the queue. For an informed node this could be its remaining
    /// cost estimate.
    fn queue_bias(&self) -> Option<Self::Cost>;
}

#[derive(Debug, Clone, Copy)]
pub struct TreeQueueTicket<Cost> {
    pub evaluation: Cost,
    pub bias: Option<Cost>,
    pub node_id: usize,
}

pub type TreeFrontierQueue<Cost> = BinaryHeap<Reverse<TreeQueueTicket<Cost>>>;

impl<Cost: PartialEq> PartialEq for TreeQueueTicket<Cost> {
    fn eq(&self, other: &Self) -> bool {
        self.evaluation.eq(&other.evaluation)
    }
}

impl<Cost: Eq> Eq for TreeQueueTicket<Cost> {}

impl<Cost: PartialOrd> PartialOrd for TreeQueueTicket<Cost> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.evaluation.partial_cmp(&other.evaluation) {
            // TODO(@mxgrey): Let downstream users to define criteria for
            // deciding when two evaluations are close enough that the biases
            // should be compared instead. Small floating point numerical
            // residue could cause one evaluation to be unfairly favored over
            // another that should be considered equal.
            Some(Ordering::Equal) => {
                if let (Some(l), Some(r)) = (&self.bias, &other.bias) {
                    l.partial_cmp(r)
                } else {
                    Some(Ordering::Equal)
                }
            }
            value => value,
        }
    }
}

impl<Cost: Ord> Ord for TreeQueueTicket<Cost> {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.evaluation.cmp(&other.evaluation) {
            Ordering::Equal => {
                if let (Some(l), Some(r)) = (&self.bias, &other.bias) {
                    l.cmp(r)
                } else {
                    Ordering::Equal
                }
            }
            value => value,
        }
    }
}

pub trait NodeContainer<N: TreeNode> {
    fn get_node(&self, index: usize) -> Result<&N, TreeError>;
    fn retrace(&self, index: usize) -> Result<Path<N::State, N::Action, N::Cost>, TreeError>;
}

impl<N: TreeNode> NodeContainer<N> for Vec<N>
where
    N::State: Clone,
    N::Action: Clone,
{
    fn get_node(&self, index: usize) -> Result<&N, TreeError> {
        self.get(index)
            .ok_or_else(|| TreeError::BrokenReference(index))
    }

    fn retrace(&self, node_id: usize) -> Result<Path<N::State, N::Action, N::Cost>, TreeError> {
        let total_cost = self.get_node(node_id)?.cost();
        let mut initial_node_id = node_id;
        let mut next_node_id = Some(node_id);
        let mut sequence = Vec::new();
        while let Some(current_node_id) = next_node_id {
            initial_node_id = current_node_id;
            let node = self.get_node(current_node_id)?;
            next_node_id = if let Some((parent_id, action)) = node.parent() {
                sequence.push((action.clone(), node.state().clone()));
                Some(parent_id)
            } else {
                None
            };
        }

        sequence.reverse();

        let initial_state = self.get_node(initial_node_id)?.state().clone();
        Ok(Path {
            initial_state,
            sequence,
            total_cost,
        })
    }
}

#[derive(ThisError, Debug)]
pub enum TreeError {
    #[error(
        "A node [{0}] is referenced but does not exist in the search memory. \
    This is a critical implementation error, please report this to the mapf developers."
    )]
    BrokenReference(usize),
}

impl<Closed, Node: TreeNode> QueueLength for Tree<Closed, Node, Node::Cost> {
    fn queue_length(&self) -> usize {
        self.queue.len()
    }
}

impl<Closed, Node: TreeNode> MinimumCostBound for Tree<Closed, Node, Node::Cost>
where
    Node::Cost: Clone,
{
    type Cost = Node::Cost;
    fn minimum_cost_bound(&self) -> Option<Self::Cost> {
        self.queue.peek().map(|n| n.0.evaluation.clone())
    }
}
