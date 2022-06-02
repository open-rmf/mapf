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

use crate::expander::{Expander, ExpansionErrorOf};
use crate::node::{Node, CostCmp, ClosedSet};
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::sync::Arc;
use num::Zero;

/// Tree implements a simple Dijkstra search algorithm. The tree is rooted at a
/// start node and expands according to the provided expander.
///
/// For a bidirectional variant that leverages the tree while maintaining its
/// search effort for efficient reuse, see the Garden class in the tree module.
pub struct Tree<E: Expander> {
    closed_set: <E::Node as Node>::ClosedSet,
    queue: BinaryHeap<Reverse<CostCmp<E::Node>>>,
    expander: Arc<E>,
}

impl<E: Expander> Tree<E> {

    pub fn new(root: Arc<E::Node>, expander: Arc<E>) -> Self {
        let mut closed_set = <E::Node as Node>::ClosedSet::default();
        closed_set.close(&root);

        let mut queue = BinaryHeap::new();
        queue.push(Reverse(CostCmp(root)));

        Self{closed_set, queue, expander}
    }

    /// Inspect the current closed set of the tree. Each item in this set
    /// contains a node that represents the shortest path from the tree's root
    /// to the node's state.
    pub fn closed(&self) -> &<E::Node as Node>::ClosedSet {
        &self.closed_set
    }

    /// Grow the current tree. This returns an iterator because if there is ever
    /// a tie for the cost of the top node, then this function will grow each
    /// tying node. When growing bidirectional trees and searching for a
    /// connection, it is important to consider all tying top nodes instead of
    /// only considering the one that happens to be first in the queue.
    pub fn grow<'a>(&'a mut self) -> Growth<'a, E> {
        let expected_cost = self.queue.peek().map(|n| n.0.0.cost()).unwrap_or(<E::Node as Node>::Cost::zero());
        Growth{tree: self, expected_cost}
    }

    pub fn is_exhausted(&self) -> bool {
        self.queue.is_empty()
    }
}

pub struct Growth<'a, E: Expander> {
    tree: &'a mut Tree<E>,
    expected_cost: <E::Node as Node>::Cost,
}

impl<'a, N: Node, E: Expander<Node=N>> Iterator for Growth<'a, E> {
    type Item = Result<Arc<N>, ExpansionErrorOf<E>>;

    fn next(&mut self) -> Option<Result<Arc<N>, ExpansionErrorOf<E>>> {
        while !self.tree.queue.is_empty() {
            if let Some(top) = self.tree.queue.peek() {
                // First check if the cost is tied for the expected cost.
                // If it is not tied, then we should not modify the queue anymore.
                if top.0.0.cost().eq(&self.expected_cost) {
                    if let Some(top) = self.tree.queue.pop().map(|n| n.0.0) {
                        if self.tree.closed_set.close(&top).accepted() {
                            for child in self.tree.expander.expand(&top, None) {
                                match child {
                                    Ok(child) => {
                                        if self.tree.closed_set.status(&child).is_open() {
                                            self.tree.queue.push(Reverse(CostCmp(child)));
                                        }
                                    },
                                    Err(e) => {
                                        return Some(Err(e));
                                    }
                                }
                            }

                            return Some(Ok(top));
                        } else {
                            // The selected node was already closed, so we should
                            // move onto the next one instead of returning it.
                            continue;
                        }
                    }
                }
            }

            return None;
        }

        return None;
    }
}
