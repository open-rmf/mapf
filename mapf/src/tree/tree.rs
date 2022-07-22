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

use crate::expander::{Aimless, Closable};
use crate::node::{Weighted, CostCmp, ClosedSet};
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::sync::Arc;
use num::Zero;

/// Tree implements a simple Dijkstra search algorithm. The tree is rooted at a
/// start node and expands according to the provided expander.
///
/// For a bidirectional variant that leverages the tree while maintaining its
/// search effort for efficient reuse, see the Garden class in the tree module.
pub struct Tree<E: Aimless<Node: Weighted> + Closable> {
    closed_set: <E as Closable>::ClosedSet,
    queue: BinaryHeap<Reverse<CostCmp<E::Node>>>,
    expander: Arc<E>,
}

impl<E: Aimless<Node: Weighted> + Closable> Tree<E> {

    pub fn new(root: Arc<E::Node>, expander: Arc<E>) -> Self {
        let mut queue = BinaryHeap::new();
        queue.push(Reverse(CostCmp(root)));
        Self{closed_set: Default::default(), queue, expander}
    }

    /// Inspect the current closed set of the tree. Each item in this set
    /// contains a node that represents the shortest path from the tree's root
    /// to the node's state.
    pub fn closed(&self) -> &<E as Closable>::ClosedSet {
        &self.closed_set
    }

    /// Grow the current tree. This returns an iterator because if there is ever
    /// a tie for the cost of the top node, then this function will grow each
    /// tying node. When growing bidirectional trees and searching for a
    /// connection, it is important to consider all tying top nodes instead of
    /// only considering the one that happens to be first in the queue.
    pub fn grow<'a>(&'a mut self) -> Growth<'a, E> {
        let expected_cost = self.queue.peek().map(|n| n.0.0.cost()).unwrap_or(<E::Node as Weighted>::Cost::zero());
        Growth{tree: self, expected_cost}
    }

    pub fn is_exhausted(&self) -> bool {
        self.queue.is_empty()
    }
}

pub struct Growth<'a, E: Aimless<Node: Weighted> + Closable> {
    tree: &'a mut Tree<E>,
    expected_cost: <E::Node as Weighted>::Cost,
}

impl<'a, N: Weighted, E: Aimless<Node=N> + Closable> Iterator for Growth<'a, E> {
    type Item = Result<Arc<N>, E::AimlessError>;

    fn next(&mut self) -> Option<Result<Arc<N>, E::AimlessError>> {
        while !self.tree.queue.is_empty() {
            if let Some(top) = self.tree.queue.peek() {
                // First check if the cost is tied for the expected cost.
                // If it is not tied, then we should not modify the queue anymore.
                if top.0.0.cost().eq(&self.expected_cost) {
                    if let Some(top) = self.tree.queue.pop().map(|n| n.0.0) {
                        if self.tree.closed_set.close(&top).accepted() {
                            for child in self.tree.expander.aimless_expand(&top) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        node::PartialKeyed,
        expander::InitTargeted,
        motion::r2::{
            Position, timed_position::LineFollow,
            graph_search::make_default_expander,
        },
        directed::simple::SimpleGraph,
    };
    use std::collections::HashSet;

    fn make_test_graph() -> SimpleGraph<Position> {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
        */

        let mut vertices = Vec::<Position>::new();
        vertices.push(Position::new(0.0, 0.0)); // 0
        vertices.push(Position::new(1.0, 0.0)); // 1
        vertices.push(Position::new(2.0, 0.0)); // 2
        vertices.push(Position::new(3.0, 0.0)); // 3
        vertices.push(Position::new(1.0, -1.0)); // 4
        vertices.push(Position::new(2.0, -1.0)); // 5
        vertices.push(Position::new(3.0, -1.0)); // 6
        vertices.push(Position::new(2.0, -2.0)); // 7
        vertices.push(Position::new(3.0, -2.0)); // 8

        let mut edges = Vec::<Vec::<usize>>::new();
        edges.resize(9, Vec::new());
        let mut add_bidir_edge = |v0: usize, v1: usize| {
            edges.get_mut(v0).unwrap().push(v1);
            edges.get_mut(v1).unwrap().push(v0);
        };
        add_bidir_edge(0, 1);
        add_bidir_edge(1, 2);
        add_bidir_edge(2, 3);
        add_bidir_edge(2, 4);
        add_bidir_edge(3, 6);
        add_bidir_edge(4, 5);
        add_bidir_edge(5, 7);
        add_bidir_edge(7, 8);

        return SimpleGraph::new(vertices, edges);
    }

    fn make_test_extrapolation() -> LineFollow{
        return LineFollow::new(1.0f64).unwrap();
    }

    #[test]
    fn test_r2_tree_expansion() {
        let expander = Arc::new(make_default_expander(
            Arc::new(make_test_graph()),
            Arc::new(make_test_extrapolation())
        ));

        let mut visited: HashSet<usize> = HashSet::default();
        for start in expander.start(&0, &0) {
            let start = start.unwrap();
            let mut tree = Tree::new(
                start, expander.clone()
            );

            while !tree.is_exhausted() {
                for node in tree.grow() {
                    assert!(node.is_ok());
                    if let Ok(n) = node {
                        assert!(visited.insert(*n.partial_key().unwrap()));
                    }
                }
            }
        }

        for i in 0..=8 {
            assert!(visited.contains(&i));
        }
    }
}
