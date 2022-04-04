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

use super::algorithm;
use super::algorithm::Status;
use super::expander;
use super::expander::Goal;
use super::tracker::Tracker;
use super::node;
use super::node::{TotalCostEstimateCmp as NodeCmp, ClosedSet, CloseResult, ClosedStatus};
use std::collections::BinaryHeap;
use std::rc::Rc;

pub struct Storage<N, E>
where
    N: node::Informed,
    E: expander::Expander<Node=N>
{
    closed_set: <N as node::Node>::ClosedSet,
    queue: BinaryHeap<node::TotalCostEstimateCmp<N>>,
    expander: Rc<E>,
    goal: E::Goal,
}

impl<N, E> algorithm::Storage<E> for Storage<N, E>
where
    N: node::Informed,
    E: expander::Expander<Node=N>,
{
    fn node_count(&self) -> usize {
        return self.queue.len();
    }

    fn top_cost_estimate(&self) -> Option<expander::Cost<E>> {
        self.queue.peek().map(|x| x.0.total_cost_estimate())
    }
}

#[derive(Default)]
struct Algorithm;

impl<N, E> algorithm::Algorithm<E> for Algorithm
where
    N: node::Informed,
    E: expander::Expander<Node=N>,
{
    type Storage = Storage<N, E>;

    fn initialize<T: Tracker<N>>(
        &self,
        expander: Rc<E>,
        start: &E::Start,
        goal: E::Goal,
        tracker: &mut T
    ) -> Self::Storage {

        let mut queue = BinaryHeap::default();
        for node in expander.start(start, &goal) {
            queue.push(NodeCmp(node));
        }

        return Storage{
            closed_set: N::ClosedSet::default(),
            queue,
            expander,
            goal
        };
    }

    fn step<T: Tracker<N>>(
        &self,
        storage: &mut Self::Storage,
        options: &E::Options,
        tracker: &mut T
    ) -> Status<E> {

        if let Some(top) = storage.queue.pop().map(|x| x.0) {
            if storage.goal.is_satisfied(&top) {
                tracker.solution_found_from(&top);
                return Status::Solved(storage.expander.make_solution(&top, options));
            }

            if let CloseResult::Closed = storage.closed_set.close(&top) {
                for next in storage.expander.expand(&top, &storage.goal, options) {
                    if let ClosedStatus::Open = storage.closed_set.status(next.as_ref()) {
                        tracker.expanded_to(&next);
                        storage.queue.push(NodeCmp(next));
                    }
                }
            }

            return Status::Incomplete;
        }

        return Status::Impossible;
    }
}
