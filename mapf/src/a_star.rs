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
use super::expander::{Goal, CostOf};
use super::tracker::Tracker;
use super::node::{self, TotalCostEstimateCmp as NodeCmp, ClosedSet, CloseResult, ClosedStatus};
use std::collections::BinaryHeap;
use std::sync::Arc;
use std::cmp::Reverse;

pub struct Storage<N, E>
where
    N: node::Informed,
    E: expander::Expander<Node=N>
{
    closed_set: <N as node::Node>::ClosedSet,
    queue: BinaryHeap<Reverse<node::TotalCostEstimateCmp<N>>>,
    expander: Arc<E>,
    goal: E::Goal,
}

impl<N: node::Informed, E: expander::Expander<Node=N>> Storage<N, E> {
    pub fn queue(&self) -> &BinaryHeap<Reverse<node::TotalCostEstimateCmp<N>>> {
        &self.queue
    }
}

impl<N, E> algorithm::Storage<E> for Storage<N, E>
where
    N: node::Informed,
    E: expander::Expander<Node=N>,
{
    fn node_count(&self) -> usize {
        return self.queue.len();
    }

    fn top_cost_estimate(&self) -> Option<CostOf<E>> {
        self.queue.peek().map(|x| x.0.0.total_cost_estimate())
    }
}

#[derive(Default)]
pub struct Algorithm;

impl<N, E> algorithm::Algorithm<E> for Algorithm
where
    N: node::Informed,
    E: expander::Expander<Node=N>,
{
    type Storage = Storage<N, E>;
    type Error = ();

    fn initialize<T: Tracker<N>>(
        &self,
        expander: Arc<E>,
        start: &E::Start,
        goal: E::Goal,
        tracker: &mut T
    ) -> Result<Self::Storage, algorithm::Error<E, Self>> {

        let mut queue = BinaryHeap::default();
        for node in expander.start(start, Some(&goal)) {
            let node = node.map_err(algorithm::Error::Expander)?;
            tracker.expanded_to(&node);
            queue.push(Reverse(NodeCmp(node)));
        }

        return Ok(Storage{
            closed_set: N::ClosedSet::default(),
            queue,
            expander,
            goal
        });
    }

    fn step<T: Tracker<N>>(
        &self,
        storage: &mut Self::Storage,
        tracker: &mut T
    ) -> Result<Status<E>, algorithm::Error<E, Self>> {

        if let Some(top) = storage.queue.pop().map(|x| x.0.0) {
            if storage.goal.is_satisfied(&top) {
                tracker.solution_found_from(&top);
                let solution = storage.expander.make_solution(&top).map_err(algorithm::Error::Expander)?;
                return Ok(Status::Solved(solution));
            }

            if let CloseResult::Closed = storage.closed_set.close(&top) {
                tracker.expanded_from(&top);
                for next in storage.expander.expand(&top, Some(&storage.goal)) {
                    let next = next.map_err(algorithm::Error::Expander)?;
                    if let ClosedStatus::Open = storage.closed_set.status(next.as_ref()) {
                        tracker.expanded_to(&next);
                        storage.queue.push(Reverse(NodeCmp(next)));
                    }
                }
            }

            return Ok(Status::Incomplete);
        }

        return Ok(Status::Impossible);
    }
}
