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

use super::algorithm::{self, Status};
use super::expander::{Expander, Initializable, Solvable, Closable, Goal, CostOf, InitErrorOf, ExpansionErrorOf, SolveErrorOf};
use super::Trace;
use super::node::{Informed, TotalCostEstimateCmp as NodeCmp, ClosedSet, CloseResult, ClosedStatus};
use std::collections::BinaryHeap;
use std::sync::Arc;
use std::cmp::Reverse;

pub struct Memory<N, E>
where
    N: Informed,
    E: Expander<Node=N> + Closable
{
    closed_set: <E as Closable>::ClosedSet,
    queue: BinaryHeap<Reverse<NodeCmp<N>>>,
    expander: Arc<E>,
    goal: E::Goal,
}

impl<N: Informed, E: Expander<Node=N> + Closable> Memory<N, E> {
    pub fn queue(&self) -> &BinaryHeap<Reverse<NodeCmp<N>>> {
        &self.queue
    }
}

impl<N, E> algorithm::Memory for Memory<N, E>
where
    N: Informed,
    E: Expander<Node=N> + Closable,
{
    fn node_count(&self) -> usize {
        return self.queue.len();
    }
}

impl<N, E> algorithm::WeightSorted<E> for Memory<N, E>
where
    N: Informed,
    E: Expander<Node=N> + Closable,
{
    fn top_cost_estimate(&self) -> Option<CostOf<E>> {
        self.queue.peek().map(|x| x.0.0.total_cost_estimate())
    }
}

#[derive(Default, Debug)]
pub struct Algorithm;

impl<N, E> algorithm::Algorithm<E> for Algorithm
where
    N: Informed,
    E: Expander<Node=N> + Closable + Solvable,
{
    type Memory = Memory<N, E>;
    type InitError = ();
    type StepError = ();

    fn initialize<S, T: Trace<N>>(
        &self,
        expander: Arc<E>,
        start: &S,
        goal: E::Goal,
        tracker: &mut T
    ) -> Result<Self::Memory, algorithm::InitError<Self::StepError, InitErrorOf<E, S>>>
    where E: Initializable<S>
    {

        let mut queue = BinaryHeap::default();
        for node in expander.start(start, Some(&goal)) {
            let node = node.map_err(algorithm::InitError::Expander)?;
            tracker.expanded_to(&node);
            queue.push(Reverse(NodeCmp(node)));
        }

        return Ok(Memory{
            closed_set: E::ClosedSet::default(),
            queue,
            expander,
            goal
        });
    }

    fn step<T: Trace<N>>(
        &self,
        storage: &mut Self::Memory,
        tracker: &mut T
    ) -> Result<Status<E>, algorithm::StepError<Self::StepError, ExpansionErrorOf<E>, SolveErrorOf<E>>> {

        if let Some(top) = storage.queue.pop().map(|x| x.0.0) {
            if storage.goal.is_satisfied(&top) {
                tracker.solution_found_from(&top);
                let solution = storage.expander.make_solution(&top).map_err(algorithm::StepError::Solve)?;
                return Ok(Status::Solved(solution));
            }

            if let CloseResult::Closed = storage.closed_set.close(&top) {
                tracker.expanded_from(&top);
                for next in storage.expander.expand(&top, Some(&storage.goal)) {
                    let next = next.map_err(algorithm::StepError::Expansion)?;
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
