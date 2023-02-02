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
    error::Error,
    algorithm::{Algorithm, Coherent, Solvable, Status, MinimumCostBound, Measure},
    domain::{
        Domain, Activity, Dynamics, Weighted, Initializable, Informed, Satisfiable,
        Closable, ClosedSet, ClosedStatus, CloseResult,
    },
};
use std::{
    cmp::{Reverse, PartialOrd},
    collections::BinaryHeap,
    ops::Add,
};
use thiserror::Error as ThisError;

pub struct Solution<State, Action, Cost> {
    pub initial_state: State,
    pub sequence: Vec<(Action, State)>,
    pub total_cost: Cost,
}

#[derive(Default, Debug)]
pub struct AStar<D> {
    pub domain: D,
}

impl<D> AStar<D> {
    pub fn domain_err(err: impl Into<D::Error>) -> AStarSearchError<D::Error>
    where
        D: Domain,
        D::Error: Error,
    {
        AStarSearchError::Domain(err.into())
    }

    pub fn algo_err(err: AStarImplError) -> AStarSearchError<D::Error>
    where
        D: Domain,
        D::Error: Error,
    {
        AStarSearchError::Algorithm(err)
    }
}

impl<D> Algorithm for AStar<D>
where
    D: Domain
    + Closable<D::State, usize>
    + Weighted<D::State, D::Action>,
{
    type Memory = Memory<D::ClosedSet, D::State, D::Action, D::Cost>;
}

impl<D, Start, Goal> Coherent<Start, Goal> for AStar<D>
where
    D: Domain
    + Initializable<Start, D::State>
    + Closable<D::State, usize>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
    D::Error: Error,
    D::InitialError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::InformedError: Into<D::Error>,
{
    type InitError = AStarSearchError<D::Error>;

    fn initialize(
        &self,
        start: Start,
        goal: &Goal,
    ) -> Result<Self::Memory, Self::InitError> {
        let mut memory = Memory {
            closed_set: self.domain.new_closed_set(),
            queue: Default::default(),
            arena: Default::default(),
        };

        for state in self.domain.initialize(start) {
            let state = state.map_err(Self::domain_err)?;
            let cost = match self.domain.initial_cost(&state).map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };
            let remaining_cost_estimate = match self.domain.remaining_cost_estimate(
                &state, &goal
            ).map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };

            memory.push_node(Node {
                cost,
                remaining_cost_estimate,
                state,
                parent: None,
            }).map_err(Self::algo_err)?;
        }

        Ok(memory)
    }
}

impl<D, Goal> Solvable<Goal> for AStar<D>
where
    D: Domain
    + Closable<D::State, usize>
    + Activity<D::State, ActivityAction=D::Action>
    + Dynamics<D::State, D::Action>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>
    + Satisfiable<D::State, Goal>,
    D::State: Clone,
    D::Action: Clone,
    D::Error: Error,
    D::Cost: Ord + Add<Output = D::Cost> + Clone,
    D::SatisfactionError: Into<D::Error>,
    D::ActivityError: Into<D::Error>,
    D::DynamicsError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::InformedError: Into<D::Error>,
{
    type Solution = Solution<D::State, D::Action, D::Cost>;
    type StepError = AStarSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<Status<Self::Solution>, Self::StepError> {
        let top_id = match memory.queue.pop().map(|x| x.0) {
            Some(top) => top,
            None => return Ok(Status::Impossible),
        }.node_id;

        let top = memory.arena.get_node(top_id).map_err(Self::algo_err)?;
        if self.domain.is_satisfied(&top.state, goal).map_err(Self::domain_err)? {
            return memory
                .retrace(top_id)
                .map(Into::into)
                .map_err(Self::algo_err);
        }

        if let CloseResult::Rejected { prior, .. } = memory.closed_set.close(
            top.state.clone(), top_id
        ) {
            let prior_node = memory.arena.get_node(*prior).map_err(Self::algo_err)?;
            if prior_node.cost <= top.cost {
                // The state we are attempting to expand has already been closed
                // in the past by a lower cost node, so we will not expand from
                // this top node. Instead we will finish this iteration.
                return Ok(Status::Incomplete);
            }

            // The top node has a lower cost so it should replace the node that
            // previously closed this state.
            *prior = top_id;
        }

        // Expand from the top node
        let parent_state = top.state.clone();
        let parent_cost = top.cost.clone();
        for next in self.domain.choices(parent_state.clone()) {
            let action = next.map_err(Self::domain_err)?;

            let child_state = match self.domain.advance(
                parent_state.clone(), &action
            ).map_err(Self::domain_err)? {
                Some(s) => s,
                None => continue,
            };

            let cost = match self.domain
                .cost(&parent_state, &action, &child_state)
                .map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            } + parent_cost.clone();

            let remaining_cost_estimate = match self.domain
                .remaining_cost_estimate(&child_state, goal)
                .map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };

            memory.push_node(Node {
                state: child_state,
                cost,
                remaining_cost_estimate,
                parent: Some((top_id, action)),
            }).map_err(Self::algo_err)?;
        }

        Ok(Status::Incomplete)
    }
}

pub struct QueueTicket<Cost> {
    total_cost_estimate: Cost,
    node_id: usize,
}

pub struct Node<State, Action, Cost> {
    state: State,
    cost: Cost,
    remaining_cost_estimate: Cost,
    parent: Option<(usize, Action)>,
}

impl<Cost: PartialEq> PartialEq for QueueTicket<Cost> {
    fn eq(&self, other: &Self) -> bool {
        self.total_cost_estimate.eq(&other.total_cost_estimate)
    }
}

impl<Cost: Eq> Eq for QueueTicket<Cost> {}

impl<Cost: PartialOrd> PartialOrd for QueueTicket<Cost> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.total_cost_estimate.partial_cmp(&other.total_cost_estimate)
    }
}

impl<Cost: Ord> Ord for QueueTicket<Cost> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.total_cost_estimate.cmp(&other.total_cost_estimate)
    }
}

pub struct Memory<Closed, State, Action, Cost> {
    closed_set: Closed,
    queue: BinaryHeap<Reverse<QueueTicket<Cost>>>,
    arena: Vec<Node<State, Action, Cost>>,
}

impl<Closed, State: Clone, Action: Clone, Cost: Clone> Memory<Closed, State, Action, Cost> {
    pub fn retrace(&self, node_id: usize) -> Result<Solution<State, Action, Cost>, AStarImplError> {
        let total_cost = self.arena.get_node(node_id)?.cost.clone();
        let mut initial_node_id = node_id;
        let mut next_node_id = Some(node_id);
        let mut sequence = Vec::new();
        while let Some(current_node_id) = next_node_id {
            initial_node_id = current_node_id;
            let node = self.arena.get_node(node_id)?;
            if let Some((_, action)) = &node.parent {
                sequence.push((action.clone(), node.state.clone()));
            }

            next_node_id = node.parent.as_ref().map(|(parent, _)| *parent);
        }

        sequence.reverse();

        let initial_state = self.arena.get_node(initial_node_id)?.state.clone();
        Ok(Solution { initial_state, sequence, total_cost })
    }
}

impl<Closed, State, Action, Cost> Memory<Closed, State, Action, Cost> {
    pub fn queue(&self) -> &BinaryHeap<Reverse<QueueTicket<Cost>>> {
        &self.queue
    }

    pub fn push_node(&mut self, node: Node<State, Action, Cost>) -> Result<(), AStarImplError>
    where
        Closed: ClosedSet<State, usize>,
        Cost: Ord + Add<Output=Cost> + Clone,
    {
        if let ClosedStatus::Closed(prior) = self.closed_set.status(&node.state) {
            if let Some(prior) = self.arena.get(*prior) {
                if prior.cost <= node.cost {
                    // The state is already closed with a lower-cost node, so
                    // we should not push this new node.
                    return Ok(());
                }
            } else {
                // The closed set is referencing a node that does not exist in
                // the memory arena. This is a major bug.
                return Err(AStarImplError::BrokenReference(*prior));
            }
        }

        let node_id = self.arena.len();
        let total_cost_estimate = node.cost.clone() + node.remaining_cost_estimate.clone();
        self.arena.push(node);
        self.queue.push(Reverse(QueueTicket { node_id, total_cost_estimate }));
        Ok(())
    }
}

trait GetNode<N> {
    fn get_node(&self, index: usize) -> Result<&N, AStarImplError>;
}

impl<N> GetNode<N> for Vec<N> {
    fn get_node(&self, index: usize) -> Result<&N, AStarImplError> {
        self.get(index).ok_or(AStarImplError::BrokenReference(index))
    }
}

impl<Closed, State, Action, Cost> Measure for Memory<Closed, State, Action, Cost> {
    fn size(&self) -> usize {
        self.arena.len() * std::mem::size_of::<Node<State, Action, Cost>>()
    }
}

impl<Closed, State, Action, Cost: Clone> MinimumCostBound for Memory<Closed, State, Action, Cost> {
    type Cost = Cost;
    fn minimum_cost_bound(&self) -> Option<Self::Cost> {
        self.queue.peek().map(|n| n.0.total_cost_estimate.clone())
    }
}

#[derive(ThisError, Debug)]
pub enum AStarSearchError<D: Error> {
    #[error("An error occurred in the algorithm:\n{0}")]
    Algorithm(AStarImplError),
    #[error("An error occurred in the domain:\n{0}")]
    Domain(D),
}

#[derive(ThisError, Debug)]
pub enum AStarImplError {
    #[error("A node [{0}] is referenced but does not exist in the search memory. \
    This is a critical implementation error, please report this to the mapf developers.")]
    BrokenReference(usize),
}
