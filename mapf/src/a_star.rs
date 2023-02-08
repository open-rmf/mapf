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
        Domain, Activity, Weighted, Initializable, Informed, Satisfiable,
        Closable, ClosedSet, ClosedStatus, CloseResult, Connectable,
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

/// The AStar algorithm can be used on domains that implement the following traits:
/// * [`Initializable`]
/// * [`Closable`]
/// * [`Activity`]
/// * [`Weighted`]
/// * [`Informed`]
/// * [`Satisfiable`]
///
/// The following templates implement these traits:
/// * [`InformedGraphMotion`]
#[derive(Default, Debug)]
pub struct AStar<D>(D);

/// The AStarConnect algorithm is a variation on AStar that can attempt to find
/// a connection directly to the goal each time a node is expanded. In addition
/// to required traits of [`AStar`], the domain must also implement:
/// * [`Connectable`] as `Connectable<D::State, Goal>`
pub struct AStarConnect<D>(D);

pub struct Memory<Closed, State, Action, Cost> {
    closed_set: Closed,
    queue: BinaryHeap<Reverse<QueueTicket<Cost>>>,
    arena: Vec<Node<State, Action, Cost>>,
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

impl<D> AStar<D>
where
    D: Domain
    + Closable<D::State>
    + Weighted<D::State, D::Action>,
    D::State: Clone,
    D::Action: Clone,
    D::Error: Error,
    D::WeightedError: Into<D::Error>,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
{
    #[inline]
    fn initialize_impl<Start, Goal>(
        domain: &D,
        start: Start,
        goal: &Goal,
    ) -> Result<<Self as Algorithm>::Memory, AStarSearchError<D::Error>>
    where
        D: Initializable<Start, D::State>
        + Informed<D::State, Goal, CostEstimate=D::Cost>,
        D::InitialError: Into<D::Error>,
        D::InformedError: Into<D::Error>,
    {
        let mut memory = Memory {
            closed_set: domain.new_closed_set(),
            queue: Default::default(),
            arena: Default::default(),
        };

        for state in domain.initialize(start) {
            let state = state.map_err(Self::domain_err)?;
            let cost = match domain.initial_cost(&state).map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };
            let remaining_cost_estimate = match domain.remaining_cost_estimate(
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

    #[inline]
    fn choose_top<Goal>(
        domain: &D,
        closed_set: &mut D::ClosedSet<usize>,
        queue: &mut BinaryHeap<Reverse<QueueTicket<D::Cost>>>,
        arena: &Vec<Node<D::State, D::Action, D::Cost>>,
        goal: &Goal,
    ) -> Result<Flow<(usize, Node<D::State, D::Action, D::Cost>), D>, AStarSearchError<D::Error>>
    where
        D: Satisfiable<D::State, Goal>,
        D::SatisfactionError: Into<D::Error>,
    {
        let top_id = match queue.pop().map(|x| x.0) {
            Some(top) => top,
            None => return Ok(Flow::Return(Status::Impossible)),
        }.node_id;

        let top = arena.get_node(top_id).map_err(Self::algo_err)?;
        if domain.is_satisfied(&top.state, goal).map_err(Self::domain_err)? {
            let solution = arena
                .retrace(top_id)
                .map(Into::into)
                .map_err(Self::algo_err)?;
            return Ok(Flow::Return(Status::Solved(solution)));
        }

        if let CloseResult::Rejected { prior, .. } = closed_set.close(
            top.state.clone(), top_id
        ) {
            let prior_node = arena.get_node(*prior).map_err(Self::algo_err)?;
            if prior_node.cost <= top.cost {
                // The state we are attempting to expand has already been closed
                // in the past by a lower cost node, so we will not expand from
                // this top node. Instead we will finish this iteration.
                return Ok(Flow::Return(Status::Incomplete));
            }

            // The top node has a lower cost so it should replace the node that
            // previously closed this state.
            *prior = top_id;
        }

        Ok(Flow::Continue((top_id, top.clone())))
    }

    #[inline]
    fn expand_from_parent<Goal>(
        domain: &D,
        memory: &mut <Self as Algorithm>::Memory,
        parent_id: usize,
        parent: &Node<D::State, D::Action, D::Cost>,
        goal: &Goal,
    ) -> Result<(), AStarSearchError<D::Error>>
    where
        D: Activity<D::State>,
        D::ActivityAction: Into<D::Action>,
        D::ActivityError: Into<D::Error>,
        D: Informed<D::State, Goal, CostEstimate=D::Cost>,
        D::InformedError: Into<D::Error>,
    {
        for next in domain.choices(parent.state.clone()) {
            let (action, child_state) = next.map_err(Self::domain_err)?;
            Self::make_child_node(
                domain, memory,
                parent_id, &parent.state, &parent.cost,
                action.into(), child_state, goal
            )?
        }

        Ok(())
    }

    #[inline]
    fn make_child_node<Goal>(
        domain: &D,
        memory: &mut <Self as Algorithm>::Memory,
        parent_id: usize,
        parent_state: &D::State,
        parent_cost: &D::Cost,
        action: D::Action,
        child_state: D::State,
        goal: &Goal,
    ) -> Result<(), AStarSearchError<D::Error>>
    where
        D: Informed<D::State, Goal, CostEstimate=D::Cost>,
        D::InformedError: Into<D::Error>,
    {
        let cost = match domain
            .cost(parent_state, &action, &child_state)
            .map_err(Self::domain_err)? {
            Some(c) => c,
            None => return Ok(()),
        } + parent_cost.clone();

        let remaining_cost_estimate = match domain
            .remaining_cost_estimate(&child_state, goal)
            .map_err(Self::domain_err)? {
            Some(c) => c,
            None => return Ok(()),
        };

        memory.push_node(Node {
            state: child_state,
            cost,
            remaining_cost_estimate,
            parent: Some((parent_id, action)),
        }).map_err(Self::algo_err)?;

        Ok(())
    }
}

impl<D> Algorithm for AStar<D>
where
    D: Domain
    + Closable<D::State>
    + Weighted<D::State, D::Action>,
{
    type Memory = Memory<D::ClosedSet<usize>, D::State, D::Action, D::Cost>;
}

impl<D, Start, Goal> Coherent<Start, Goal> for AStar<D>
where
    D: Domain
    + Initializable<Start, D::State>
    + Closable<D::State>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>,
    D::State: Clone,
    D::Action: Clone,
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
        Self::initialize_impl(&self.0, start, goal)
    }
}

impl<D, Goal> Solvable<Goal> for AStar<D>
where
    D: Domain
    + Closable<D::State>
    + Activity<D::State, ActivityAction=D::Action>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>
    + Satisfiable<D::State, Goal>,
    D::State: Clone,
    D::Action: Clone,
    D::Error: Error,
    D::Cost: Ord + Add<Output = D::Cost> + Clone,
    D::SatisfactionError: Into<D::Error>,
    D::ActivityError: Into<D::Error>,
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
        let (top_id, top) = match AStar::<D>::choose_top(
            &self.0,
            &mut memory.closed_set,
            &mut memory.queue,
            &memory.arena,
            goal,
        )? {
            Flow::Continue(r) => r,
            Flow::Return(r) => return Ok(r),
        };

        AStar::<D>::expand_from_parent(&self.0, memory, top_id, &top, goal)?;

        Ok(Status::Incomplete)
    }
}

impl<D> Algorithm for AStarConnect<D>
where
    D: Domain
    + Closable<D::State>
    + Weighted<D::State, D::Action>,
{
    type Memory = Memory<D::ClosedSet<usize>, D::State, D::Action, D::Cost>;
}

impl<D, Start, Goal> Coherent<Start, Goal> for AStarConnect<D>
where
    D: Domain
    + Initializable<Start, D::State>
    + Closable<D::State>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>,
    D::State: Clone,
    D::Action: Clone,
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
        AStar::<D>::initialize_impl(&self.0, start, goal)
    }
}

impl<D, Goal> Solvable<Goal> for AStarConnect<D>
where
    D: Domain
    + Closable<D::State>
    + Activity<D::State, ActivityAction=D::Action>
    + Weighted<D::State, D::Action>
    + Informed<D::State, Goal, CostEstimate=D::Cost>
    + Satisfiable<D::State, Goal>
    + Connectable<D::State, Goal>,
    D::State: Clone,
    D::Action: Clone,
    D::Error: Error,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
    D::SatisfactionError: Into<D::Error>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::InformedError: Into<D::Error>,
    D::Connection: Into<D::Action>,
    D::ConnectionError: Into<D::Error>,
{
    type Solution = Solution<D::State, D::Action, D::Cost>;
    type StepError = AStarSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<Status<Self::Solution>, Self::StepError> {
        let (top_id, top) = match AStar::<D>::choose_top(
            &self.0,
            &mut memory.closed_set,
            &mut memory.queue,
            &memory.arena,
            goal
        )? {
            Flow::Continue(r) => r,
            Flow::Return(r) => return Ok(r),
        };

        AStar::<D>::expand_from_parent(&self.0, memory, top_id, &top, goal)?;

        // Attempt to connect the top to the goal
        for connection in self.0.connect(top.state.clone(), goal) {
            let (action, child_state) = connection.map_err(AStar::<D>::domain_err)?;
            AStar::<D>::make_child_node(
                &self.0, memory,
                top_id, &top.state, &top.cost,
                action.into(), child_state, goal
            )?;
        }

        Ok(Status::Incomplete)
    }
}

pub struct QueueTicket<Cost> {
    total_cost_estimate: Cost,
    node_id: usize,
}

#[derive(Clone)]
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

/// Control flow return value for functions that constitute step()
enum Flow<T, D>
where
    D: Domain + Weighted<D::State, D::Action>,
    D::Error: Error,
{
    Continue(T),
    Return(Status<Solution<D::State, D::Action, D::Cost>>),
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

trait NodeContainer<State, Action, Cost> {
    fn get_node(&self, index: usize) -> Result<&Node<State, Action, Cost>, AStarImplError>;
    fn retrace(&self, index: usize) -> Result<Solution<State, Action, Cost>, AStarImplError>;
}

impl<S: Clone, A: Clone, C: Clone> NodeContainer<S, A, C> for Vec<Node<S, A, C>> {
    fn get_node(&self, index: usize) -> Result<&Node<S, A, C>, AStarImplError> {
        self.get(index).ok_or(AStarImplError::BrokenReference(index))
    }
    fn retrace(&self, node_id: usize) -> Result<Solution<S, A, C>, AStarImplError> {
        let total_cost = self.get_node(node_id)?.cost.clone();
        let mut initial_node_id = node_id;
        let mut next_node_id = Some(node_id);
        let mut sequence = Vec::new();
        while let Some(current_node_id) = next_node_id {
            initial_node_id = current_node_id;
            let node = self.get_node(current_node_id)?;
            if let Some((_, action)) = &node.parent {
                sequence.push((action.clone(), node.state.clone()));
            }

            next_node_id = node.parent.as_ref().map(|(parent, _)| *parent);
        }

        sequence.reverse();

        let initial_state = self.get_node(initial_node_id)?.state.clone();
        Ok(Solution { initial_state, sequence, total_cost })
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
