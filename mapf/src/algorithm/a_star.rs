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
    error::{Anyhow, ThisError},
    algorithm::{
        Algorithm, Coherent, Solvable, SearchStatus, MinimumCostBound, Measure,
        Path, tree::*,
    },
    domain::{
        Domain, Activity, Weighted, Initializable, Informed, Satisfiable,
        Closable, ClosedSet, CloseResult, Connectable, Configurable,
    },
};
use std::ops::Add;

/// The AStar algorithm can be used on domains that implement the following traits:
/// * [`Initializable`]
/// * [`Closable`]
/// * [`Activity`]
/// * [`Weighted`]
/// * [`Informed`]
/// * [`Satisfiable`]
///
/// The following templates implement these traits:
/// * [`InformedSearch`]
#[derive(Default, Debug, Clone)]
pub struct AStar<D>(pub D);

/// The AStarConnect algorithm is a variation on AStar that can attempt to find
/// a connection directly to the goal each time a node is expanded. In addition
/// to required traits of [`AStar`], the domain must also implement:
/// * [`Connectable`] as `Connectable<D::State, Goal>`
#[derive(Debug, Clone)]
pub struct AStarConnect<D>(pub D);

#[derive(Debug)]
pub struct Memory<Closed, State, Action, Cost>(
    pub Tree<Closed, Node<State, Action, Cost>, Cost>
);

impl<Closed, State, Action, Cost> Measure for Memory<Closed, State, Action, Cost>
where
    Cost: Clone + Add<Cost, Output = Cost>,
{
    fn size(&self) -> usize {
        self.0.size()
    }
}

impl<Closed, State, Action, Cost> MinimumCostBound for Memory<Closed, State, Action, Cost>
where
    Cost: Clone + Add<Cost, Output = Cost>,
{
    type Cost = Cost;
    fn minimum_cost_bound(&self) -> Option<Self::Cost> {
        self.0.minimum_cost_bound()
    }
}

#[derive(ThisError, Debug)]
pub enum AStarSearchError<D> {
    #[error("An error occurred in the algorithm:\n{0}")]
    Algorithm(TreeError),
    #[error("An error occurred in the domain:\n{0}")]
    Domain(D),
}


impl<D> AStar<D> {
    fn domain_err(err: impl Into<D::Error>) -> AStarSearchError<D::Error>
    where
        D: Domain,
    {
        AStarSearchError::Domain(err.into())
    }

    fn algo_err(err: TreeError) -> AStarSearchError<D::Error>
    where
        D: Domain,
    {
        AStarSearchError::Algorithm(err)
    }
}

impl<D> AStar<D>
where
    D: Domain
    + Closable<D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>,
    D::State: Clone,
    D::ActivityAction: Clone,
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
        D: Initializable<Start, Goal, D::State>
        + Informed<D::State, Goal, CostEstimate=D::Cost>,
        D::InitialError: Into<D::Error>,
        D::InformedError: Into<D::Error>,
    {
        let mut memory = Memory(Tree::new(domain.new_closed_set()));

        for state in domain.initialize(start, goal) {
            let state = state.map_err(Self::domain_err)?;
            let cost = match domain.initial_cost(&state).map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };
            let remaining_cost_estimate = match domain.estimate_remaining_cost(
                &state, &goal
            ).map_err(Self::domain_err)? {
                Some(c) => c,
                None => continue,
            };

            memory.0.push_node(Node {
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
        queue: &mut TreeFrontierQueue<D::Cost>,
        arena: &Vec<Node<D::State, D::ActivityAction, D::Cost>>,
        goal: &Goal,
    ) -> Result<Flow<(usize, Node<D::State, D::ActivityAction, D::Cost>), D>, AStarSearchError<D::Error>>
    where
        D: Satisfiable<D::State, Goal> + Activity<D::State>,
        D::SatisfactionError: Into<D::Error>,
    {
        let top_id = match queue.pop() {
            Some(top) => top.0.node_id,
            None => return Ok(Flow::Return(SearchStatus::Impossible)),
        };

        let top = arena.get_node(top_id).map_err(Self::algo_err)?;
        if domain.is_satisfied(&top.state, goal).map_err(Self::domain_err)? {
            let solution = arena
                .retrace(top_id)
                .map_err(Self::algo_err)?;
            return Ok(Flow::Return(SearchStatus::Solved(solution)));
        }

        if let CloseResult::Rejected { prior, .. } = closed_set.close(&top.state, top_id) {
            let prior_node = arena.get_node(*prior).map_err(Self::algo_err)?;
            if prior_node.cost <= top.cost {
                // The state we are attempting to expand has already been closed
                // in the past by a lower cost node, so we will not expand from
                // this top node. Instead we will finish this iteration.
                return Ok(Flow::Return(SearchStatus::Incomplete));
            }

            // The top node has a lower cost so it should replace the node that
            // previously closed this state.
            *prior = top_id;
        }

        Ok(Flow::Proceed((top_id, top.clone())))
    }

    #[inline]
    fn expand_from_parent<Goal>(
        domain: &D,
        memory: &mut <Self as Algorithm>::Memory,
        parent_id: usize,
        parent: &Node<D::State, D::ActivityAction, D::Cost>,
        goal: &Goal,
    ) -> Result<(), AStarSearchError<D::Error>>
    where
        D: Activity<D::State>,
        D::ActivityAction: Into<D::ActivityAction>,
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
        action: D::ActivityAction,
        child_state: D::State,
        goal: &Goal,
    ) -> Result<(), AStarSearchError<D::Error>>
    where
        D: Informed<D::State, Goal, CostEstimate=D::Cost>,
        D::InformedError: Into<D::Error>,
    {
        let cost = match domain
            .cost(parent_state, &action, &child_state)
            .map_err(Self::domain_err)?
        {
            Some(c) => c,
            None => return Ok(()),
        } + parent_cost.clone();

        let remaining_cost_estimate = match domain
            .estimate_remaining_cost(&child_state, goal)
            .map_err(Self::domain_err)? {
            Some(c) => c,
            None => return Ok(()),
        };

        memory.0.push_node(Node {
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
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>,
{
    type Memory = Memory<D::ClosedSet<usize>, D::State, D::ActivityAction, D::Cost>;
}

impl<D, Start, Goal> Coherent<Start, Goal> for AStar<D>
where
    D: Domain
    + Initializable<Start, Goal, D::State>
    + Closable<D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Informed<D::State, Goal, CostEstimate=D::Cost>,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
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
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Informed<D::State, Goal, CostEstimate=D::Cost>
    + Satisfiable<D::State, Goal>,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Error: Into<Anyhow>,
    D::Cost: Ord + Add<Output = D::Cost> + Clone,
    D::SatisfactionError: Into<D::Error>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::InformedError: Into<D::Error>,
{
    type Solution = Path<D::State, D::ActivityAction, D::Cost>;
    type StepError = AStarSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError> {
        let (top_id, top) = match AStar::<D>::choose_top(
            &self.0,
            &mut memory.0.closed_set,
            &mut memory.0.queue,
            &memory.0.arena,
            goal,
        )? {
            Flow::Proceed(r) => r,
            Flow::Return(r) => return Ok(r),
        };

        AStar::<D>::expand_from_parent(&self.0, memory, top_id, &top, goal)?;

        Ok(SearchStatus::Incomplete)
    }
}

impl<D: Configurable> Configurable for AStar<D> {
    type Configuration = D::Configuration;
    type ConfigurationError = D::ConfigurationError;
    fn configure<F>(self, f: F) -> Result<Self, Self::ConfigurationError>
    where
        F: FnOnce(Self::Configuration) -> Self::Configuration,
    {
        Ok(AStar(self.0.configure(f)?))
    }
}

impl<D> Algorithm for AStarConnect<D>
where
    D: Domain
    + Closable<D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>,
{
    type Memory = Memory<D::ClosedSet<usize>, D::State, D::ActivityAction, D::Cost>;
}

impl<D, Start, Goal> Coherent<Start, Goal> for AStarConnect<D>
where
    D: Domain
    + Initializable<Start, Goal, D::State>
    + Closable<D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Informed<D::State, Goal, CostEstimate=D::Cost>,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
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
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Informed<D::State, Goal, CostEstimate=D::Cost>
    + Satisfiable<D::State, Goal>
    + Connectable<D::State, D::ActivityAction, Goal>,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Cost: Ord + Add<Output=D::Cost> + Clone,
    D::SatisfactionError: Into<D::Error>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::InformedError: Into<D::Error>,
    D::ConnectionError: Into<D::Error>,
{
    type Solution = Path<D::State, D::ActivityAction, D::Cost>;
    type StepError = AStarSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError> {
        let (top_id, top) = match AStar::<D>::choose_top(
            &self.0,
            &mut memory.0.closed_set,
            &mut memory.0.queue,
            &memory.0.arena,
            goal
        )? {
            Flow::Proceed(r) => r,
            Flow::Return(r) => return Ok(r),
        };

        AStar::<D>::expand_from_parent(&self.0, memory, top_id, &top, goal)?;

        // Attempt to connect the top to the goal
        for connection in self.0.connect(top.state.clone(), goal) {
            let (action, child_state) = connection.map_err(AStar::<D>::domain_err)?;
            AStar::<D>::make_child_node(
                &self.0, memory,
                top_id, &top.state, &top.cost,
                action, child_state, goal
            )?;
        }

        Ok(SearchStatus::Incomplete)
    }
}

impl<D: Configurable> Configurable for AStarConnect<D> {
    type Configuration = D::Configuration;
    type ConfigurationError = D::ConfigurationError;
    fn configure<F>(self, f: F) -> Result<Self, Self::ConfigurationError>
    where
        F: FnOnce(Self::Configuration) -> Self::Configuration,
    {
        Ok(AStarConnect(self.0.configure(f)?))
    }
}

#[derive(Debug, Clone)]
pub struct Node<State, Action, Cost> {
    state: State,
    cost: Cost,
    remaining_cost_estimate: Cost,
    parent: Option<(usize, Action)>,
}

impl<State, Action, Cost> Node<State, Action, Cost> {
    pub fn cost(&self) -> &Cost {
        &self.cost
    }

    pub fn remaining_cost_estimate(&self) -> &Cost {
        &self.remaining_cost_estimate
    }
}

impl<State, Action, Cost> TreeNode for Node<State, Action, Cost>
where
    Cost: Clone + Add<Cost, Output = Cost>,
{
    type State = State;
    type Action = Action;
    type Cost = Cost;

    fn state(&self) -> &Self::State {
        &self.state
    }

    fn parent(&self) -> Option<(usize, &Self::Action)> {
        self.parent.as_ref().map(|(id, action)| (*id, action))
    }

    fn cost(&self) -> Self::Cost {
        self.cost.clone()
    }

    fn queue_evaluation(&self) -> Self::Cost {
        self.cost.clone() + self.remaining_cost_estimate.clone()
    }
}

/// Control flow return value for functions that constitute step()
enum Flow<T, D>
where
    D: Domain + Activity<D::State> + Weighted<D::State, D::ActivityAction>,
    // D::Error: StdError,
{
    Proceed(T),
    Return(SearchStatus<Path<D::State, D::ActivityAction, D::Cost>>),
}
