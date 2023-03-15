/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
    domain::{
        Domain, Keyed, Closable, Activity, Weighted, Initializable, Keyring,
        ClosedSet, ClosedStatusForKey, ClosedStatus, CloseResult,
    },
    algorithm::{Algorithm, Coherent, Solvable, Status},
    tree::*,
    error::ThisError,
};
use std::{
    sync::{Arc, Mutex, RwLock},
    ops::Add,
    collections::{HashMap, hash_map::Entry},
    borrow::Borrow,
};

pub struct Dijkstra<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    domain: D,
    cache: Mutex<Cache<D>>,
}

impl<D> Algorithm for Dijkstra<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    type Memory = Memory<D>;
}

impl<D> Dijkstra<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    pub fn domain(&self) -> &D {
        &self.domain
    }

    fn domain_err(err: impl Into<D::Error>) -> DijkstraSearchError<D::Error> {
        DijkstraSearchError::Domain(err.into())
    }

    fn algo_err(err: impl Into<DijkstraImplError>) -> DijkstraSearchError<D::Error> {
        DijkstraSearchError::Algorithm(err.into())
    }
}

impl<D> Coherent<D::Key, D::Key> for Dijkstra<D>
where
    D: Domain
    + Keyed
    + Initializable<D::Key, D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
    D::InitialError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::Cost: Clone + Ord,
    D::Key: Clone,
{
    type InitError = DijkstraSearchError<D::Error>;

    fn initialize(
        &self,
        start: D::Key,
        _: &D::Key,
    ) -> Result<Self::Memory, Self::InitError> {
        let tree = match self.cache.lock() {
            Ok(mut r) => {
                match r.trees.entry(start.clone()) {
                    Entry::Occupied(entry) => entry.get().clone(),
                    Entry::Vacant(entry) => {
                        let mut vt = CachedTree::new(self.domain.new_closed_set());
                        for initial_state in self.domain.initialize(start) {
                            let state = initial_state.map_err(Self::domain_err)?;
                            let cost = match self.domain.initial_cost(&state).map_err(Self::domain_err)? {
                                Some(c) => c,
                                None => continue,
                            };

                            vt.tree.push_node(Node {
                                cost,
                                state,
                                parent: None
                            }).map_err(Self::algo_err)?;
                        }

                        entry.insert(Arc::new(RwLock::new(vt))).clone()
                    }
                }
            }
            Err(_) => return Err(Self::algo_err(DijkstraImplError::PoisonedMutex)),
        };

        Ok(Memory::new(tree))
    }
}

impl<D> Solvable<D::Key> for Dijkstra<D>
where
    D: Domain
    + Keyring<D::State>
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
    D::State: Clone,
    D::ActivityAction: Clone,
    D::Cost: Clone + Ord + Add<D::Cost, Output = D::Cost>,
    D::ClosedSet<usize>: ClosedStatusForKey<D::Key, usize>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
{
    type Solution = Path<D::State, D::ActivityAction, D::Cost>;
    type StepError = DijkstraSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        goal: &D::Key,
    ) -> Result<Status<Self::Solution>, Self::StepError> {
        if let Some(solution) = &memory.solution {
            return Ok(Status::Solved(solution.clone()));
        }

        match memory.tree.read() {
            Ok(cache) => {
                let tree = &cache.tree;
                let closed_set_len = tree.closed_set.closed_keys_len();
                if Some(closed_set_len) != memory.last_known_closed_len {
                    // The tree was grown by another search, so let's check if
                    // we already found a solution.
                    match tree.closed_set.status_for_key(goal) {
                        ClosedStatus::Open => {
                            // The goal was never reached, so continue searching
                        }
                        ClosedStatus::Closed(node_id) => {
                            // The goal was reached during a different search
                            let path = tree.arena.retrace(*node_id)
                                .map_err(Self::algo_err)?;
                            memory.last_known_closed_len = Some(closed_set_len);
                            memory.solution = Some(path.clone());
                            return Ok(Status::Solved(path));
                        }
                    }
                }
            }
            Err(_) => return Err(Self::algo_err(DijkstraImplError::PoisonedMutex)),
        };

        // A solution does not already exist in the cached tree, so we need to
        // get write access to the tree and grow it.
        let mut cache = memory.tree.write()
            .map_err(|_| Self::algo_err(DijkstraImplError::PoisonedMutex))?;
        let tree = &mut cache.tree;

        for _ in 0..memory.iterations_per_step {
            let top_id = match tree.queue.pop() {
                Some(top) => top.0.node_id,
                None => return Ok(Status::Impossible),
            };

            let top = tree.arena.get_node(top_id).map_err(Self::algo_err)?.clone();
            if let CloseResult::Rejected { prior, .. } = tree.closed_set.close(&top.state, top_id) {
                let prior_node = tree.arena.get_node(*prior).map_err(Self::algo_err)?;
                if prior_node.cost <= top.cost {
                    // The state we are attempting to expand has already been
                    // closed in the past by a lower cost node, so we will not
                    // expand from this top node.
                    continue;
                }
            }

            for next in self.domain.choices(top.state.clone()) {
                let (action, child_state) = next.map_err(Self::domain_err)?;
                let child_cost = match self.domain
                    .cost(&top.state, &action, &child_state)
                    .map_err(Self::domain_err)?
                {
                    Some(c) => c,
                    None => continue,
                } + top.cost.clone();

                tree.push_node(Node {
                    state: child_state,
                    cost: child_cost,
                    parent: Some((top_id, action)),
                }).map_err(Self::algo_err)?;
            }

            let top_key_ref = self.domain.key_for(top.state());
            let top_key: &D::Key = top_key_ref.borrow();
            if *top_key == *goal {
                // We have found the solution
                let solution = tree.arena.retrace(top_id).map_err(Self::algo_err)?;
                return Ok(Status::Solved(solution));
            }
        }

        memory.last_known_closed_len = Some(tree.closed_set.closed_keys_len());
        return Ok(Status::Incomplete);
    }
}

#[derive(ThisError, Debug)]
pub enum DijkstraSearchError<D> {
    #[error("An error occurred in the algorithm:\n{0}")]
    Algorithm(DijkstraImplError),
    #[error("An error occurred in the domain:\n{0}")]
    Domain(D),
}

#[derive(ThisError, Debug)]
pub enum DijkstraImplError {
    #[error("An error occurred with the tree:\n{0}")]
    Tree(TreeError),
    #[error("A mutex was poisoned")]
    PoisonedMutex,
}

impl From<TreeError> for DijkstraImplError {
    fn from(value: TreeError) -> Self {
        DijkstraImplError::Tree(value)
    }
}

struct Cache<D>
where
    D: Domain
    + Keyed
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    trees: HashMap<D::Key, SharedCachedTree<D>>,
}

/// A RwLock is used to store the cached tree because the asymptotic behavior of
/// the algorithm is to repeatedly look up solutions after the tree has fully
/// or at least sufficiently grown. The time spent growing a tree rapidly
/// diminishes over the course of multiple searches.
type SharedCachedTree<D> = Arc<RwLock<CachedTree<D>>>;

struct CachedTree<D>
where
    D: Domain
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>
{
    /// The tree data that has been cached
    tree: Tree<D::ClosedSet<usize>, Node<D::State, D::ActivityAction, D::Cost>, D::Cost>,
}

impl<D> CachedTree<D>
where
    D: Domain
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    fn new(closed_set: D::ClosedSet<usize>) -> Self
    where
        D::Cost: Clone + Ord,
    {
        Self {
            tree: Tree::new(closed_set),
        }
    }
}

pub struct Memory<D>
where
    D: Domain
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    /// Tracks how many closed items were in the set the last time the tree was
    /// grown by this search. This lets us know if another search has grown the
    /// tree in the interim.
    last_known_closed_len: Option<usize>,
    /// How many iterations to attempt per step of the algorithm. Having a
    /// higher value reduces the overhead of borrowing and releasing the RwLock
    /// but leaves less opportunity to interrupt the search.
    iterations_per_step: usize,
    /// A reference to the cache entry that is being searched.
    tree: SharedCachedTree<D>,
    /// Tracks whether a solution has already been found.
    solution: Option<Path<D::State, D::ActivityAction, D::Cost>>,
}

impl<D> Memory<D>
where
    D: Domain
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>
    + Closable<D::State>,
{
    fn new(tree: SharedCachedTree<D>) -> Self {
        Self {
            tree,
            iterations_per_step: 10,
            last_known_closed_len: None,
            solution: None,
        }
    }
}

#[derive(Debug, Clone)]
struct Node<State, Action, Cost> {
    state: State,
    cost: Cost,
    parent: Option<(usize, Action)>,
}

impl<State, Action, Cost: Clone> TreeNode for Node<State, Action, Cost> {
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
        self.cost.clone()
    }
}
