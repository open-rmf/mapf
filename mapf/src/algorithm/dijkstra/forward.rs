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
    algorithm::{tree::*, Algorithm, Coherent, Path, SearchStatus, Solvable},
    domain::{
        Activity, ArrivalKeyring, Closable, CloseResult, ClosedSet, ClosedStatus,
        ClosedStatusForKey, Configurable, Connectable, Domain, Initializable, Keyed, Keyring,
        Weighted,
    },
    error::{Anyhow, ThisError},
};
use std::{
    borrow::Borrow,
    collections::{hash_map::Entry, HashMap},
    ops::Add,
    sync::{Arc, Mutex, RwLock},
};

pub struct Dijkstra<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    domain: D,
    cache: Arc<Mutex<Cache<D>>>,
}

impl<D> Algorithm for Dijkstra<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    type Memory = Memory<D>;
}

impl<D> Dijkstra<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    pub fn new(domain: D) -> Self {
        Self {
            domain,
            cache: Arc::new(Mutex::new(Default::default())),
        }
    }

    pub fn domain(&self) -> &D {
        &self.domain
    }

    pub fn inspect_trees<U, F: FnMut(&D::Key, &CachedTree<D>) -> U>(&self, mut f: F) -> Vec<U> {
        let mut result = Vec::new();
        let guard = self.cache.lock().unwrap();
        for (key, tree) in guard.trees.iter() {
            let ct = tree.read().unwrap();
            result.push(f(key, &ct));
        }

        return result;
    }

    fn domain_err(err: impl Into<D::Error>) -> DijkstraSearchError<D::Error> {
        DijkstraSearchError::Domain(err.into())
    }

    fn algo_err(err: impl Into<DijkstraImplError>) -> DijkstraSearchError<D::Error> {
        DijkstraSearchError::Algorithm(err.into())
    }
}

impl<D, Start, Goal> Coherent<Start, Goal> for Dijkstra<D>
where
    D: Domain
        + Keyring<D::State>
        + Initializable<Start, Goal, D::State>
        + Activity<D::State>
        + Weighted<D::State, D::Action>
        + Closable<D::State>
        + Connectable<D::State, D::Action, D::Key>
        + ArrivalKeyring<D::Key, Start, Goal>,
    D::ClosedSet<usize>: ClosedStatusForKey<D::Key, usize>,
    D::Action: Clone,
    D::InitialError: Into<D::Error>,
    D::ArrivalKeyError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::ConnectionError: Into<D::Error>,
    D::State: Clone,
    D::Cost: Clone + Ord + Add<D::Cost, Output = D::Cost>,
    D::Key: Clone,
    Goal: Clone,
{
    type InitError = DijkstraSearchError<D::Error>;

    fn initialize(&self, start: Start, goal: &Goal) -> Result<Self::Memory, Self::InitError> {
        let goal_keys: Result<Vec<_>, _> = self
            .domain
            .get_arrival_keys(&start, goal)
            .into_iter()
            .map(|r| r.map_err(Self::domain_err))
            .collect();
        let goal_keys = goal_keys?;

        let mut trees = Vec::new();
        for state in self.domain.initialize(start, goal) {
            let state = state.map_err(Self::domain_err)?;
            let key_ref = self.domain.key_for(&state);
            let tree = match self.cache.lock() {
                Ok(mut r) => {
                    match r.trees.entry(key_ref.borrow().clone()) {
                        Entry::Occupied(mut entry) => {
                            {
                                let mut mt = match entry.get_mut().write() {
                                    Ok(mt) => mt,
                                    Err(_) => {
                                        return Err(Self::algo_err(
                                            DijkstraImplError::PoisonedMutex,
                                        ))
                                    }
                                };

                                // let tree = &mut mt.tree;
                                let cache: &mut CachedTree<D> = &mut mt;
                                let tree = &mut cache.tree;
                                for goal_key in &goal_keys {
                                    if tree.closed_set.status_for_key(goal_key).is_open() {
                                        // This goal has never been reached. It is possible that
                                        // optimally reaching this goal will require a lazy connection
                                        // from a node that has already been closed. Therefore
                                        // we will go through the entire closed set and attempt a
                                        // lazy connection to this goal, finding the one that gives
                                        // the best cost.
                                        let mut best_node: Option<Node<_, _, _>> = None;
                                        for parent_id in &cache.decisive_closed_nodes {
                                            let node = tree
                                                .arena
                                                .get_node(*parent_id)
                                                .map_err(Self::algo_err)?;
                                            for next in
                                                self.domain.connect(node.state.clone(), goal_key)
                                            {
                                                let (action, child_state) =
                                                    next.map_err(Self::domain_err)?;
                                                let child_cost = match self
                                                    .domain
                                                    .cost(&node.state, &action, &child_state)
                                                    .map_err(Self::domain_err)?
                                                {
                                                    Some(c) => c,
                                                    None => continue,
                                                } + node.cost.clone();

                                                let new_node = Node {
                                                    state: child_state,
                                                    cost: child_cost,
                                                    parent: Some((*parent_id, action)),
                                                    decisive: false,
                                                };

                                                if let Some(best_node) = &mut best_node {
                                                    if new_node.cost < best_node.cost {
                                                        *best_node = new_node;
                                                    }
                                                } else {
                                                    best_node = Some(new_node);
                                                }
                                            }
                                        }

                                        if let Some(node) = best_node {
                                            tree.push_node(node).map_err(Self::algo_err)?;
                                        }
                                    }
                                }
                            }

                            entry.get().clone()
                        }
                        Entry::Vacant(entry) => {
                            let mut ct = CachedTree::new(self.domain.new_closed_set());
                            let cost =
                                match self.domain.initial_cost(&state).map_err(Self::domain_err)? {
                                    Some(c) => c,
                                    None => continue,
                                };

                            ct.tree
                                .push_node(Node {
                                    cost,
                                    state: state.clone(),
                                    parent: None,
                                    decisive: true,
                                })
                                .map_err(Self::algo_err)?;

                            entry.insert(Arc::new(RwLock::new(ct))).clone()
                        }
                    }
                }
                Err(_) => return Err(Self::algo_err(DijkstraImplError::PoisonedMutex)),
            };

            trees.push(TreeMemory::new(tree));
        }

        Ok(Memory::new(trees, goal_keys))
    }
}

impl<D, Goal> Solvable<Goal> for Dijkstra<D>
where
    D: Domain
        + Keyring<D::State>
        + Activity<D::State>
        + Weighted<D::State, D::Action>
        + Closable<D::State>
        + Connectable<D::State, D::Action, D::Key>,
    D::State: Clone,
    D::Action: Clone,
    D::Cost: Clone + Ord + Add<D::Cost, Output = D::Cost>,
    D::ClosedSet<usize>: ClosedStatusForKey<D::Key, usize>,
    D::ActivityError: Into<D::Error>,
    D::WeightedError: Into<D::Error>,
    D::ConnectionError: Into<D::Error>,
{
    type Solution = Path<D::State, D::Action, D::Cost>;
    type StepError = DijkstraSearchError<D::Error>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        _: &Goal,
    ) -> Result<SearchStatus<Self::Solution>, Self::StepError> {
        if memory.exhausted {
            if let Some((s, _)) = memory.best_solution {
                let solution = memory.trees.get(s).map(|t| t.solution.clone()).flatten();
                if let Some(solution) = solution {
                    return Ok(SearchStatus::Solved(solution));
                } else {
                    return Err(Self::algo_err(DijkstraImplError::MissingSolutionReference(
                        s,
                    )));
                }
            }

            // If the search is exhausted then simply return that the problem
            // is impossible.
            return Ok(SearchStatus::Impossible);
        }

        memory.exhausted = true;
        for (tree_i, mt) in memory.trees.iter_mut().enumerate() {
            let cost_bound = memory.best_solution.as_ref().map(|(_, c)| c.clone());

            let mut tree_solution = match mt.tree.read() {
                Ok(cache) => {
                    let tree = &cache.tree;
                    let closed_set_len = tree.closed_set.closed_keys_len();
                    if Some(closed_set_len) != mt.last_known_closed_len {
                        // The tree was grown by another search, so let's check
                        // if we already found a solution.
                        let mut best_solution: Option<Path<D::State, D::Action, D::Cost>> = None;
                        for goal_key in &memory.goal_keys {
                            match tree.closed_set.status_for_key(goal_key) {
                                ClosedStatus::Open => {
                                    // The goal was never reached
                                }
                                ClosedStatus::Closed(node_id) => {
                                    let node =
                                        tree.arena.get_node(*node_id).map_err(Self::algo_err)?;

                                    if let Some(best_solution) = &best_solution {
                                        if best_solution.total_cost <= node.cost {
                                            // The path that would be created
                                            // for this node is not better than
                                            // the solution we already have.
                                            continue;
                                        }
                                    }

                                    let path =
                                        tree.arena.retrace(*node_id).map_err(Self::algo_err)?;
                                    best_solution = Some(path);
                                }
                            }
                        }

                        best_solution
                    } else {
                        // The tree has not grown since the last search, so
                        // there is no reason to expect a solution to have
                        // shown up since then.
                        None
                    }
                }
                Err(_) => return Err(Self::algo_err(DijkstraImplError::PoisonedMutex)),
            };

            if tree_solution.is_none() {
                // A solution does not already exist in the cached stree, so we
                // will get write access to the tree and grow it.
                let mut guard = mt
                    .tree
                    .write()
                    .map_err(|_| Self::algo_err(DijkstraImplError::PoisonedMutex))?;
                // let tree = &mut cache.tree;
                let cache: &mut CachedTree<_> = &mut guard;

                'grow: for _ in 0..memory.iterations_per_step {
                    let top_id = match cache.tree.queue.pop() {
                        Some(top) => top.0.node_id,
                        None => break,
                    };

                    let top = cache
                        .tree
                        .arena
                        .get_node(top_id)
                        .map_err(Self::algo_err)?
                        .clone();
                    if let CloseResult::Rejected { prior, .. } =
                        cache.tree.closed_set.close(&top.state, top_id)
                    {
                        let prior_node =
                            cache.tree.arena.get_node(*prior).map_err(Self::algo_err)?;
                        if prior_node.cost <= top.cost {
                            // The state we are attempting to expand has already been
                            // closed in the past by a lower cost node, so we will not
                            // expand from this top node.
                            continue;
                        }

                        *prior = top_id;
                    }
                    if top.decisive {
                        cache.decisive_closed_nodes.push(top_id);
                    }

                    for next in self.domain.choices(top.state.clone()) {
                        let (action, child_state) = next.map_err(Self::domain_err)?;
                        let child_cost = match self
                            .domain
                            .cost(&top.state, &action, &child_state)
                            .map_err(Self::domain_err)?
                        {
                            Some(c) => c,
                            None => continue,
                        } + top.cost.clone();

                        cache
                            .tree
                            .push_node(Node {
                                state: child_state,
                                cost: child_cost,
                                parent: Some((top_id, action)),
                                decisive: true,
                            })
                            .map_err(Self::algo_err)?;
                    }

                    let top_key_ref = self.domain.key_for(top.state());
                    let top_key: &D::Key = top_key_ref.borrow();
                    for goal_key in &memory.goal_keys {
                        if *top_key == *goal_key {
                            // We have found a solution.
                            let path = cache.tree.arena.retrace(top_id).map_err(Self::algo_err)?;
                            tree_solution = Some(path);
                            break 'grow;
                        } else {
                            // TODO(@mxgrey): De-duplicate this with the above
                            // iteration through the activity
                            for next in self.domain.connect(top.state.clone(), goal_key) {
                                let (action, child_state) = next.map_err(Self::domain_err)?;
                                let child_cost = match self
                                    .domain
                                    .cost(&top.state, &action, &child_state)
                                    .map_err(Self::domain_err)?
                                {
                                    Some(c) => c,
                                    None => continue,
                                } + top.cost.clone();

                                cache
                                    .tree
                                    .push_node(Node {
                                        state: child_state,
                                        cost: child_cost,
                                        parent: Some((top_id, action)),
                                        decisive: false,
                                    })
                                    .map_err(Self::algo_err)?;
                            }
                        }
                    }

                    if let Some(cost_bound) = &cost_bound {
                        if *cost_bound < top.cost {
                            // There is no point growing the tree further
                            // because it cannot produce a solution better than
                            // the current best.
                            break 'grow;
                        }
                    }
                }

                mt.last_known_closed_len = Some(cache.tree.closed_set.closed_keys_len());
                if cache
                    .tree
                    .queue
                    .peek()
                    .filter(|t| {
                        if let Some(cost_bound) = &cost_bound {
                            t.0.evaluation < *cost_bound
                        } else {
                            true
                        }
                    })
                    .is_some()
                {
                    memory.exhausted = false;
                }
            }

            if let Some(solution) = tree_solution {
                if mt
                    .solution
                    .as_ref()
                    .filter(|prior_solution| prior_solution.total_cost < solution.total_cost)
                    .is_none()
                {
                    if let Some((best, cost)) = &mut memory.best_solution {
                        if solution.total_cost < *cost {
                            *best = tree_i;
                            *cost = solution.total_cost.clone();
                        }
                    } else {
                        memory.best_solution = Some((tree_i, solution.total_cost.clone()));
                    }

                    mt.solution = Some(solution);
                }
            }
        }

        return Ok(SearchStatus::Incomplete);
    }
}

/// Note that configuring a Dijkstra algorithm will clear the cache. If the
/// underlying configuration changes then we cannot assume that any of the
/// previous search remains valid.
impl<D: Configurable> Configurable for Dijkstra<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    type Configuration = D::Configuration;
    fn configure<F>(self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(Self::Configuration) -> Result<Self::Configuration, Anyhow>,
    {
        // We have to assume that all caches are invalidated when the domain
        // gets configured.
        Ok(Self::new(self.domain.configure(f)?))
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
    #[error("Expected to find a solution in tree {0} but found None - please report this bug")]
    MissingSolutionReference(usize),
    #[error("A mutex was poisoned")]
    PoisonedMutex,
}

impl From<TreeError> for DijkstraImplError {
    fn from(value: TreeError) -> Self {
        DijkstraImplError::Tree(value)
    }
}

#[derive(Clone)]
struct Cache<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    trees: HashMap<D::Key, SharedCachedTree<D>>,
}

impl<D> Default for Cache<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    fn default() -> Self {
        Self {
            trees: Default::default(),
        }
    }
}

/// A RwLock is used to store the cached tree because the asymptotic behavior of
/// the algorithm is to repeatedly look up solutions after the tree has fully
/// or at least sufficiently grown. The time spent growing a tree rapidly
/// diminishes over the course of multiple searches.
type SharedCachedTree<D> = Arc<RwLock<CachedTree<D>>>;

pub struct CachedTree<D>
where
    D: Domain + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    /// The tree data that has been cached
    tree: Tree<D::ClosedSet<usize>, Node<D::State, D::Action, D::Cost>, D::Cost>,
    decisive_closed_nodes: Vec<usize>,
}

impl<D> CachedTree<D>
where
    D: Domain + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    fn new(closed_set: D::ClosedSet<usize>) -> Self
    where
        D::Cost: Clone + Ord,
    {
        Self {
            tree: Tree::new(closed_set),
            decisive_closed_nodes: Vec::new(),
        }
    }

    pub fn tree(&self) -> &Tree<D::ClosedSet<usize>, Node<D::State, D::Action, D::Cost>, D::Cost> {
        &self.tree
    }
}

pub struct Memory<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    /// Trees that are being grown for this search.
    // TODO(@mxgrey): Consider using a SmallVec here to avoid heap allocation
    trees: Vec<TreeMemory<D>>,
    /// Valid keys
    // TODO(@mxgrey): Consider using a SmallVec here to avoid heap allocation
    goal_keys: Vec<D::Key>,
    /// How many iterations to attempt per step of the algorithm. Having a
    /// higher value reduces the overhead of borrowing and releasing the RwLock
    /// but leaves less opportunity to interrupt the search.
    iterations_per_step: usize,
    /// Tracks which found solution is the best.
    best_solution: Option<(usize, D::Cost)>,
    /// Tracks whether all possible solutions have been exhausted.
    exhausted: bool,
}

impl<D> Memory<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    fn new(trees: Vec<TreeMemory<D>>, goal_keys: Vec<D::Key>) -> Self {
        Self {
            trees,
            goal_keys,
            iterations_per_step: 10,
            best_solution: None,
            exhausted: false,
        }
    }
}

pub struct TreeMemory<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    /// A reference to the cache entry that is being searched.
    tree: SharedCachedTree<D>,
    /// Tracks how many closed items were in the set the last time the tree was
    /// grown by this search. This lets us know if another search has grown the
    /// tree in the interim.
    last_known_closed_len: Option<usize>,
    /// Tracks whether a solution was found for this tree.
    solution: Option<Path<D::State, D::Action, D::Cost>>,
}

impl<D> TreeMemory<D>
where
    D: Domain + Keyed + Activity<D::State> + Weighted<D::State, D::Action> + Closable<D::State>,
{
    fn new(tree: SharedCachedTree<D>) -> Self {
        Self {
            tree,
            last_known_closed_len: None,
            solution: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Node<State, Action, Cost> {
    state: State,
    cost: Cost,
    decisive: bool,
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

    fn queue_bias(&self) -> Option<Self::Cost> {
        None
    }
}
