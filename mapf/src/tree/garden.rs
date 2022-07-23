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

use super::Tree;
use crate::{
    node::{self, PartialKeyed, Keyed, ClosedSet, KeyedSet, KeyOf, Weighted},
    expander::{
        Expander, InitAimless, Aimless, Closable, Reversible, Solvable, BidirSolvable,
        CostOf, NodeOf, ReverseOf, ReverseNodeOf, SolutionOf, InitAimlessErrorOf, AimlessErrorOf, ReversalErrorOf, BidirSolveErrorOf,
    },
    util::Minimum,
};
use std::{
    collections::hash_map::HashMap,
    sync::{Arc, Mutex},
    cell::RefCell,
};
use thiserror::Error as ThisError;

type MutexRefCell<T> = Mutex<RefCell<T>>;


#[derive(ThisError, Debug)]
pub enum ErrorImpl<I, E, S> {
    #[error("An error occurred while initializing:\n{0}")]
    Init(I),
    #[error("An error occurred while expanding:\n{0}")]
    Expansion(E),
    #[error("An error occurred while constructing the solution:\n{0}")]
    Solve(S),
    #[error("a mutex was poisoned")]
    PoisenedMutex,
}

// We use this wrapper because otherwise derive(Debug) does not work correctly
pub type Error<E, S> = ErrorImpl<
    <E as InitAimless<S>>::InitAimlessError,
    <E as Aimless>::AimlessError,
    <E as Solvable>::SolveError,
>;

type TreeCache<E> = MutexRefCell<HashMap<KeyOf<NodeOf<E>>, Arc<MutexRefCell<Tree<E>>>>>;
type SolutionCache<E> = MutexRefCell<HashMap<(KeyOf<NodeOf<E>>, KeyOf<NodeOf<E>>), Option<SolutionOf<E>>>>;
type ConnectionMap<E> = HashMap<KeyOf<NodeOf<E>>, (Option<Arc<NodeOf<E>>>, Option<Arc<NodeOf<ReverseOf<E>>>>)>;

fn new_tree_entry<E: Aimless<Node: Weighted> + Closable>(tree: Tree<E>) -> Arc<MutexRefCell<Tree<E>>> {
    Arc::new(Mutex::new(RefCell::new(tree)))
}

pub struct Garden<E>
where
    E: Aimless + Solvable + Closable,
    NodeOf<E>: Weighted + PartialKeyed,
{
    expander: Arc<E>,
    trees: TreeCache<E>,
    solutions: SolutionCache<E>,
}

impl<E> Garden<E>
where
    E: Aimless + Solvable + Closable,
    NodeOf<E>: Weighted + PartialKeyed,
{
    pub fn new(expander: Arc<E>) -> Self {
        Self{
            expander,
            trees: Default::default(),
            solutions: Default::default(),
        }
    }

    pub fn solve<S>(&self, from: &S, to: &KeyOf<NodeOf<E>>) -> Result<Option<E::Solution>, Error<E, S>>
    where
        E: InitAimless<S>,
        E::Solution: Clone + Weighted,
        E::ClosedSet: KeyedSet<NodeOf<E>, Key=KeyOf<NodeOf<E>>>,
        S: Keyed<Key=KeyOf<NodeOf<E>>>,
    {
        let mut best_solution = Minimum::new(|u: &E::Solution, v: &E::Solution| { u.cost().cmp(&v.cost()) });

        let start_key = from.key();
        for start in self.expander.aimless_start(from) {
            let start = start.map_err(ErrorImpl::Init)?;
            dbg!(start.partial_key());
            let already_have_solution = {
                // We put this in a deeper scope to make sure that the lock on
                // the solutions map mutex is released before we begin expanding
                // the tree.
                if let Some(solved) = self.solutions.lock()
                    .map_err(|_| ErrorImpl::PoisenedMutex)?
                    .borrow().get(&(start_key.clone(), to.clone())) {
                    // The tree from this start point has already been grown towards
                    // this target and either reached it or exhausted itself.
                    if let Some(solution) = solved {
                        // This tree has reached the target, so let's return the
                        // solution for it.
                        best_solution.consider(solution);
                    }

                    true
                } else {
                    false
                }
            };

            if !already_have_solution {
                // This tree from this start point has not been grown towards
                // this target yet.
                let tree_arc = {
                    self.trees.lock()
                        .map_err(|_| ErrorImpl::PoisenedMutex)?
                        .borrow_mut()
                        .entry(start_key.clone())
                        .or_insert_with(|| new_tree_entry(Tree::new(start.clone(), self.expander.clone())))
                        .clone()
                };

                let tree_lock = tree_arc.lock().map_err(|_| ErrorImpl::PoisenedMutex)?;
                let mut tree = tree_lock.borrow_mut();
                if let Some(reached_goal) = tree.closed().get(to) {
                    // This tree has already passed by the target, so we can
                    // construct a solution for it.
                    let solution = self.expander.make_solution(reached_goal).map_err(ErrorImpl::Solve)?;
                    self.solutions.lock().map_err(|_| ErrorImpl::PoisenedMutex)?
                        .borrow_mut().insert((start_key.clone(), to.clone()), Some(solution.clone()));
                    best_solution.consider_take(solution);
                } else {
                    // This tree has never passed by the taraget, so we need to
                    // grow it until it reaches.
                    while !tree.is_exhausted() {
                        let mut found_solution = false;
                        for node in tree.grow() {
                            let node: Arc<NodeOf<E>> = node.map_err(ErrorImpl::Expansion)?;
                            dbg!(node.partial_key());
                            if node.partial_key() == Some(to) {
                                let solution = self.expander.make_solution(&node).map_err(ErrorImpl::Solve)?;
                                best_solution.consider(&solution);
                                dbg!("Locking solutions");
                                self.solutions.lock().map_err(|_| ErrorImpl::PoisenedMutex)?
                                    .borrow_mut().insert((start_key.clone(), to.clone()), Some(solution));
                                dbg!("Done locking");
                                found_solution = true;
                            }
                        }

                        dbg!("Finished growth");

                        if found_solution {
                            dbg!("Found solution");
                            break;
                        }
                    }

                    dbg!("Left loop");
                }
            }
        }

        return Ok(best_solution.result());
    }
}

#[derive(ThisError, Debug)]
pub enum BidirInitError<E, S, G>
where
    E: InitAimless<S> + Reversible,
    E::Reverse: InitAimless<G>,
{
    #[error("The forward tree had an error:\n{0}")]
    Forward(InitAimlessErrorOf<E, S>),
    #[error("The reverse tree had an error:\n{0}")]
    Reverse(InitAimlessErrorOf<ReverseOf<E>, G>),
    #[error("a mutex was poisoned")]
    PoisonedMutex,
}

#[derive(ThisError, Debug)]
pub enum BidirExpansionError<E>
where
    E: Aimless + BidirSolvable,
    E::Reverse: Aimless,
{
    #[error("The forward tree had an error:\n{0}")]
    Forward(AimlessErrorOf<E>),
    #[error("The reverse tree had an error:\n{0}")]
    Reverse(AimlessErrorOf<ReverseOf<E>>),
    #[error("An error occurred while constructing the solution:\n{0}")]
    Solve(BidirSolveErrorOf<E>),
    #[error("a mutex was poisoned")]
    PoisonedMutex,
}

#[derive(ThisError, Debug)]
pub enum BidirError<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
    ReverseOf<E>: Aimless,
{
    #[error("An error occurred while initializing:\n{0}")]
    Init(BidirInitError<E, S, G>),
    #[error("An error occurred while expanding:\n{0}")]
    Expansion(BidirExpansionError<E>),
}

impl<E, S, G> From<BidirInitError<E, S, G>> for BidirError<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
{
    fn from(e: BidirInitError<E, S, G>) -> Self {
        BidirError::Init(e)
    }
}

impl<E, S, G> From<BidirExpansionError<E>> for BidirError<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
{
    fn from(e: BidirExpansionError<E>) -> Self {
        BidirError::Expansion(e)
    }
}

pub struct BidirGarden<E>
where
    E: Aimless + BidirSolvable + Closable,
    E::Reverse: Aimless + Closable,
    NodeOf<E>: Weighted + node::Reversible + Keyed,
    ReverseNodeOf<E>: Weighted + Keyed,
{
    expander: Arc<E>,
    reverser: Arc<E::Reverse>,
    forward_trees: TreeCache<E>,
    reverse_trees: TreeCache<E::Reverse>,
    solutions: SolutionCache<E>,
}

impl<E> BidirGarden<E>
where
    E: Aimless + BidirSolvable + Closable,
    E::Reverse: Aimless + Closable,
    NodeOf<E>: Weighted + node::Reversible + Keyed,
    ReverseNodeOf<E>: Weighted + Keyed,
{
    pub fn new(expander: Arc<E>) -> Result<Self, ReversalErrorOf<E>> {
        let reverser = expander.reverse()?;
        let reverser = Arc::new(reverser);
        Ok(Self{
            expander,
            reverser,
            forward_trees: Default::default(),
            reverse_trees: Default::default(),
            solutions: Default::default(),
        })
    }

    pub fn solve<S, G>(&self, from: &S, to: &G) -> Result<Option<E::Solution>, BidirError<E, S, G>>
    where
        E: InitAimless<S>,
        E::Reverse: InitAimless<G>,
        E::Solution: Clone + Weighted,
        E::Node: Weighted,
        ReverseNodeOf<E>: Weighted<Cost=CostOf<E>>,
    {
        // TODO(MXG): It should be possible to parallelize some of this effort.
        // We should look into how to use async and runtimes here.
        let mut best_solution = Minimum::new(|u: &E::Solution, v: &E::Solution| { u.cost().cmp(&v.cost()) });

        // Note: We use None for goal here because we are not interested in
        // doing an informed search. Calculating a heuristic would be a waste of
        // effort.
        for forward in self.expander.aimless_start(from) {
            let forward = forward.map_err(BidirInitError::Forward)?;
            let key_f = forward.key();
            for reverse in self.reverser.aimless_start(to) {
                let reverse = reverse.map_err(BidirInitError::Reverse)?;
                let key_r = reverse.key();
                {
                    let guard = self.solutions.lock().map_err(|_| BidirExpansionError::PoisonedMutex)?;
                    if let Some(prior) = guard.borrow().get(&(key_f.clone(), key_r.clone())) {
                        if let Some(solution) = prior {
                            best_solution.consider(solution);
                        }

                        // We have already solved or exhausted this pair of trees,
                        // so just return the result that we got in the past.
                        continue;
                    };
                }

                if let Some((f, r)) = self.grow_best_connection(forward.clone(), key_f.clone(), reverse.clone(), key_r.clone())? {
                    let solution = self.expander.make_bidirectional_solution(&f, &r).map_err(BidirExpansionError::Solve)?;
                    best_solution.consider(&solution);
                    self.solutions.lock().map_err(|_| BidirInitError::PoisonedMutex)?.borrow_mut().insert((key_f.clone(), key_r.clone()), Some(solution));
                } else {
                    self.solutions.lock().map_err(|_| BidirInitError::PoisonedMutex)?.borrow_mut().insert((key_f.clone(), key_r.clone()), None);
                }
            }
        }

        return Ok(best_solution.result());
    }

    fn grow_best_connection(
        &self,
        forward: Arc<E::Node>,
        key_f: <E::Node as PartialKeyed>::Key,
        reverse: Arc<<E::Reverse as Expander>::Node>,
        key_r: <E::Node as PartialKeyed>::Key,
    ) -> Result<Option<(Arc<E::Node>, Arc<<E::Reverse as Expander>::Node>)>, BidirExpansionError<E>>
    where
        E::Node: Weighted,
        ReverseNodeOf<E>: Weighted<Cost=CostOf<E>>,
    {
        let mut best_connection = Minimum::new(
            |u: &(Arc<E::Node>, Arc<<E::Reverse as Expander>::Node>), v: &(Arc<E::Node>, Arc<<E::Reverse as Expander>::Node>)| {
                (u.0.cost() + u.1.cost()).cmp(&(v.0.cost() + v.1.cost()))
            }
        );

        let forward_tree_arc = {
            // The forward_trees map is locked within these braces. We
            // lock it to get access to the tree (or create it) and then
            // we unlock it to avoid blocking any other threads trying
            // to use this Garden.
            self.forward_trees.lock()
                .map_err(|_| BidirExpansionError::PoisonedMutex)?
                .borrow_mut()
                .entry(key_f)
                .or_insert_with(|| new_tree_entry(Tree::new(forward, self.expander.clone())))
                .clone()
        };
        let forward_tree_guard = forward_tree_arc.lock().map_err(|_| BidirExpansionError::PoisonedMutex)?;
        let mut forward_tree = forward_tree_guard.borrow_mut();

        let reverse_tree_arc = {
            self.reverse_trees.lock()
                .map_err(|_| BidirExpansionError::PoisonedMutex)?
                .borrow_mut()
                .entry(key_r)
                .or_insert_with(|| new_tree_entry(Tree::new(reverse, self.reverser.clone())))
                .clone()
        };
        let reverse_tree_guard = reverse_tree_arc.lock().map_err(|_| BidirExpansionError::PoisonedMutex)?;
        let mut reverse_tree = reverse_tree_guard.borrow_mut();

        let mut connections = ConnectionMap::<E>::new();
        for node in forward_tree.closed().iter() {
            connections.insert(node.key().clone(), (Some(node.clone()), None));
        }

        for node in reverse_tree.closed().iter() {
            let connection = connections.entry(node.key().clone())
                .or_insert((None, Some(node.clone())));

            if let (Some(f), Some(r)) = connection {
                best_connection.consider_take((f.clone(), r.clone()));
            }
        }

        if best_connection.has_value() {
            // The trees already have a connection, so there is no need to grow
            // either of them.
            return Ok(best_connection.result());
        }

        while !forward_tree.is_exhausted() || !reverse_tree.is_exhausted() {
            for node in forward_tree.grow() {
                let node = node.map_err(BidirExpansionError::Forward)?;
                let connection = connections.entry(node.key().clone())
                    .or_insert((Some(node), None));

                if let (Some(f), Some(r)) = connection {
                    best_connection.consider_take((f.clone(), r.clone()));
                }
            }

            if best_connection.has_value() {
                // No need to grow the reverse tree because we already found a
                // connection.
                return Ok(best_connection.result());
            }

            for node in reverse_tree.grow() {
                let node = node.map_err(BidirExpansionError::Reverse)?;
                let connection = connections.entry(node.key().clone())
                    .or_insert((None, Some(node)));

                if let (Some(f), Some(r)) = connection {
                    best_connection.consider_take((f.clone(), r.clone()));
                }
            }

            if best_connection.has_value() {
                return Ok(best_connection.result());
            }
        }

        assert!(!best_connection.has_value());
        return Ok(None);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        motion::r2::{Point, graph_search::make_default_expander, timed_position::LineFollow},
        directed::SimpleGraph,
    };
    use std::sync::Arc;

    fn make_test_graph() -> SimpleGraph<Point> {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
        */

        let mut vertices = Vec::<Point>::new();
        vertices.push(Point::new(0.0, 0.0)); // 0
        vertices.push(Point::new(1.0, 0.0)); // 1
        vertices.push(Point::new(2.0, 0.0)); // 2
        vertices.push(Point::new(3.0, 0.0)); // 3
        vertices.push(Point::new(1.0, -1.0)); // 4
        vertices.push(Point::new(2.0, -1.0)); // 5
        vertices.push(Point::new(3.0, -1.0)); // 6
        vertices.push(Point::new(2.0, -2.0)); // 7
        vertices.push(Point::new(3.0, -2.0)); // 8

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

    #[test]
    fn test_simple_garden() {
        let garden = Garden::new(Arc::new(
            make_default_expander(
                Arc::new(make_test_graph()),
                Arc::new(LineFollow::new(1.0).unwrap()),
            )
        ));

        let solution = garden.solve(&0, &8).unwrap();
        assert!(solution.is_some());
        print!("{solution:?}");
    }
}
