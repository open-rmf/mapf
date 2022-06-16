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
use crate::node::{self, PartialKeyed, Keyed, ClosedSet, KeyOf, Weighted};
use crate::expander::{Goal, Expander, Initializable, Expandable, Aimless, Closable, Reversible, BidirSolvable, CostOf, NodeOf, ReverseOf, ReverseNodeOf, SolutionOf, InitErrorOf, AimlessErrorOf, ReversalErrorOf, BidirSolveErrorOf};
use crate::util::Minimum;
use std::collections::hash_map::HashMap;
use std::sync::{Arc, Mutex};
use std::cell::RefCell;

type MutexRefCell<T> = Mutex<RefCell<T>>;

#[derive(Debug)]
pub enum InitError<E, S, G>
where
    E: Initializable<S, G> + Reversible<S, G>,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    Forward(InitErrorOf<E, S, G>),
    Reverse(InitErrorOf<ReverseOf<E, S, G>, G, S>),
    PoisonedMutex,
}

#[derive(Debug)]
pub enum ExpansionError<E, S, G>
where
    E: Initializable<S, G> + Aimless + BidirSolvable<S, G>,
    ReverseOf<E, S, G>: Aimless,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    Forward(AimlessErrorOf<E>),
    Reverse(AimlessErrorOf<ReverseOf<E, S, G>>),
    Solve(BidirSolveErrorOf<E, S, G>),
    PoisonedMutex,
}

#[derive(Debug)]
pub enum Error<E, S, G>
where
    E: Initializable<S, G> + Aimless + BidirSolvable<S, G>,
    ReverseOf<E, S, G>: Aimless,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    Init(InitError<E, S, G>),
    Expansion(ExpansionError<E, S, G>),
}

impl<E, S, G> From<InitError<E, S, G>> for Error<E, S, G>
where
    E: Initializable<S, G> + Aimless + BidirSolvable<S, G>,
    ReverseOf<E, S, G>: Aimless,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    fn from(e: InitError<E, S, G>) -> Self {
        Error::Init(e)
    }
}

impl<E, S, G> From<ExpansionError<E, S, G>> for Error<E, S, G>
where
    E: Initializable<S, G> + Aimless + BidirSolvable<S, G>,
    ReverseOf<E, S, G>: Aimless,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    fn from(e: ExpansionError<E, S, G>) -> Self {
        Error::Expansion(e)
    }
}

// type TreeCache<E: Expander> where E::Node: HashOption = MutexRefCell<HashMap<<E::Node as node::HashOption>::Key, MutexRefCell<Tree<E>>>>;
type TreeCache<E> = MutexRefCell<HashMap<KeyOf<NodeOf<E>>, Arc<MutexRefCell<Tree<E>>>>>;
type SolutionCache<E> = MutexRefCell<HashMap<(KeyOf<NodeOf<E>>, KeyOf<NodeOf<E>>), Option<SolutionOf<E>>>>;
type ConnectionMap<E, S, G> = HashMap<KeyOf<NodeOf<E>>, (Option<Arc<NodeOf<E>>>, Option<Arc<NodeOf<ReverseOf<E, S, G>>>>)>;

pub struct Garden<E: Aimless + BidirSolvable<S, G> + Closable, S, G>
where
    NodeOf<E>: Weighted + node::Reversible + Keyed,
    ReverseNodeOf<E, S, G>: Weighted + Keyed,
    ReverseOf<E, S, G>: Aimless + Closable,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
{
    expander: Arc<E>,
    reverser: Arc<E::Reverse>,
    forward_trees: TreeCache<E>,
    reverse_trees: TreeCache<E::Reverse>,
    solutions: SolutionCache<E>,
}

impl<E, S, G> Garden<E, S, G>
where
    E: Initializable<S, G> + Expandable<G> + Aimless + BidirSolvable<S, G> + Closable,
    S: Goal<ReverseNodeOf<E, S, G>>,
    G: Goal<E::Node>,
    E::Node: node::Weighted + node::Reversible + node::Keyed,
    E::Solution: node::Weighted + Clone,
    ReverseNodeOf<E, S, G>: node::Weighted<Cost=CostOf<E>> + node::Keyed,
    ReverseOf<E, S, G>: Aimless + Closable,
{
    pub fn new(expander: Arc<E>) -> Result<Self, ReversalErrorOf<E, S, G>> {
        let reverser = expander.reverse()?;
        Ok(Self{
            expander,
            reverser,
            forward_trees: Default::default(),
            reverse_trees: Default::default(),
            solutions: Default::default(),
        })
    }

    pub fn solve(&self, from: &S, to: &G) -> Result<Option<E::Solution>, Error<E, S, G>> {
        // TODO(MXG): It should be possible to parallelize some of this effort.
        // We should look into how to use async and runtimes here.
        let mut best_solution = Minimum::new(|u: &E::Solution, v: &E::Solution| { u.cost().cmp(&v.cost()) });

        // Note: We use None for goal here because we are not interested in
        // doing an informed search. Calculating a heuristic would be a waste of
        // effort.
        for forward in self.expander.start(from, to) {
            let forward = forward.map_err(InitError::Forward)?;
            let key_f = forward.key();
            for reverse in self.reverser.start(to, from) {
                let reverse = reverse.map_err(InitError::Reverse)?;
                let key_r = reverse.key();
                {
                    let guard = self.solutions.lock().map_err(|_| ExpansionError::PoisonedMutex)?;
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
                    let solution = self.expander.make_bidirectional_solution(&f, &r).map_err(ExpansionError::Solve)?;
                    best_solution.consider(&solution);
                    self.solutions.lock().map_err(|_| InitError::PoisonedMutex)?.borrow_mut().insert((key_f.clone(), key_r.clone()), Some(solution));
                } else {
                    self.solutions.lock().map_err(|_| InitError::PoisonedMutex)?.borrow_mut().insert((key_f.clone(), key_r.clone()), None);
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
    ) -> Result<Option<(Arc<E::Node>, Arc<<E::Reverse as Expander>::Node>)>, ExpansionError<E, S , G>> {

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
                .map_err(|_| ExpansionError::PoisonedMutex)?
                .borrow_mut()
                .entry(key_f)
                .or_insert_with(|| Arc::new(Mutex::new(RefCell::new(Tree::new(forward, self.expander.clone())))))
                .clone()
        };
        let forward_tree_guard = forward_tree_arc.lock().map_err(|_| ExpansionError::PoisonedMutex)?;
        let mut forward_tree = forward_tree_guard.borrow_mut();

        let reverse_tree_arc = {
            self.reverse_trees.lock()
                .map_err(|_| ExpansionError::PoisonedMutex)?
                .borrow_mut()
                .entry(key_r)
                .or_insert_with(|| Arc::new(Mutex::new(RefCell::new(Tree::new(reverse, self.reverser.clone())))))
                .clone()
        };
        let reverse_tree_guard = reverse_tree_arc.lock().map_err(|_| ExpansionError::PoisonedMutex)?;
        let mut reverse_tree = reverse_tree_guard.borrow_mut();

        let mut connections = ConnectionMap::<E, S, G>::new();
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
                let node = node.map_err(ExpansionError::Forward)?;
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
                let node = node.map_err(ExpansionError::Reverse)?;
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

}
