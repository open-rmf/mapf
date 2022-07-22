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
    node::{self, PartialKeyed, Keyed, ClosedSet, KeyOf, Weighted},
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

type MutexRefCell<T> = Mutex<RefCell<T>>;

#[derive(Debug)]
pub enum InitError<E, S, G>
where
    E: InitAimless<S> + Reversible,
    E::Reverse: InitAimless<G>,
{
    Forward(InitAimlessErrorOf<E, S>),
    Reverse(InitAimlessErrorOf<ReverseOf<E>, G>),
    PoisonedMutex,
}

#[derive(Debug)]
pub enum ExpansionError<E>
where
    E: Aimless + BidirSolvable,
    E::Reverse: Aimless,
{
    Forward(AimlessErrorOf<E>),
    Reverse(AimlessErrorOf<ReverseOf<E>>),
    Solve(BidirSolveErrorOf<E>),
    PoisonedMutex,
}

#[derive(Debug)]
pub enum Error<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
    ReverseOf<E>: Aimless,
{
    Init(InitError<E, S, G>),
    Expansion(ExpansionError<E>),
}

impl<E, S, G> From<InitError<E, S, G>> for Error<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
{
    fn from(e: InitError<E, S, G>) -> Self {
        Error::Init(e)
    }
}

impl<E, S, G> From<ExpansionError<E>> for Error<E, S, G>
where
    E: InitAimless<S> + Aimless + BidirSolvable,
    E::Reverse: InitAimless<G> + Aimless,
{
    fn from(e: ExpansionError<E>) -> Self {
        Error::Expansion(e)
    }
}

// type TreeCache<E: Expander> where E::Node: HashOption = MutexRefCell<HashMap<<E::Node as node::HashOption>::Key, MutexRefCell<Tree<E>>>>;
type TreeCache<E> = MutexRefCell<HashMap<KeyOf<NodeOf<E>>, Arc<MutexRefCell<Tree<E>>>>>;
type SolutionCache<E> = MutexRefCell<HashMap<(KeyOf<NodeOf<E>>, KeyOf<NodeOf<E>>), Option<SolutionOf<E>>>>;
type ConnectionMap<E> = HashMap<KeyOf<NodeOf<E>>, (Option<Arc<NodeOf<E>>>, Option<Arc<NodeOf<ReverseOf<E>>>>)>;

pub struct Garden<E>
where
    E: Aimless + Reversible + Solvable + Closable,
    E::Reverse: Aimless + Closable + Solvable + Closable,
    NodeOf<E>: Weighted + Keyed,
    ReverseNodeOf<E>: Weighted + Keyed,
{
    expander: Arc<E>,
    trees: TreeCache<E>,
    solutions: SolutionCache<E>,
}

pub struct BidirectionalGarden<E>
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

impl<E> BidirectionalGarden<E>
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

    pub fn solve<S, G>(&self, from: &S, to: &G) -> Result<Option<E::Solution>, Error<E, S, G>>
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
        for forward in self.expander.start(from) {
            let forward = forward.map_err(InitError::Forward)?;
            let key_f = forward.key();
            for reverse in self.reverser.start(to) {
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
    ) -> Result<Option<(Arc<E::Node>, Arc<<E::Reverse as Expander>::Node>)>, ExpansionError<E>>
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
