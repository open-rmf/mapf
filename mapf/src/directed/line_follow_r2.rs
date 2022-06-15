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

use super::simple::SimpleGraph;
use crate::expander::{Closable, Initializable, Expandable, Solvable, NodeOf, InitErrorOf, ExpansionErrorOf, ReverseOf};
use crate::motion::{
    self, Extrapolator,
    extrapolator::{self, Reversible},
    timed::Timed,
    trajectory::{Trajectory, CostCalculator},
    r2::{
        Position,
        timed_position::{LineFollow, Waypoint},
    },
};
use crate::node::{self, Cost as NodeCost, PartialKeyed, PartialKeyedClosedSet};
use std::{
    cell::RefCell,
    sync::{Arc, Mutex},
};
use time_point::TimePoint;
use num::Zero;
use derivative::Derivative;

#[derive(Clone, Debug)]
pub struct Node<Cost: NodeCost> {
    cost: Cost,
    remaining_cost_estimate: Cost,
    total_cost_estimate: Cost,
    pub state: Waypoint,
    pub vertex: usize,
    pub motion_from_parent: Option<Trajectory<Waypoint>>,
    pub parent: Option<Arc<Node<Cost>>>,
}

impl<C: NodeCost> node::Weighted for Node<C> {
    type Cost = C;
    fn cost(&self) -> Self::Cost {
        return self.cost;
    }
}

impl<C: NodeCost> node::PathSearch for Node<C> {
    fn parent(&self) -> &Option<Arc<Self>> {
        return &self.parent;
    }
}

impl<Cost: NodeCost> PartialKeyed for Node<Cost> {
    type Key = usize;

    fn key(&self) -> Option<&Self::Key> {
        Some(&self.vertex)
    }
}

impl<Cost: NodeCost> node::Informed for Node<Cost> {
    fn remaining_cost_estimate(&self) -> Self::Cost {
        self.remaining_cost_estimate
    }

    fn total_cost_estimate(&self) -> Self::Cost {
        self.total_cost_estimate
    }
}

impl<Cost: NodeCost> node::Reversible for Node<Cost> {
    type Reverse = Self;
}

pub trait Heuristic<C: NodeCost> {
    type EstimationError: std::error::Error;
    fn estimate_cost(
        &self,
        from_vertex: usize,
        to_goal: Option<usize>
    ) -> Result<Option<C>, Self::EstimationError>;
}

pub trait Policy {
    type Cost: NodeCost;
    type CostCalculator: CostCalculator<Waypoint, Cost=Self::Cost>;
    type Heuristic: Heuristic<Self::Cost>;
}

type NodeType<P> = Node<<P as Policy>::Cost>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Goal(usize);

impl<C: NodeCost> crate::expander::Goal<Node<C>> for usize {
    fn is_satisfied(&self, node: &Node<C>) -> bool {
        return node.vertex == *self;
    }
}

pub struct Expander<P: Policy> {
    graph: Arc<SimpleGraph<Position>>,
    extrapolator: Arc<LineFollow>,
    cost_calculator: Arc<P::CostCalculator>,
    heuristic: Arc<P::Heuristic>,
    reverse: Mutex<RefCell<Option<Arc<Expander<P>>>>>,
}

impl<P: Policy> Expander<P> {
    pub fn new(
        graph: Arc<SimpleGraph<Position>>,
        extrapolator: Arc<LineFollow>,
        cost_calculator: Arc<P::CostCalculator>,
        heuristic: Arc<P::Heuristic>,
    ) -> Self {
        Self{graph, extrapolator, cost_calculator, heuristic, reverse: Mutex::new(RefCell::new(None))}
    }

    fn make_node(
        &self,
        state: Waypoint,
        vertex: usize,
        remaining_cost_estimate: P::Cost,
        motion: Option<Trajectory<Waypoint>>,
        parent: &Arc<Node<P::Cost>>,
    ) -> Arc<NodeType<P>> {
        let added_cost = motion.as_ref().map(
            |trajectory| self.cost_calculator.compute_cost(&trajectory)
        ).unwrap_or(P::Cost::zero());

        let cost = parent.cost + added_cost;
        return Arc::new(Node{
            cost,
            remaining_cost_estimate,
            total_cost_estimate: cost + remaining_cost_estimate,
            state,
            vertex,
            motion_from_parent: motion,
            parent: Some(parent.clone()),
        });
    }
}

// impl<P: Policy> expander::Reversible for Expander<P> {
//     type Reverse = Expander<P>;
//     type ReversalError = ReversalError;
//     type BidirSolveError = BidirSolveError;

//     fn reverse(&self) -> Result<Arc<Self::Reverse>, ReversalError> {
//         let reversal = self.extrapolator.reverse().map_err(ReversalError::Extrapolator)?;
//         Ok(self.reverse.lock().map_err(|_| ReversalError::PoisonedMutex)?.borrow_mut()
//             .get_or_insert_with(|| {
//                 Arc::new(
//                     Expander::new(
//                         Arc::new(self.graph.reverse()),
//                         Arc::new(reversal),
//                         self.cost_calculator,
//                         self.heuristic,
//                     )
//                 )
//             }
//         ).clone())
//     }

//     fn make_bidirectional_solution(
//         &self,
//         forward_solution_node: &Arc<Self::Node>,
//         reverse_solution_node: &Arc<NodeOf<ReverseOf<Self>>>,
//     ) -> Result<Self::Solution, BidirSolveError> {
//         let mut node = forward_solution_node.clone();
//         let mut waypoints = Vec::new();
//         loop {

//         }
//     }
// }

#[derive(Derivative)]
#[derivative(Debug, Clone)]
pub struct Solution<P: Policy> {
    cost: P::Cost,
    motion: Option<motion::r2::LinearTrajectory>,
}

impl<P: Policy> Solution<P> {
    pub fn motion(&self) -> &Option<motion::r2::LinearTrajectory> {
        &self.motion
    }
}

impl<P: Policy> node::Weighted for Solution<P> {
    type Cost = P::Cost;
    fn cost(&self) -> P::Cost {
        self.cost
    }
}

#[derive(Debug, Clone)]
pub enum ReversalError {
    /// The expander may use mutexes to protect cache read/writes across multiple
    /// threads. If a mutex gets [poisoned](https://doc.rust-lang.org/nomicon/poisoning.html)
    /// then this error will be raised.
    PoisonedMutex,
    Extrapolator(<LineFollow as extrapolator::Reversible<Waypoint, Position>>::Error)
}

/// Returned by make_bidirectional_solution
#[derive(Debug, Clone)]
pub enum BidirSolveError {
    /// The forward and reverse nodes do not have matching keys.
    KeyMismatch
}

#[derive(Derivative)]
#[derivative(Debug)]
pub enum ExpansionError<P: Policy> {
    Extrapolator(<LineFollow as Extrapolator<Waypoint, Position>>::Error),
    Heuristic(<P::Heuristic as Heuristic<P::Cost>>::EstimationError),
}

impl<P: Policy> Initializable<usize, NodeType<P>, usize> for Expander<P> {
    type InitError = <P::Heuristic as Heuristic<P::Cost>>::EstimationError;
    type InitialNodes<'a> where P: 'a = InitialNodes<'a, P>;

    fn start<'a>(&'a self, start: &'a usize, goal: Option<&'a usize>) -> Self::InitialNodes<'a> {
        InitialNodes{
            start: *start,
            expanded: false,
            expander: self,
            goal: goal.map(|g| *g),
        }
    }
}

impl<P: Policy> Expandable<NodeType<P>, usize> for Expander<P> {
    type ExpansionError = ExpansionError<P>;
    type Expansion<'a> where P: 'a = Expansion<'a, P>;

    fn expand<'a>(&'a self, parent: &Arc<NodeType<P>>, goal: Option<&'a usize>) -> Self::Expansion<'a> {
        Expansion{
            from_node: parent.clone(),
            expansion_index: 0,
            expander: self,
            goal: goal.map(|g| *g),
        }
    }
}

impl<P: Policy> Solvable<NodeType<P>> for Expander<P> {
    type SolveError = ();
    type Solution = Solution<P>;

    fn make_solution(&self, solution_node: &Arc<NodeType<P>>) -> Result<Self::Solution, Self::SolveError> {
        Ok(Solution{
            cost: solution_node.cost,
            motion: Trajectory::from_iter(ReconstructMotion::new(solution_node.clone())).ok(),
        })
    }
}

impl<P: Policy> crate::Expander for Expander<P> {
    type Node = NodeType<P>;
    type Start = usize;
    type Goal = usize;
}

impl<P: Policy> Closable<NodeType<P>> for Expander<P> {
    type ClosedSet = PartialKeyedClosedSet<NodeType<P>>;
}

struct ReconstructMotion<C: NodeCost> {
    node: Option<Arc<Node<C>>>,
    index: usize,
    shift: time_point::Duration,
}

impl<C: NodeCost> ReconstructMotion<C> {
    fn new(root: Arc<Node<C>>) -> Self {
        Self{
            node: Some(root),
            index: 0,
            shift: time_point::Duration::zero(),
        }
    }

    fn shifted(mut self, shift: time_point::Duration) -> Self {
        self.shift = shift;
        self
    }
}

impl<C: NodeCost> Iterator for ReconstructMotion<C> {
    type Item = Waypoint;

    fn next(&mut self) -> Option<Waypoint> {
        loop {
            if let Some(node) = &self.node {
                if let Some(motion) = &node.motion_from_parent {
                    if self.index < motion.len() {
                        self.index += 1;
                        let mut wp = motion[self.index - 1].0;
                        wp.set_time(*wp.time() + self.shift);
                        return Some(wp);
                    }
                }

                self.node = node.parent.clone();
                self.index = 0;
                continue;
            }

            return None;
        }
    }
}

pub struct InitialNodes<'a, P: Policy> {
    start: usize,
    expanded: bool,
    expander: &'a Expander<P>,
    goal: Option<usize>,
}

impl<'a, P: Policy> Iterator for InitialNodes<'a, P> {
    type Item = Result<Arc<Node<P::Cost>>, InitErrorOf<Expander<P>>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.expanded {
            return None;
        }
        self.expanded = true;

        let estimate = match self.expander.heuristic.estimate_cost(self.start, self.goal) {
            Ok(value) => value,
            Err(e) => { return Some(Err(e)); }
        };

        if let Some(remaining_cost_estimate) = estimate {
            let p = self.expander.graph.vertices.get(self.start)?;
            return Some(Ok(Arc::new(Node{
                cost: P::Cost::zero(),
                remaining_cost_estimate,
                total_cost_estimate: remaining_cost_estimate,
                state: Waypoint{
                    time: TimePoint::zero(),
                    position: *p,
                },
                vertex: self.start,
                motion_from_parent: None,
                parent: None,
            })));
        }

        return None;
    }
}

pub struct Expansion<'a, P: Policy> {
    from_node: Arc<NodeType<P>>,
    expansion_index: usize,
    expander: &'a Expander<P>,
    goal: Option<usize>,
}

impl<'a, P: Policy> Iterator for Expansion<'a, P> {
    type Item = Result<Arc<Node<P::Cost>>, ExpansionErrorOf<Expander<P>>>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(to_vertices) = self.expander.graph.edges.get(self.from_node.vertex) {
                if let Some(to_vertex) = to_vertices.get(self.expansion_index) {
                    self.expansion_index += 1;
                    let estimate = match self.expander.heuristic.estimate_cost(
                        *to_vertex, self.goal
                    ).map_err(ExpansionError::Heuristic) {
                        Ok(estimate) => estimate,
                        Err(e) => { return Some(Err(e)); }
                    };

                    if let Some(remaining_cost_estimate) = estimate {
                        if let Some(to_target) = self.expander.graph.vertices.get(*to_vertex) {
                            let mut waypoints = match self.expander.extrapolator.extrapolate(
                                &self.from_node.state,
                                &to_target
                            ).map_err(ExpansionError::Extrapolator) {
                                Ok(value) => value,
                                Err(e) => { return Some(Err(e)); }
                            };

                            let state = waypoints.last().unwrap_or(&self.from_node.state).clone();
                            waypoints.insert(0, self.from_node.state);
                            let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
                            return Some(Ok(self.expander.make_node(
                                state, *to_vertex, remaining_cost_estimate, trajectory, &self.from_node,
                            )));
                        }
                    }

                    // If we reach this point then we did not manage to produce
                    // an expansion on this loop, but may be able to in the next
                    // loop.
                    continue;
                }
            }

            return None;
        }
    }
}
