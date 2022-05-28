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

use super::simple::Graph;
use crate::motion::{
    self, Extrapolation,
    trajectory::{Trajectory, CostCalculator},
    r2::{
        Position,
        timed_position::{LineFollow, Waypoint},
    },
};
use crate::node::{Cost as NodeCost, HashOption, HashOptionClosedSet};
use std::{
    hash::{Hash, Hasher},
    rc::Rc,
};
use time_point::TimePoint;
use num::Zero;

#[derive(Clone, Debug)]
pub struct Node<Cost: NodeCost> {
    cost: Cost,
    remaining_cost_estimate: Cost,
    total_cost_estimate: Cost,
    pub state: Waypoint,
    pub vertex: usize,
    pub motion_from_parent: Option<Trajectory<Waypoint>>,
    pub parent: Option<Rc<Node<Cost>>>,
}

impl<Cost: NodeCost> HashOption for Node<Cost> {
    fn hash_opt<H: Hasher>(&self, state: &mut H) -> bool {
        self.vertex.hash(state);
        return true;
    }
}

impl<Cost: NodeCost> crate::Node for Node<Cost> {
    type Cost = Cost;
    type ClosedSet = HashOptionClosedSet<Self>;

    fn cost(&self) -> Self::Cost {
        return self.cost;
    }

    fn parent(&self) -> &Option<Rc<Self>> {
        return &self.parent;
    }
}

pub trait Heuristic<Cost: NodeCost> {
    fn estimate_cost(&self, from_vertex: usize, to_goal: usize) -> Option<Cost>;
}

pub trait Policy {
    type Cost: crate::Cost;
    type CostCalculator: CostCalculator<Waypoint, Cost=Self::Cost>;
    type Heuristic: Heuristic<Self::Cost>;
}

type NodeType<P> = Node<<P as Policy>::Cost>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Goal(usize);

impl<C: NodeCost> crate::expander::Goal<Node<C>> for Goal {
    fn is_satisfied(&self, node: &Node<C>) -> bool {
        return node.vertex == self.0;
    }
}

pub struct Expander<P: Policy> {
    graph: Rc<Graph<Position>>,
    extrapolation: Rc<LineFollow>,
    cost_calculator: Rc<P::CostCalculator>,
    heuristic: P::Heuristic,
}

impl<P: Policy> Expander<P> {
    pub fn new(
        graph: Rc<Graph<Position>>,
        extrapolation: Rc<LineFollow>,
        cost_calculator: Rc<P::CostCalculator>,
        heuristic: P::Heuristic,
    ) -> Self {
        Self{graph, extrapolation, cost_calculator, heuristic}
    }

    fn make_node(
        &self,
        state: Waypoint,
        vertex: usize,
        remaining_cost_estimate: P::Cost,
        motion: Option<Trajectory<Waypoint>>,
        parent: &Rc<Node<P::Cost>>,
    ) -> Rc<NodeType<P>> {
        let added_cost = motion.as_ref().map(
            |trajectory| self.cost_calculator.compute_cost(&trajectory)
        ).unwrap_or(P::Cost::zero());

        let cost = parent.cost + added_cost;
        return Rc::new(Node{
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

impl<P: Policy> crate::Expander for Expander<P> {
    type Node = NodeType<P>;
    type Start = usize;
    type Goal = Goal;
    type Options = ();
    type Solution = Option<motion::r2::LinearTrajectory>;
    type InitialNodes<'a> where P: 'a = InitialNodes<'a, P>;
    type Expansion<'a> where P: 'a = Expansion<'a, P>;

    fn default_options(&self) -> Self::Options {
        return ();
    }

    fn start<'a>(&'a self, start: &'a Self::Start, goal: &'a Self::Goal) -> Self::InitialNodes<'a> {
        InitialNodes{
            start: *start,
            expanded: false,
            expander: self,
            goal: goal.0,
        }
    }

    fn expand<'a>(&'a self, parent: &Rc<Self::Node>, goal: &'a Self::Goal, _: &'a Self::Options) -> Self::Expansion<'a> {
        Expansion{
            from_node: parent.clone(),
            expansion_index: 0,
            expander: self,
            goal: goal.0,
        }
    }

    fn make_solution(&self, solution_node: &Rc<Self::Node>, _: &Self::Options) -> Self::Solution {
        let mut node = solution_node.clone();
        let mut waypoints = Vec::new();
        loop {
            if let Some(next_waypoints) = &node.motion_from_parent {
                for wp in next_waypoints.iter().rev() {
                    waypoints.push(*wp.clone());
                }
            }

            match &node.parent {
                Some(parent) => { node = parent.clone() },
                None => { break; }
            }
        }

        waypoints.reverse();
        return Trajectory::from_iter(waypoints.into_iter()).ok();
    }
}

pub struct InitialNodes<'a, P: Policy> {
    start: usize,
    expanded: bool,
    expander: &'a Expander<P>,
    goal: usize,
}

impl<'a, P: Policy> Iterator for InitialNodes<'a, P> {
    type Item = Rc<Node<P::Cost>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.expanded {
            return None;
        }
        self.expanded = true;

        if let Some(remaining_cost_estimate) = self.expander.heuristic
            .estimate_cost(self.start, self.goal)
        {
            let p = self.expander.graph.vertices.get(self.start)?;
            return Some(Rc::new(Node{
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
            }));
        }

        return None;
    }
}

pub struct Expansion<'a, P: Policy> {
    from_node: Rc<NodeType<P>>,
    expansion_index: usize,
    expander: &'a Expander<P>,
    goal: usize,
}

impl<'a, P: Policy> Iterator for Expansion<'a, P> {
    type Item = Rc<Node<P::Cost>>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(to_vertices) = self.expander.graph.edges.get(self.from_node.vertex) {
                if let Some(to_vertex) = to_vertices.get(self.expansion_index) {
                    self.expansion_index += 1;
                    if let Some(remaining_cost_estimate) = self.expander.heuristic.estimate_cost(
                        self.from_node.vertex, *to_vertex
                    ) {
                        if let Some(to_target) = self.expander.graph.vertices.get(*to_vertex) {
                            let waypoints_opt = self.expander.extrapolation.extrapolate(
                                &self.from_node.state,
                                &to_target
                            );

                            if let Ok(mut waypoints) = waypoints_opt {
                                let state = waypoints.last().unwrap_or(&self.from_node.state).clone();
                                waypoints.insert(0, self.from_node.state);
                                let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
                                return Some(self.expander.make_node(
                                    state, *to_vertex, remaining_cost_estimate, trajectory, &self.from_node,
                                ));
                            }
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
