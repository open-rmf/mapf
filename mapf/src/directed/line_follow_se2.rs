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
    Extrapolation,
    trajectory::{Trajectory, CostCalculator},
    se2::{
        Position,
        timed_position::{
            DifferentialDriveLineFollow,
            Waypoint
        },
    }
};
use crate::node::{Cost as NodeCost, HashOption, HashOptionClosedSet};
use std::{
    hash::{Hash, Hasher},
    rc::Rc,
    cell::RefCell,
};
use nalgebra::{UnitComplex as Rotation, Vector2};
use time_point::TimePoint;
use cached::{Cached, UnboundCache};
use num::Zero;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Side {
    Beginning,
    Finish
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Key {
    from_vertex: usize,
    to_vertex: usize,
    side: Side,
}

#[derive(Clone, Debug)]
pub struct Node<Cost: NodeCost> {
    cost: Cost,
    remaining_cost_estimate: Cost,
    total_cost_estimate: Cost,
    pub state: Waypoint,
    pub key: Option<Key>,
    pub vertex: usize,
    pub motion_from_parent: Option<Trajectory<Waypoint>>,
    pub parent: Option<Rc<Node<Cost>>>,
    pub is_start: Option<Start>,
}

impl<Cost: NodeCost> HashOption for Node<Cost> {
    fn hash_opt<H: Hasher>(&self, state: &mut H) -> bool {
        match self.key {
            Some(key) => {
                key.hash(state);
                return true;
            },
            None => false
        }
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

impl<Cost: NodeCost> crate::node::Informed for Node<Cost> {
    fn remaining_cost_estimate(&self) -> Self::Cost {
        return self.remaining_cost_estimate;
    }

    fn total_cost_estimate(&self) -> Self::Cost {
        return self.total_cost_estimate;
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Start {
    vertex: usize,
    orientation: Rotation<f64>,
    offset_location: Option<Vector2<f64>>,
}

impl Start {
    /// Convert the start value into a waypoint. If the start value has an
    /// invalid vertex, this will return None.
    fn to_waypoint(&self, graph: &Graph<Vector2<f64>>) -> Option<Waypoint> {
        if let Some(location) = self.offset_location {
            return Some(Waypoint{
                time: TimePoint::zero(),
                position: Position::new(location, self.orientation.angle()),
            });
        }

        if let Some(location) = graph.vertices.get(self.vertex) {
            return Some(Waypoint{
                time: TimePoint::zero(),
                position: Position::new(*location, self.orientation.angle()),
            });
        }

        None
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct OrientationGoal {
    target: Rotation<f64>,
    threshold: f64,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Goal {
    vertex: usize,
    orientation: Option<OrientationGoal>,
}

impl<Cost: NodeCost> crate::expander::Goal<Node<Cost>> for Goal {
    fn is_satisfied(&self, node: &Node<Cost>) -> bool {
        if self.vertex != node.vertex {
            return false;
        }

        match self.orientation {
            Some(OrientationGoal{target, threshold}) => {
                let delta_yaw = (node.state.position.rotation / target).angle().abs();
                return delta_yaw <= threshold;
            },
            None => true
        }
    }
}

pub trait Heuristic<Cost: NodeCost> {
    fn new(graph: Rc<Graph<nalgebra::Vector2<f64>>>, to_goal: usize) -> Self;
    fn estimate_cost(&self, from_vertex: usize) -> Option<Cost>;
}

pub trait Policy {
    type Cost: crate::Cost;
    type CostCalculator: CostCalculator<Waypoint, Cost=Self::Cost>;
    type Heuristic: Heuristic<Self::Cost>;
}

type NodeType<P> = Node<<P as Policy>::Cost>;

pub struct InternalExpander<P: Policy> {
    graph: Rc<Graph<nalgebra::Vector2<f64>>>,
    extrapolation: DifferentialDriveLineFollow,
    cost_calculator: P::CostCalculator,
    // TOOD(MXG): Put the heuristic cache in some kind of mutex so that one
    // expander can be nicely shared across threads.
    heuristic_cache: RefCell<UnboundCache<usize, P::Heuristic>>,
}

pub struct Expander<P: Policy> {
    internal: Rc<InternalExpander<P>>,
}

#[derive(Clone, PartialEq, Default)]
pub struct Options {
    // TODO: Come up with some options
}

pub struct StartExpansion<P: Policy> {
    start: Start,
    expanded: bool,
    expander: Rc<InternalExpander<P>>,
    goal_vertex: usize,
}

pub struct NodeExpansion<P: Policy> {
    from_node: Rc<NodeType<P>>,
    expansion_index: usize,
    expanded_start: bool,
    expander: Rc<InternalExpander<P>>,
    goal_vertex: usize,
}

pub enum Expansion<P: Policy> {
    // TODO(MXG): Figure out how we can arrange the lifetimes so that this use
    // of expander is acceptable
    // expander: &'ex Expander<P>,
    Start(StartExpansion<P>),
    Node(NodeExpansion<P>),
}

impl<P: Policy> Expansion<P> {
    fn from_node(expander: &Expander<P>, from_node: Rc<NodeType<P>>, toward_goal: usize) -> Self {
        return Expansion::Node(
            NodeExpansion{
                from_node,
                expansion_index: 0,
                expanded_start: false,
                expander: expander.internal.clone(),
                goal_vertex: toward_goal,
            }
        );
    }

    fn from_start(expander: &Expander<P>, from_start: &Start, toward_goal: usize) -> Self {
        return Expansion::Start(
            StartExpansion{
                start: from_start.clone(),
                expanded: false,
                expander: expander.internal.clone(),
                goal_vertex: toward_goal,
            }
        )
    }
}

impl<P: Policy> NodeExpansion<P> {
    fn expand_to(&self, to_vertex: usize) -> Option<Rc<NodeType<P>>> {
        if let Some(vertex) = self.expander.graph.vertices.get(to_vertex) {
            let extrapolation = self.expander.extrapolation.extrapolate(
                &self.from_node.state,
                vertex
            );
            if let Ok(mut waypoints) = extrapolation {
                let state = waypoints.last().unwrap_or(&self.from_node.state).clone();
                waypoints.insert(0, self.from_node.state);
                let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
                let remaining_cost_estimate = self.expander.heuristic(to_vertex, self.goal_vertex)?;
                return Some(self.expander.make_node(state, to_vertex, remaining_cost_estimate, trajectory, &self.from_node));
            }
            // else: We should propogate these inner expansion errors somehow so
            // users can be aware of unexpected problems.
        }

        return None;
    }

    fn expand_from_start(&self, to_vertex: usize) -> Option<Rc<NodeType<P>>> {
        if let Some(to_target) = self.expander.graph.vertices.get(to_vertex) {
            let extrapolation = self.expander.extrapolation.extrapolate(
                &self.from_node.state,
                to_target
            );
            if let Ok(mut waypoints) = extrapolation {
                let state = waypoints.last().unwrap_or(&self.from_node.state).clone();
                waypoints.insert(0, self.from_node.state);
                let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
                let remaining_cost_estimate = self.expander.heuristic(to_vertex, self.goal_vertex)?;
                return Some(self.expander.make_node(state, to_vertex, remaining_cost_estimate, trajectory, &self.from_node));
            }
        }

        return None;
    }
}

impl<P: Policy> Iterator for Expansion<P> {
    type Item=Rc<Node<P::Cost>>;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Expansion::Node(expansion) => {
                if let Some(start) = &expansion.from_node.is_start {
                    if let Some(_) = &start.offset_location {
                        if expansion.expanded_start {
                            return None;
                        }

                        expansion.expanded_start = true;
                        return expansion.expand_from_start(expansion.from_node.vertex);
                    }
                }

                if let Some(to_vertices) = expansion.expander.graph.edges.get(expansion.from_node.vertex) {
                    if let Some(to_vertex) = to_vertices.get(expansion.expansion_index) {
                        let next = expansion.expand_to(*to_vertex);
                        expansion.expansion_index += 1;
                        return next;
                    }
                }
            },
            Expansion::Start(expansion) => {
                if expansion.expanded {
                    return None;
                }
                expansion.expanded = true;

                if let Some(remaining_cost_estimate) = expansion.expander.heuristic(
                    expansion.start.vertex, expansion.goal_vertex
                ) {
                    let waypoint = expansion.start.to_waypoint(expansion.expander.graph.as_ref())?;
                    return Some(Rc::new(Node{
                        cost: P::Cost::zero(),
                        remaining_cost_estimate,
                        total_cost_estimate: remaining_cost_estimate,
                        state: waypoint,
                        key: None,
                        vertex: expansion.start.vertex,
                        motion_from_parent: None,
                        parent: None,
                        is_start: Some(expansion.start),
                    }))
                }
            }
        }
        return None;
    }
}

impl<P: Policy> InternalExpander<P> {
    fn heuristic(&self, from_vertex: usize, to_goal: usize) -> Option<P::Cost> {
        return self.heuristic_cache.borrow_mut().cache_get_or_set_with(
            to_goal, || { P::Heuristic::new(self.graph.clone(), to_goal) }
        ).estimate_cost(from_vertex);
    }

    fn make_node(
        &self,
        state: Waypoint,
        to_vertex: usize,
        remaining_cost_estimate: P::Cost,
        motion: Option<Trajectory<Waypoint>>,
        parent: &Rc<Node<P::Cost>>,
    ) -> Rc<NodeType<P>> {

        let added_cost = match &motion {
            Some(trajectory) => self.cost_calculator.compute_cost(trajectory),
            None => P::Cost::zero()
        };
        let cost = parent.cost + added_cost;
        return Rc::new(Node{
            cost,
            remaining_cost_estimate,
            total_cost_estimate: cost + remaining_cost_estimate,
            state,
            key: Some(Key{from_vertex: parent.vertex, to_vertex, side: Side::Finish}),
            vertex: to_vertex,
            motion_from_parent: motion,
            parent: Some(parent.clone()),
            is_start: None,
        });
    }
}

impl<P: Policy> crate::Expander for Expander<P> {
    type Node = NodeType<P>;
    type Start = Start;
    type Goal = Goal;
    type Options = Options;
    /// The Solution will appear as None if no trajectory is actually needed for
    /// the goal to be reached (i.e. the agent is starting on its goal).
    type Solution = Option<crate::motion::se2::LinearTrajectory>;
    type Expansion = Expansion<P>;

    fn default_options(&self) -> Self::Options {
        return Options::default();
    }

    fn start(&self, start: &Start, goal: &Self::Goal) -> Expansion<P> {
        return Expansion::from_start(self, start, goal.vertex);
    }

    fn expand(&self, parent: &Rc<Self::Node>, goal: &Self::Goal, _: &Self::Options) -> Expansion<P> {
        return Expansion::from_node(self, parent.clone(), goal.vertex);
    }

    fn make_solution(&self, solution_node: &Rc<Self::Node>, _: &Self::Options) -> Self::Solution {
        let mut node: Rc<Self::Node> = solution_node.clone();
        let mut waypoints: Vec<Waypoint> = Vec::new();
        loop {
            if let Some(next_waypoints) = &node.motion_from_parent {
                for wp in next_waypoints.iter().rev() {
                    waypoints.push(wp.clone());
                }
            }

            match &node.parent {
                Some(parent) => { node = parent.clone(); },
                None => { break; }
            }
        }

        waypoints.reverse();
        return Trajectory::from_iter(waypoints.into_iter()).ok();
    }
}
