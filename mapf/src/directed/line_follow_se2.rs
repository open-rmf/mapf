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
use std::rc::Rc;
use std::hash::{Hash, Hasher};
use nalgebra::{UnitComplex as Rotation, Vector2};

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

pub trait Policy {
    type CostCalculator: CostCalculator<Waypoint>;
}

type CostType<P: Policy> = <P::CostCalculator as CostCalculator<Waypoint>>::Cost;
type NodeType<P: Policy> = Node<CostType<P>>;

pub struct Expander<P: Policy> {
    pub graph: Graph<nalgebra::Vector2<f64>>,
    pub extrapolation: DifferentialDriveLineFollow,
    // TODO: Add a heuristic
    pub cost_calculator: P::CostCalculator,
}

#[derive(Clone, PartialEq, Default)]
pub struct Options {
    // TODO: Come up with some options
}

pub struct Expansion<'ex, P: Policy> {
    expander: &'ex Expander<P::CostCalculator>,
    from_node: Rc<NodeType<P>>,
    goal_vertex: usize,
    expansion_index: usize,
    expanded_start: bool
}

impl<'ex, P: Policy> Expansion<'ex, P> {
    fn new(expander: &'ex Expander<P>, from_node: &Rc<NodeType<P>>, toward_goal: usize) -> Self {
        return Self{expander, from_node, goal_vertex: toward_goal, expansion_index: 0, expanded_start: false};
    }
}

impl <'ex, P: Policy> Expansion<'ex, P>
{
    fn expand_to(&self, to_vertex: usize) -> Option<Rc<NodeType<P>>> {
        if let Some(vertex) = self.expander.graph.vertices.get(to_vertex) {
            let extrapolation = self.expander.extrapolation.extrapolate(
                self.from_node.state,
                vertex
            );
            if let Ok(mut waypoints) = extrapolation {
                let state = waypoints.last().clone();
                waypoints.insert(self.from_node.state);
                let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
                let remaining_cost_estimate = self.expander.heuristic(to_vertex, self.goal_vertex)?;
                return Some(self.make_node(state, vertex, remaining_cost_estimate, trajectory));
            }
            // else: We should propogate these inner expansion errors somehow so
            // users can be aware of unexpected problems.
        }

        return None;
    }

    fn expand_start(&self, to_vertex: usize) -> Option<Rc<NodeType<P>>> {
        let extrapolation = self.expander.extrapolation.extrapolate(
            self.from_node.state,
            to_vertex
        );
        if let Ok(mut waypoints) = extrapolation {
            let state = waypoints.last().clone();
            waypoints.insert(self.from_node.state);
            let trajectory = Trajectory::from_iter(waypoints.into_iter()).ok();
            let remaining_cost_estimate = self.expander.heuristic(to_vertex, self.goal_vertex)?;
            return Some(self.make_node(state, to_vertex, remaining_cost_estimate, trajectory));
        }

        return None;
    }

    fn make_node(
        &self,
        state: Waypoint,
        to_vertex: usize,
        remaining_cost_estimate: CostType<P>,
        motion: Option<Trajectory<Waypoint>>
    ) -> Rc<NodeType<P>> {

        let added_cost = match motion {
            Some(trajectory) => self.expander.cost_calculator.compute_cost(&trajectory),
            None => CostType::<P>::zero()
        };
        let cost = self.from_node.cost + added_cost;
        return Rc::new(Node{
            cost,
            remaining_cost_estimate,
            total_cost_estimate: cost + remaining_cost_estimate,
            state,
            key: Some(Key{from_vertex: self.from_node.vertex, to_vertex, side: Side::Finish}),
            vertex: to_vertex,
            motion_from_parent: motion,
            parent: Some(self.from_node.clone()),
            is_start: None,
        });
    }
}

impl<'ex, CostCalc> Iterator for Expansion<'ex, CostCalc>
where
    CostCalc: CostCalculator<Waypoint>
{
    type Item=Rc<Node<CostCalc::Cost>>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(start) = &self.from_node.is_start {
            if let Some(initial_p) = &start.offset_location {
                // return
                if self.expanded_start {
                    return None;
                }

                self.expanded_start = true;
                return self.expand_start(self.from_node.vertex);
            }
        }

        if let Some(to_vertices) = self.expander.graph.edges.get(self.from_node.vertex) {
            if let Some(to_vertex) = to_vertices.get(self.expansion_index) {
                let next = Some(self.expand_to(to_vertex));
                self.expansion_index += 1;
                return next;
            }
        }

        return None;
    }
}

impl<P: Policy> Expander<P> {
    fn heuristic(&self, from_vertex: usize, to_goal: usize) -> Option<CostType<P>> {
        return None;
    }
}

impl<'ex, P: Policy> crate::Expander<'ex> for Expander<P> {
    type Node = NodeType<P>;
    type Start = Start;
    type Goal = Goal;
    type Options = Options;
    type Solution = crate::motion::se2::LinearTrajectory;
    type Expansion = Expansion<'ex, P>;

    fn default_options(&self) -> Self::Options {
        return Options::default();
    }
}
