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
    Heuristic,
    graph::{Graph, Edge, VertexOf, KeyOf},
    expander::{Expander as ExpanderTrait, Goal, Expandable, Closable, Solvable},
    node::{self, Agent, Cost, ClosedSet, Weighted, Timed, PartialKeyed, Keyed, Informed, PathSearch},
    motion::{
        Waypoint, Trajectory, Extrapolator,
        trajectory::CostCalculator,
        reach::Reachable,
        movable::{Movable, ArcMovable, StartingPoint},
    }
};
use std::{
    fmt::Debug,
    sync::Arc,
};
use num::Zero;

pub trait Key<TargetKey: node::Key, TargetState>: node::Key + Into<TargetKey> {
    fn make_child(&self, target_key: &TargetKey, target_state: &TargetState) -> Self;
}

pub trait Policy: Sized {
    type Waypoint: Waypoint;
    type ClosedSet: ClosedSet<Self::Node>;
    type Graph: Graph;
    type Key: Key<KeyOf<Self::Graph>, Self::Waypoint>;
    type Node: Informed + Movable<Self::Waypoint, Key=Self::Key>;
    type Extrapolator: Extrapolator<Self::Waypoint, VertexOf<Self::Graph>>;
    type Heuristic;
    type Reach;
    type CostCalculator: CostCalculator<Self::Waypoint, Cost=NodeCostOf<Self>>;
}

pub type WaypointOf<P> = <P as Policy>::Waypoint;
pub type NodeOf<P> = <P as Policy>::Node;
pub type NodeKeyOf<P> = <NodeOf<P> as PartialKeyed>::Key;
pub type NodeCostOf<P> = <NodeOf<P> as Weighted>::Cost;
pub type GraphOf<P> = <P as Policy>::Graph;
pub type GraphKeyOf<P> = <GraphOf<P> as Graph>::Key;
pub type ExtrapolatorOf<P> = <P as Policy>::Extrapolator;
pub type ExtrapolatorErrorOf<P> = <ExtrapolatorOf<P> as Extrapolator<WaypointOf<P>, VertexOf<GraphOf<P>>>>::Error;
pub type HeuristicOf<P> = <P as Policy>::Heuristic;
pub type HeuristicErrorOf<P, G> = <HeuristicOf<P> as Heuristic<NodeKeyOf<P>, G, NodeCostOf<P>>>::Error;
pub type ReachOf<P> = <P as Policy>::Reach;
pub type ReachErrorOf<P, G> = <ReachOf<P> as Reachable<NodeOf<P>, G, WaypointOf<P>>>::ReachError;

#[derive(Debug)]
pub struct BuiltinNode<C, K, W: Waypoint> {
    cost: C,
    remaining_cost_estimate: C,
    total_cost_estimate: C,
    state: W,
    key: Option<K>,
    motion_from_parent: Option<Trajectory<W>>,
    parent: Option<Arc<Self>>,
}

impl<C: Cost, K, W: Waypoint> Weighted for BuiltinNode<C, K, W> {
    type Cost = C;
    fn cost(&self) -> Self::Cost {
        self.cost
    }
}

impl<C: Cost, K, W: Waypoint> Informed for BuiltinNode<C, K, W> {
    fn remaining_cost_estimate(&self) -> Self::Cost {
        self.remaining_cost_estimate
    }

    fn total_cost_estimate(&self) -> Self::Cost {
        self.total_cost_estimate
    }
}

impl<C: Cost, K, W: Waypoint> Timed for BuiltinNode<C, K, W> {
    fn time(&self) -> &time_point::TimePoint {
        self.state.time()
    }
}

impl<C, K, W: Waypoint> Agent<W, Trajectory<W>> for BuiltinNode<C, K, W> {
    fn state(&self) -> &W {
        &self.state
    }

    fn action(&self) -> &Option<Trajectory<W>> {
        &self.motion_from_parent
    }
}

impl<C, K, W: Waypoint> PathSearch for BuiltinNode<C, K, W> {
    fn parent(&self) -> &Option<std::sync::Arc<Self>> {
        &self.parent
    }
}

impl<C, K: node::Key, W: Waypoint> PartialKeyed for BuiltinNode<C, K, W> {
    type Key = K;
    fn partial_key(&self) -> Option<&Self::Key> {
        self.key.as_ref()
    }
}

impl<C: Cost, K: node::Key, W: Waypoint> Movable<W> for BuiltinNode<C, K, W> {
    fn move_from(
        parent: Arc<Self>,
        key: Option<K>,
        cost_from_parent: C,
        remaining_cost_estimate: C,
        motion_from_parent: Option<Trajectory<W>>,
    ) -> Self {
        let cost = parent.cost + cost_from_parent;
        let state = motion_from_parent.as_ref().map(
            |t| t.finish().clone()
        ).unwrap_or(parent.state().clone());

        BuiltinNode{
            cost,
            remaining_cost_estimate,
            total_cost_estimate: cost + remaining_cost_estimate,
            state,
            key,
            motion_from_parent,
            parent: Some(parent)
        }
    }
}

impl<C: Cost, K: node::Key, W: Waypoint> StartingPoint<C, K, W> for BuiltinNode<C, K, W> {
    fn start_from(
        state: W,
        key: Option<K>,
        initial_cost: C,
        remaining_cost_estimate: C,
        initial_motion: Option<Trajectory<W>>,
    ) -> Self {
        BuiltinNode{
            cost: initial_cost,
            remaining_cost_estimate,
            total_cost_estimate: initial_cost + remaining_cost_estimate,
            state,
            key,
            motion_from_parent: initial_motion,
            parent: None
        }
    }
}

#[derive(Debug, Clone)]
pub struct MakeBuiltinNode<W, N> {
    _ignore: std::marker::PhantomData<(W, N)>,
}

impl<W, N> Default for MakeBuiltinNode<W, N> {
    fn default() -> Self {
        Self{_ignore: Default::default()}
    }
}

pub struct Expander<P: Policy> where NodeKeyOf<P>: Key<GraphKeyOf<P>, P::Waypoint> {
    pub graph: Arc<P::Graph>,
    pub extrapolator: Arc<P::Extrapolator>,
    pub cost_calculator: Arc<P::CostCalculator>,
    pub heuristic: Arc<P::Heuristic>,
    pub reacher: Arc<P::Reach>,
}

impl<P: Policy> Expander<P>
where
    NodeKeyOf<P>: Key<GraphKeyOf<P>, P::Waypoint>,
{
    pub fn make_child_node(
        &self,
        key: Option<NodeKeyOf<P>>,
        remaining_cost_estimate: NodeCostOf<P>,
        motion_from_parent: Option<Trajectory<P::Waypoint>>,
        parent: Arc<P::Node>,
    ) -> Arc<P::Node> {
        let cost_from_parent = motion_from_parent.as_ref().map(
            |t| self.cost_calculator.compute_cost(t)
        ).unwrap_or(NodeCostOf::<P>::zero());

        parent.moved_with(
            key, cost_from_parent, remaining_cost_estimate, motion_from_parent
        )
    }

    pub fn start_from(
        &self,
        state: WaypointOf<P>,
        key: Option<NodeKeyOf<P>>,
        remaining_cost_estimate: NodeCostOf<P>,
        initial_motion: Option<Trajectory<P::Waypoint>>,
    ) -> Arc<P::Node>
    where
        P::Node: StartingPoint<NodeCostOf<P>, P::Key, P::Waypoint>,
    {
        let initial_cost = initial_motion.as_ref().map(
            |t| self.cost_calculator.compute_cost(t)
        ).unwrap_or(NodeCostOf::<P>::zero());

        Arc::new(<P::Node as StartingPoint<NodeCostOf<P>, P::Key, P::Waypoint>>::start_from(
            state, key, initial_cost, remaining_cost_estimate, initial_motion
        ))
    }
}

impl<P: Policy> ExpanderTrait for Expander<P> {
    type Node = P::Node;
}

pub enum ExpansionError<P: Policy, G>
where
    P::Heuristic: Heuristic<NodeKeyOf<P>, G, NodeCostOf<P>>,
    P::Reach: Reachable<P::Node, G, P::Waypoint>,
{
    Extrapolator(ExtrapolatorErrorOf<P>),
    Heuristic(HeuristicErrorOf<P, G>),
    Reach(ReachErrorOf<P, G>),
}

impl<P: Policy, G> Debug for ExpansionError<P, G>
where
    P::Heuristic: Heuristic<NodeKeyOf<P>, G, NodeCostOf<P>>,
    P::Reach: Reachable<P::Node, G, P::Waypoint>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExpansionError::Extrapolator(e) => {
                f.debug_tuple("graph_search::ExpansionError::Extrapolator").field(e).finish()
            },
            ExpansionError::Heuristic(e) => {
                f.debug_tuple("graph_search::ExpansionError::Heuristic").field(e).finish()
            },
            ExpansionError::Reach(e) => {
                f.debug_tuple("graph_search::ExpansionError::Reach").field(e).finish()
            }
        }
    }
}

impl<P: Policy> Closable for Expander<P> {
    type ClosedSet = P::ClosedSet;
}

impl<P: Policy, G> Expandable<G> for Expander<P>
where
    G: Keyed<Key=GraphKeyOf<P>> + Goal<P::Node>,
    P::Heuristic: Heuristic<NodeKeyOf<P>, G, NodeCostOf<P>>,
    P::Reach: Reachable<P::Node, G, P::Waypoint>,
{
    type ExpansionError = ExpansionError<P, G>;
    type Expansion<'a> where P: 'a, G: 'a = impl Iterator<Item=Result<Arc<P::Node>, ExpansionError<P, G>>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a std::sync::Arc<P::Node>,
        goal: &'a G,
    ) -> Self::Expansion<'a> {
        [parent.partial_key()].into_iter()
            .filter_map(|x| x)
            .flat_map(move |parent_key| {
                self.graph.edges_from_vertex((*parent_key).clone().into()).into_iter()
                    .filter_map(|edge| -> Option<(&KeyOf<P::Graph>, &VertexOf<P::Graph>)> {
                        let key = edge.endpoint_key();
                        self.graph.vertex((*key).clone()).map(|target| (key, target))
                    })
                    .map(move |(to_key, to_target)| {
                        let trajectory = self.extrapolator.make_trajectory(
                            parent.state().clone(),
                            to_target
                        ).map_err(ExpansionError::Extrapolator)?;
                        Ok((parent_key, to_key, trajectory))
                    })
                    .map(move |r| {
                        r.and_then(|(parent_key, to_key, trajectory)| {
                            let state = trajectory.as_ref().map(|t| t.finish()).unwrap_or(&parent.state());
                            let to_key = parent_key.make_child(&to_key, state);
                            let h = self.heuristic.estimate_cost(
                                &to_key, goal
                            ).map_err(ExpansionError::Heuristic)?;

                            Ok(h.map(|h| self.make_child_node(
                                Some(to_key), h, trajectory, parent.clone())
                            ))
                        })
                    })
                    .filter_map(|r| r.transpose())
            })
            .chain(
                [parent.partial_key()].into_iter()
                .filter_map(|x| x)
                .flat_map(|parent_key| {
                    self.reacher.reach_for(parent.as_ref(), goal).into_iter()
                    .map(|r| {
                        r.and_then(|trajectory| {
                            // We assume the goal is reached, because otherwise
                            // the Reachable trait was implemented incorrectly.
                            let to_key = parent_key.make_child(goal.key(), trajectory.finish());
                            Ok(self.make_child_node(
                                Some(to_key), NodeCostOf::<P>::zero(), Some(trajectory), parent.clone()
                            ))
                        }).map_err(ExpansionError::Reach)
                    })
                })
            )
    }
}

pub struct Solution<P: Policy> {
    cost: NodeCostOf<P>,
    motion: Option<Trajectory<P::Waypoint>>,
}

impl<P: Policy> Debug for Solution<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("motion::graph_search::Solution")
        .field("cost", &self.cost)
        .field("motion", &self.motion)
        .finish()
    }
}

impl<P: Policy> Solution<P> {
    pub fn cost(&self) -> &NodeCostOf<P> {
        &self.cost
    }

    pub fn motion(&self) -> &Option<Trajectory<P::Waypoint>> {
        &self.motion
    }
}

impl<P: Policy> node::Weighted for Solution<P> {
    type Cost = NodeCostOf<P>;
    fn cost(&self) -> Self::Cost {
        self.cost
    }
}

pub struct ReconstructMotion<N, W> {
    node: Option<Arc<N>>,
    index: usize,
    shift: time_point::Duration,
    _ignore: std::marker::PhantomData<W>,
}

impl<W: Waypoint, N: Agent<W, Trajectory<W>>> ReconstructMotion<N, W> {
    pub fn new(root: Arc<N>) -> Self {
        Self{node: Some(root), index: 0, shift: time_point::Duration::zero(), _ignore: Default::default()}
    }

    pub fn shifted(mut self, shift: time_point::Duration) -> Self {
        self.shift += shift;
        self
    }
}

impl<W: Waypoint, N: Agent<W, Trajectory<W>>> Iterator for ReconstructMotion<N, W> {
    type Item = W;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(node) = &self.node {
                if let Some(motion) = &node.action() {
                    if self.index < motion.len() {
                        self.index += 1;
                        let mut wp = motion[self.index - 1].0.clone();
                        wp.set_time(*wp.time() + self.shift);
                        return Some(wp);
                    }
                }

                self.node = node.parent().clone();
                self.index = 0;
                continue;
            }

            return None;
        }
    }
}

impl<P: Policy> Solvable for Expander<P> {
    type Solution = Solution<P>;
    type SolveError = ();

    fn make_solution(&self, solution_node: &Arc<P::Node>) -> Result<Self::Solution, Self::SolveError> {
        let motion = Trajectory::from_iter(ReconstructMotion::new(solution_node.clone())).ok();
        let cost = motion.as_ref().map(|t| self.cost_calculator.compute_cost(t)).unwrap_or(NodeCostOf::<P>::zero());
        Ok(Solution{cost, motion})
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::node::PartialKeyedClosedSet;
    use crate::motion::{se2, trajectory::DurationCostCalculator, reach::NoReach};
    use crate::expander::Goal;
    use crate::directed::simple::SimpleGraph as SimpleGraph;
    use std::fmt::Debug;

    #[derive(Debug)]
    struct GoalSE2 {
        vertex: usize,
    }

    type NodeSE2 = BuiltinNode<i64, StateKey, se2::timed_position::Waypoint>;

    impl Goal<NodeSE2> for GoalSE2 {
        fn is_satisfied(&self, node: &NodeSE2) -> bool {
            if let Some(key) = &node.key {
                let key: usize = key.clone().into();
                return key == self.vertex;
            }

            return false;
        }
    }

    impl PartialKeyed for GoalSE2 {
        type Key = usize;
        fn partial_key(&self) -> Option<&Self::Key> {
            Some(&self.vertex)
        }
    }

    impl Keyed for GoalSE2 { }

    #[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
    pub enum Side {
        Beginning,
        Finish
    }

    #[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
    struct StateKey {
        from_vertex: usize,
        to_vertex: usize,
        side: Side,
    }

    impl Into<usize> for StateKey {
        fn into(self) -> usize {
            if self.side == Side::Beginning {
                self.from_vertex
            } else {
                self.to_vertex
            }
        }
    }

    impl Key<usize, se2::timed_position::Waypoint> for StateKey {
        fn make_child(&self, target_key: &usize, _: &se2::timed_position::Waypoint) -> Self {
            StateKey{
                from_vertex: self.to_vertex,
                to_vertex: *target_key,
                side: Side::Finish,
            }
        }
    }

    #[derive(Debug)]
    struct BadHeuristic;
    impl<S: Debug, G: Debug, C: Cost> Heuristic<S, G, C> for BadHeuristic {
        type Error = ();
        fn estimate_cost(&self, _: &S, _: &G) -> Result<Option<C>, Self::Error> {
            Ok(Some(C::zero()))
        }
    }

    #[derive(Debug)]
    struct PolicySE2;
    impl Policy for PolicySE2 {
        type Waypoint = se2::timed_position::Waypoint;
        type ClosedSet = PartialKeyedClosedSet<NodeSE2>;
        type Key = StateKey;
        type Node = NodeSE2;
        type Graph = SimpleGraph<se2::Point>;
        type Extrapolator = se2::timed_position::DifferentialDriveLineFollow;
        type Heuristic = BadHeuristic;
        type Reach = NoReach;
        type CostCalculator = DurationCostCalculator;
    }

    #[test]
    fn make_test_expander() {

    }
}
