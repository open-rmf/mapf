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
    expander::{Goal, Expandable, Closable},
    node::{self, Agent, Cost, ClosedSet, Weighted, PartialKeyed, Informed, PathSearch},
    motion::{
        Waypoint, Trajectory, Extrapolator,
        trajectory::CostCalculator
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

pub trait Policy: Sized + Debug {
    type Waypoint: Waypoint;
    type Goal: Goal<Self::Node>;
    type ClosedSet: ClosedSet<Self::Node>;
    type Graph: Graph;
    type Key: Key<KeyOf<Self::Graph>, Self::Waypoint>;
    type Node: Informed + PartialKeyed<Key=Self::Key> + Agent<State=Self::Waypoint>;
    type Extrapolator: Extrapolator<Self::Waypoint, VertexOf<Self::Graph>>;
    type Heuristic: Heuristic<State=<Self::Node as PartialKeyed>::Key, Goal=Self::Goal, Cost=NodeCostOf<Self>>;
    type CostCalculator: CostCalculator<Self::Waypoint, Cost=NodeCostOf<Self>>;
    type MakeChildNode: MakeChildNode<Self::Waypoint, Self::Node>;
}

pub type NodeKeyOf<P> = <<P as Policy>::Node as PartialKeyed>::Key;
pub type NodeCostOf<P> = <<P as Policy>::Node as Weighted>::Cost;

pub trait MakeChildNode<W: Waypoint, N: Informed + PartialKeyed> {
    fn make_child_node(
        &self,
        waypoint: W,
        key: N::Key,
        cost_from_parent: N::Cost,
        remaining_cost_estimate: N::Cost,
        motion_from_parent: Option<Trajectory<W>>,
        parent: Arc<N>,
    ) -> Arc<N>;
}

pub struct BuiltinNode<C, K, W: Waypoint> {
    cost: C,
    remaining_cost_estimate: C,
    total_cost_estimate: C,
    state: W,
    key: K,
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

impl<C, K, W: Waypoint> Agent for BuiltinNode<C, K, W> {
    type State = W;
    type Action = Trajectory<W>;

    fn state(&self) -> &Self::State {
        &self.state
    }

    fn action(&self) -> &Option<Self::Action> {
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
    fn key(&self) -> Option<&Self::Key> {
        Some(&self.key)
    }
}

pub struct MakeChildBuiltinNode<W, N> {
    _waypoint_type: std::marker::PhantomData<W>,
    _node_type: std::marker::PhantomData<N>,
}

impl<C: Cost, K: node::Key, W: Waypoint> MakeChildNode<W, BuiltinNode<C, K, W>> for MakeChildBuiltinNode<W, BuiltinNode<C, K, W>> {
    fn make_child_node(
        &self,
        waypoint: W,
        key: K,
        cost_from_parent: C,
        remaining_cost_estimate:C,
        motion: Option<Trajectory<W>>,
        parent: Arc<BuiltinNode<C, K, W>>,
    ) -> Arc<BuiltinNode<C, K, W>> {
        let cost = parent.cost() + cost_from_parent;
        Arc::new(BuiltinNode{
            cost,
            remaining_cost_estimate,
            total_cost_estimate: cost + remaining_cost_estimate,
            state: waypoint,
            key,
            motion_from_parent: motion,
            parent: Some(parent),
        })
    }
}

pub struct Expander<P: Policy> where <<P as Policy>::Node as PartialKeyed>::Key: Key<<<P as Policy>::Graph as Graph>::Key, <P as Policy>::Waypoint> {
    graph: Arc<P::Graph>,
    extrapolator: Arc<P::Extrapolator>,
    cost_calculator: Arc<P::CostCalculator>,
    heuristic: Arc<P::Heuristic>,
    node_spawner: P::MakeChildNode,
}

impl<P: Policy> Expander<P> {
    fn make_node(
        &self,
        state: P::Waypoint,
        key: NodeKeyOf<P>,
        remaining_cost_estimate: NodeCostOf<P>,
        motion_from_parent: Option<Trajectory<P::Waypoint>>,
        parent: Arc<P::Node>,
    ) -> Arc<P::Node> {
        let cost_from_parent = motion_from_parent.as_ref().map(
            |t| self.cost_calculator.compute_cost(t)
        ).unwrap_or(NodeCostOf::<P>::zero());

        self.node_spawner.make_child_node(
            state, key, cost_from_parent, remaining_cost_estimate, motion_from_parent, parent
        )
    }
}

pub type ExtrapolatorErrorOf<P> = <<P as Policy>::Extrapolator as Extrapolator<<P as Policy>::Waypoint, VertexOf<<P as Policy>::Graph>>>::Error;
pub type HeuristicErrorOf<P> = <<P as Policy>::Heuristic as Heuristic>::Error;

#[derive(Debug)]
pub enum ExpansionError<P: Policy> {
    Extrapolator(ExtrapolatorErrorOf<P>),
    Heuristic(HeuristicErrorOf<P>),
}

impl<P: Policy> Closable<P::Node> for Expander<P> {
    type ClosedSet = P::ClosedSet;
}

impl<P: Policy> Expandable<P::Node, P::Goal> for Expander<P> {
    type ExpansionError = ExpansionError<P>;
    type Expansion<'a> where P: 'a = impl Iterator<Item=Result<Arc<P::Node>, ExpansionError<P>>> + 'a;

    fn expand<'a>(
        &'a self,
        parent: &'a std::sync::Arc<P::Node>,
        goal: Option<&'a P::Goal>,
    ) -> Self::Expansion<'a> {
        [parent.key()].into_iter()
            .filter_map(|x| x)
            .flat_map(move |parent_key| {
                self.graph.edges_from_vertex((*parent_key).clone().into()).into_iter()
                    .filter_map(|edge| -> Option<(&KeyOf<P::Graph>, &VertexOf<P::Graph>)> {
                        let key = edge.endpoint_key();
                        self.graph.vertex((*key).clone()).map(|target| (key, target))
                    })
                    .map(move |(to_key, to_target)| {
                        let waypoints = self.extrapolator.extrapolate(
                            parent.state(),
                            to_target
                        ).map_err(ExpansionError::Extrapolator)?;

                        let trajectory = Trajectory::from_iter(
                            [parent.state().clone()].into_iter().chain(waypoints.into_iter())
                        ).ok();
                        Ok((parent_key, to_key, trajectory))
                    })
                    .map(move |r| {
                        r.and_then(|(parent_key, to_key, trajectory)| {
                            let state = trajectory.as_ref().map(|t| t.finish()).unwrap_or(&parent.state()).clone();
                            let to_key = parent_key.make_child(&to_key, &state);
                            let h = self.heuristic.estimate_cost(
                                &to_key, goal
                            ).map_err(ExpansionError::Heuristic)?;

                            Ok(h.map(|h| self.make_node(state, to_key, h, trajectory, parent.clone())))
                        })
                    })
                    .filter_map(|r| r.transpose())
            })
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::node::PartialKeyedClosedSet;
    use crate::motion::{se2, trajectory::DurationCostCalculator};
    use crate::expander::Goal;
    use crate::directed::simple::Graph as SimpleGraph;

    struct GoalSE2 {
        vertex: usize,
    }

    type NodeSE2 = BuiltinNode<i64, StateKey, se2::timed_position::Waypoint>;

    impl Goal<NodeSE2> for GoalSE2 {
        fn is_satisfied(&self, node: &NodeSE2) -> bool {
            let key: usize = node.key.clone().into();
            return key == self.vertex;
        }
    }

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

    struct BadHeuristic<S, G, C> {
        _s: std::marker::PhantomData<S>,
        _g: std::marker::PhantomData<G>,
        _c: std::marker::PhantomData<C>,
    }


    impl<S, G, C: Cost> Heuristic for BadHeuristic<S, G, C> {
        type Error = ();
        type State = S;
        type Goal = G;
        type Cost = C;
        fn estimate_cost(
            &self,
            from_state: &Self::State,
            to_goal: Option<&Self::Goal>
        ) -> Result<Option<Self::Cost>, Self::Error> {
            Ok(Some(Self::Cost::zero()))
        }
    }

    #[derive(Debug)]
    struct PolicySE2;
    impl Policy for PolicySE2 {
        type Waypoint = se2::timed_position::Waypoint;
        type Goal = GoalSE2;
        type ClosedSet = PartialKeyedClosedSet<NodeSE2>;
        type Key = StateKey;
        type Node = NodeSE2;
        type Graph = SimpleGraph<se2::Point>;
        type Extrapolator = se2::timed_position::DifferentialDriveLineFollow;
        type Heuristic = BadHeuristic<StateKey, GoalSE2, i64>;
        type CostCalculator = DurationCostCalculator;
        type MakeChildNode = MakeChildBuiltinNode<Self::Waypoint, Self::Node>;
    }

    #[test]
    fn make_test_expander() {

    }
}
