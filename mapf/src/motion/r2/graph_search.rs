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
    node::{Key, Cost, ClosedSet, PartialKeyed, PartialKeyedClosedSet},
    expander::{Goal, InitAimless, InitTargeted},
    motion::{
        TimePoint,
        graph_search::{BuiltinNode, Policy, Expander},
        trajectory::{CostCalculator, DurationCostCalculator},
        r2::{self, direct_travel::DirectTravelHeuristic, timed_position::LineFollow},
        reach::NoReach,
    },
    heuristic::Heuristic,
    graph::{Graph, KeyOf as KeyOfGraph},
    directed::simple::SimpleGraph,
};
use std::sync::Arc;
use thiserror::Error as ThisError;
use num::Zero;

pub type Node<C, K> = BuiltinNode<C, K, r2::timed_position::Waypoint>;

impl<C: Cost, K: Key> Goal<Node<C, K>> for K {
    fn is_satisfied(&self, node: &Node<C, K>) -> bool {
        if let Some(key) = node.partial_key() {
            return key == self;
        }

        return false;
    }
}

pub struct LinearR2Policy<G, S, C, H> {
    _ignore: std::marker::PhantomData<(G, S, C, H)>
}

impl<G, S, C, H> Policy for LinearR2Policy<G, S, C, H>
where
    G: Graph<Vertex=r2::Position>,
    S: ClosedSet<Node<C::Cost, KeyOfGraph<G>>>,
    C: CostCalculator<r2::timed_position::Waypoint>,
    H: Heuristic<KeyOfGraph<G>, KeyOfGraph<G>, C::Cost>,
{
    type Waypoint = r2::timed_position::Waypoint;
    type ClosedSet = S;
    type Graph = G;
    type StateKey = KeyOfGraph<G>;
    type Node = Node<C::Cost, Self::StateKey>;
    type Extrapolator = r2::timed_position::LineFollow;
    type Heuristic = H;
    type Reach = NoReach;
    type CostCalculator = C;
}

pub type TimeInvariantExpander<G, C, H> = Expander<LinearR2Policy<G, PartialKeyedClosedSet<Node<<C as CostCalculator<r2::timed_position::Waypoint>>::Cost, KeyOfGraph<G>>>, C, H>>;
pub type DefaultExpander = TimeInvariantExpander<SimpleGraph<r2::Position>, DurationCostCalculator, DirectTravelHeuristic<SimpleGraph<r2::Position>, DurationCostCalculator>>;
pub fn make_default_expander(
    graph: Arc<SimpleGraph<r2::Position>>,
    extrapolator: Arc<LineFollow>,
) -> DefaultExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(DirectTravelHeuristic{
        graph: graph.clone(),
        cost_calculator: cost_calculator.clone(),
        extrapolator: (*extrapolator).clone(),
    });

    DefaultExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher: Arc::new(NoReach)
    }
}

pub type DefaultNode = Node<i64, usize>;

#[derive(ThisError, Debug)]
pub enum InitErrorR2<K, H> {
    #[error("The requested start vertex [{0:?}] is missing")]
    MissingStartVertex(K),
    #[error("The request goal vertex [{0:?}] is missing")]
    MissingGoalVertex(K),
    #[error("An error occurred while calculating the heuristic:\n{0}")]
    Heuristic(H),
}

impl<G, S, C, H> InitAimless<G::Key> for Expander<LinearR2Policy<G, S, C, H>>
where
    G: Graph<Vertex=r2::Position>,
    S: ClosedSet<Node<C::Cost, G::Key>>,
    C: CostCalculator<r2::timed_position::Waypoint>,
    H: Heuristic<G::Key, G::Key, C::Cost>,
{
    type InitAimlessError = InitErrorR2<G::Key, H::Error>;
    type InitialAimlessNodes<'a> = impl Iterator<Item=Result<Arc<Self::Node>, Self::InitAimlessError>> + 'a where G: 'a, C: 'a, H: 'a, S: 'a;

    fn aimless_start<'a>(
        &'a self,
        start: &'a G::Key,
    ) -> Self::InitialAimlessNodes<'a> {
        [self.graph.vertex(start.clone())].into_iter()
        .map(|v| {
            v.map_or(
                Err(InitErrorR2::MissingStartVertex(start.clone())),
                |p| Ok(p),
            )
        })
        .map(|r| {
            r.map(|start_p| {
                let h = C::Cost::zero();
                let state = r2::timed_position::Waypoint{
                    time: TimePoint::zero(),
                    position: start_p.clone(),
                };

                Some(self.start_from(state, Some(start.clone()), h, None))
            })
        })
        .filter_map(|r| r.transpose())
    }
}

impl<G, S, C, H> InitTargeted<G::Key, G::Key> for Expander<LinearR2Policy<G, S, C, H>>
where
    G: Graph<Vertex=r2::Position>,
    S: ClosedSet<Node<C::Cost, G::Key>>,
    C: CostCalculator<r2::timed_position::Waypoint>,
    H: Heuristic<G::Key, G::Key, C::Cost>,
{
    type InitTargetedError = InitErrorR2<G::Key, H::Error>;
    type InitialTargetedNodes<'a> = impl Iterator<Item=Result<Arc<Self::Node>, Self::InitTargetedError>> + 'a where G: 'a, C: 'a, H: 'a, S: 'a ;

    fn start<'a>(
        &'a self,
        start: &'a G::Key,
        goal: &'a G::Key,
    ) -> Self::InitialTargetedNodes<'a> {
        [self.graph.vertex(start.clone())].into_iter()
        .map(|v| {
            v.map_or(
                Err(InitErrorR2::MissingStartVertex(start.clone())),
                |p| Ok(p),
            )
        })
        .map(|r| {
            r.and_then(|start_p| {
                // Check if the goal is valid before we begin expanding. It may
                // be good to do this for other initializables as well.
                self.graph.vertex(goal.clone())
                    .map_or(Err(InitErrorR2::MissingGoalVertex(goal.clone())),
                    |p| Ok(p)
                )?;

                let h = self.heuristic.estimate_cost(
                    start, goal
                ).map_err(InitErrorR2::Heuristic)?;

                let state = r2::timed_position::Waypoint{
                    time: TimePoint::zero(),
                    position: start_p.clone(),
                };

                Ok(h.map(|h| self.start_from(state, Some(start.clone()), h, None)))
            })
        })
        .filter_map(|r| r.transpose())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        a_star, algorithm::Status,
        planner::make_planner,
        motion::{
            r2::{Position, timed_position::LineFollow}
        },
    };

    fn make_test_graph() -> SimpleGraph<Position> {
        /*
         * 0-----1-----2-----3
         *           /       |
         *         /         |
         *       4-----5     6
         *             |
         *             |
         *             7-----8
        */

        let mut vertices = Vec::<Position>::new();
        vertices.push(Position::new(0.0, 0.0)); // 0
        vertices.push(Position::new(1.0, 0.0)); // 1
        vertices.push(Position::new(2.0, 0.0)); // 2
        vertices.push(Position::new(3.0, 0.0)); // 3
        vertices.push(Position::new(1.0, -1.0)); // 4
        vertices.push(Position::new(2.0, -1.0)); // 5
        vertices.push(Position::new(3.0, -1.0)); // 6
        vertices.push(Position::new(2.0, -2.0)); // 7
        vertices.push(Position::new(3.0, -2.0)); // 8

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

    fn make_test_extrapolation() -> LineFollow{
        return LineFollow::new(1.0f64).unwrap();
    }

    #[test]
    fn test_time_invariant_expander() {
        let expander = make_default_expander(
            Arc::new(make_test_graph()),
            Arc::new(make_test_extrapolation()),
        );

        let planner = make_planner(Arc::new(expander), Arc::new(a_star::Algorithm));
        let mut progress = planner.plan(&0, 8).unwrap();

        match progress.solve().unwrap() {
            Status::Solved(solution) => {
                let motion = solution.motion().as_ref().unwrap();
                println!("{:#?}", motion);
                println!("Finish time: {:?}", motion.finish_time().as_secs_f32());
            },
            Status::Impossible => {
                assert!(false);
            },
            Status::Incomplete => {
                assert!(false);
            }
        }
    }
}
