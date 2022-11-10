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
    graph::{Graph, Edge},
    motion::{
        Extrapolator, TimePoint,
        se2::{self, QuickestPath}, r2,
        graph_search::{Policy, BuiltinNode, Expander, StateKey},
        trajectory::{CostCalculator, DurationCostCalculator},
        reach::Reachable,
        hold::Hold,
    },
    expander::{Goal, InitTargeted, Chain, Chainable},
    node::{
        Agent, PartialKeyed, Key, Keyed,
        closed_set::{ClosedSet, PartialKeyedClosedSet, TimeVariantPartialKeyedClosetSet},
    },
    heuristic::Heuristic,
    directed::simple::SimpleGraph,
    occupancy::{
        Visibility, Cell,
        sparse_grid::SparseGrid,
        graph::{NeighborhoodGraph, VisibilityGraph},
    },
    error::NoError,
};
use std::sync::Arc;
use std::fmt::Debug;
use thiserror::Error as ThisError;

#[derive(Debug, Clone, Copy)]
pub struct OrientationGoal {
    pub target: se2::Rotation,
    pub threshold: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct GoalSE2<GraphKey: Key> {
    pub vertex: GraphKey,
    pub orientation: Option<OrientationGoal>
}

impl<GraphKey: Key, const RESOLUTION: u64> Goal<Node<GraphKey, RESOLUTION>> for GoalSE2<GraphKey> {
    fn is_satisfied(&self, node: &Node<GraphKey, RESOLUTION>) -> bool {
        if node.partial_key().map(|k| k.vertex() != self.vertex).unwrap_or(false) {
            return false;
        }

        self.orientation.map(|r| {
            let delta_yaw = (node.state().position.rotation / r.target).angle().abs();
            delta_yaw <= r.threshold
        }).unwrap_or(true)
    }
}

impl<GraphKey: Key> PartialKeyed for GoalSE2<GraphKey> {
    type Key = GraphKey;
    fn partial_key(&self) -> Option<&Self::Key> {
        Some(&self.vertex)
    }
}

impl<GraphKey: Key> Keyed for GoalSE2<GraphKey> { }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KeySE2<GraphKey: Key, const RESOLUTION: u64> {
    /// The graph vertex that the agent is currently on
    vertex: GraphKey,
    /// The key of the agent's orientation. This value should be in the range of
    /// +/- 2*PI * RESOLUTION
    orientation: i64,
}

impl<GraphKey: Key, const R: u64> KeySE2<GraphKey, R> {
    pub fn vertex(&self) -> GraphKey {
        self.vertex.clone()
    }
}

impl<const RESOLUTION: u64> From<KeySE2<usize, RESOLUTION>> for usize {
    fn from(key: KeySE2<usize, RESOLUTION>) -> Self {
        key.vertex()
    }
}

impl<const RESOLUTION: u64> From<KeySE2<Cell, RESOLUTION>> for Cell {
    fn from(cell: KeySE2<Cell, RESOLUTION>) -> Self {
        cell.vertex()
    }
}

impl<GraphKey: Key, const RESOLUTION: u64> StateKey<GraphKey, se2::timed_position::Waypoint> for KeySE2<GraphKey, RESOLUTION> {
    fn from_state(graph_key: &GraphKey, state: &se2::timed_position::Waypoint) -> Self {
        Self{
            vertex: graph_key.clone(),
            orientation: (state.position.rotation.angle() * RESOLUTION as f64) as i64,
        }
    }

    fn graph_key(&self) -> GraphKey {
        self.vertex.clone()
    }
}

pub type Node<GraphKey, const RESOLUTION: u64> = BuiltinNode<
    i64, KeySE2<GraphKey, RESOLUTION>, se2::timed_position::Waypoint
>;

pub struct ReachForLinearSE2 {
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
}

impl<GraphKey: Key, const RESOLUTION: u64> Reachable<Node<GraphKey, RESOLUTION>, GoalSE2<GraphKey>, se2::timed_position::Waypoint> for ReachForLinearSE2 {
    type ReachError = NoError;
    type Reaching<'a> = impl Iterator<Item=Result<se2::LinearTrajectory, NoError>>;

    fn reach_for<'a>(&'a self, parent: &'a Node<GraphKey, RESOLUTION>, goal: &'a GoalSE2<GraphKey>) -> Self::Reaching<'a> {
        [parent].into_iter()
        .filter(|n| {
            if let Some(key) = n.partial_key() {
                return key.vertex() == goal.vertex;
            }

            return false;
        })
        .filter_map(|n| goal.orientation.map(|g| (n, g.target)))
        .filter_map(|(n, target_orientation)| {
            let to_target = se2::Position::from_parts(
                n.state().position.translation, target_orientation
            );

            self.extrapolator.make_trajectory(
                n.state().clone(), &to_target
            ).transpose()
        })
    }
}

#[derive(Debug)]
pub struct StartSE2<GraphKey: Key> {
    pub vertex: GraphKey,
    pub orientation: se2::Rotation,
}

pub struct LinearSE2Policy<G, S, C=DurationCostCalculator, H=QuickestPath<G, C>, const RESOLUTION: u64 = 100>
where
    G: Graph<Vertex=se2::Point>,
    S: ClosedSet<Node<G::Key, RESOLUTION>>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64>
        + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2<G::Key, RESOLUTION>, GoalSE2<G::Key>, i64>,
{
    _ignore: std::marker::PhantomData<(G, S, C, H)>,
}

impl<G, S, C, H, const RESOLUTION: u64> Policy for LinearSE2Policy<G, S, C, H, RESOLUTION>
where
    G: Graph<Vertex=se2::Point>,
    S: ClosedSet<Node<G::Key, RESOLUTION>>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64>
        + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2<G::Key, RESOLUTION>, GoalSE2<G::Key>, i64>,
{
    type Waypoint = se2::timed_position::Waypoint;
    type ClosedSet = S;
    type Graph = G;
    type StateKey = KeySE2<G::Key, RESOLUTION>;
    type Node = Node<G::Key, RESOLUTION>;
    type Extrapolator = se2::timed_position::DifferentialDriveLineFollow;
    type Heuristic = H;
    type Reach = ReachForLinearSE2;
    type CostCalculator = C;
}

pub type TimeInvariantExpander<G, C, H, const RESOLUTION: u64 = 100> = Expander<LinearSE2Policy<G, PartialKeyedClosedSet<Node<<G as Graph>::Key, RESOLUTION>>, C, H>>;
pub type DirectedTimeInvariantExpander = TimeInvariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, QuickestPath<SimpleGraph<se2::Point>, DurationCostCalculator>>;
pub fn make_directed_time_invariant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> DirectedTimeInvariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(se2::QuickestPath::new(
        graph.clone(),
        cost_calculator.clone(),
        Arc::new(extrapolator.as_ref().into()),
    ));
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    DirectedTimeInvariantExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher,
    }
}

pub type FreeSpaceTimeInvariantExpander = TimeInvariantExpander<NeighborhoodGraph<SparseGrid>, DurationCostCalculator, QuickestPath<VisibilityGraph<SparseGrid>, DurationCostCalculator>>;
pub fn make_free_space_time_invariant_expander(
    visibility: Arc<Visibility<SparseGrid>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
    points_of_interest: Vec<Cell>,
) -> FreeSpaceTimeInvariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(se2::QuickestPath::new(
        Arc::new(VisibilityGraph::new(visibility.clone(), points_of_interest.iter().cloned())),
        cost_calculator.clone(),
        Arc::new(extrapolator.as_ref().into()),
    ));
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });
    let graph = Arc::new(NeighborhoodGraph::new(visibility, points_of_interest.iter().cloned()));

    FreeSpaceTimeInvariantExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher
    }
}

pub type TimeVariantExpander<G, C, H, const RESOLUTION: u64 = 100> =
    Chain<
        Expander<LinearSE2Policy<G, TimeVariantPartialKeyedClosetSet<Node<<G as Graph>::Key, RESOLUTION>>, C, H>>,
        Hold<se2::timed_position::Waypoint, C, Node<<G as Graph>::Key, RESOLUTION>>,
    >;
pub type DirectedTimeVariantExpander = TimeVariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, QuickestPath<SimpleGraph<se2::Point>, DurationCostCalculator>>;

pub fn make_directed_time_variant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> DirectedTimeVariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(QuickestPath::new(
        graph.clone(),
        cost_calculator.clone(),
        Arc::new(extrapolator.as_ref().into()),
    ));
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    Expander{
        graph, extrapolator, cost_calculator: cost_calculator.clone(), heuristic, reacher,
    }.chain(
        Hold::new(cost_calculator)
    )
}

pub type FreeSpaceTimeVariantExpander = TimeVariantExpander<NeighborhoodGraph<SparseGrid>, DurationCostCalculator, QuickestPath<VisibilityGraph<SparseGrid>, DurationCostCalculator>>;
pub fn make_free_space_time_variant_expander(
    visibility: Arc<Visibility<SparseGrid>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
    points_of_interest: Vec<Cell>,
) -> FreeSpaceTimeVariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(se2::QuickestPath::new(
        Arc::new(VisibilityGraph::new(visibility.clone(), points_of_interest.iter().cloned())),
        cost_calculator.clone(),
        Arc::new(extrapolator.as_ref().into()),
    ));
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });
    let graph = Arc::new(NeighborhoodGraph::new(visibility, points_of_interest.iter().cloned()));

    Expander{
        graph, extrapolator, cost_calculator: cost_calculator.clone(), heuristic, reacher
    }.chain(
        Hold::new(cost_calculator)
    )
}

#[derive(ThisError, Debug)]
pub enum InitErrorSE2<H> {
    #[error("An error happened during extrapolation:\n{0}")]
    Extrapolator(<se2::timed_position::DifferentialDriveLineFollow as Extrapolator<se2::timed_position::Waypoint, r2::Position>>::Error),
    #[error("An error happened while calculating the heuristic:\n{0}")]
    Heuristic(H)
}

impl<G, S, C, H, const RESOLUTION: u64> InitTargeted<StartSE2<G::Key>, GoalSE2<G::Key>> for Expander<LinearSE2Policy<G, S, C, H, RESOLUTION>>
where
    G: Graph<Vertex=r2::Position>,
    S: ClosedSet<Node<G::Key, RESOLUTION>>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64> + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2<G::Key, RESOLUTION>, GoalSE2<G::Key>, i64>,
{
    type InitTargetedError = InitErrorSE2<H::Error>;
    type InitialTargetedNodes<'a> = impl Iterator<Item=Result<Arc<Node<G::Key, RESOLUTION>>, Self::InitTargetedError>> + 'a where G: 'a, C: 'a, H: 'a, S: 'a;

    fn start<'a>(
        &'a self,
        start: &'a StartSE2<G::Key>,
        goal: &'a GoalSE2<G::Key>,
    ) -> Self::InitialTargetedNodes<'a> {
        [self.graph.vertex(start.vertex.clone())].into_iter()
        .filter_map(|x| x)
        .map(move |p0| {
            let state = se2::timed_position::Waypoint{
                time: TimePoint::zero(),
                position: se2::Position::from_parts(p0.coords.into(), start.orientation),
            };
            let key = KeySE2::from_state(&start.vertex, &state);
            let h = self.heuristic.estimate_cost(&key, goal)
                .map_err(InitErrorSE2::Heuristic)?;

            Ok(h.map(|h| self.start_from(state, Some(key), h, None)))
        })
        .filter_map(|r| r.transpose())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use se2::{Point, timed_position::DifferentialDriveLineFollow};
    use crate::{
        a_star, algorithm::Status,
        planner::make_planner,
        expander::Constrainable,
        motion::{self, collide::CircleCollisionConstraint},
    };

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

    fn make_test_extrapolation() -> DifferentialDriveLineFollow{
        return DifferentialDriveLineFollow::new(1.0f64, std::f64::consts::PI).unwrap();
    }

    #[test]
    fn test_time_invariant_expander() {
        let expander = make_directed_time_invariant_expander(
            Arc::new(make_test_graph()),
            Arc::new(make_test_extrapolation()),
        );

        let planner = make_planner(Arc::new(expander), Arc::new(a_star::Algorithm));
        let mut progress = planner.plan(
            &StartSE2{
                vertex: 0,
                orientation: se2::Rotation::new(0.0),
            },
            GoalSE2{
                vertex: 8,
                // orientation: None
                orientation: Some(OrientationGoal{
                    target: se2::Rotation::new(90f64.to_radians()),
                    threshold: motion::DEFAULT_ROTATIONAL_THRESHOLD
                })
            },
        ).unwrap();

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

    fn make_test_trajectory() -> se2::LinearTrajectory {
        se2::LinearTrajectory::from_iter([
            se2::timed_position::Waypoint::new(TimePoint::from_secs_f64(0.0), 2.0, -1.0, 0.0),
            se2::timed_position::Waypoint::new(TimePoint::from_secs_f64(3.0), 2.0, 0.0, 0.0),
        ]).ok().unwrap()
    }

    #[test]
    fn test_time_variant_expander() {
        let expander = make_directed_time_variant_expander(
            Arc::new(make_test_graph()),
            Arc::new(make_test_extrapolation())
        );

        let expander = Arc::new(
            expander.constrain(
                CircleCollisionConstraint{
                    obstacles: vec![(0.5, make_test_trajectory())],
                    agent_radius: 1.0,
                }
            )
        );

        let planner = make_planner(expander, Arc::new(a_star::Algorithm));
        let mut progress = planner.plan(
            &StartSE2{
                vertex: 0,
                orientation: se2::Rotation::new(0.0),
            },
            GoalSE2{
                vertex: 8,
                orientation: Some(OrientationGoal{
                    target: se2::Rotation::new(-90_f64.to_radians()),
                    threshold: motion::DEFAULT_ROTATIONAL_THRESHOLD
                })
            },
        ).unwrap();

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
