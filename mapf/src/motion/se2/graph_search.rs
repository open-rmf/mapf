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
        se2, r2::{self, direct_travel::DirectTravelHeuristic},
        graph_search::{Policy, BuiltinNode, Expander, StateKey},
        trajectory::{CostCalculator, DurationCostCalculator},
        reach::Reachable,
        hold::Hold,
    },
    expander::{Goal, InitTargeted, Chain, Chainable},
    node::{
        Agent, PartialKeyed, Keyed,
        closed_set::{ClosedSet, PartialKeyedClosedSet, TimeVariantPartialKeyedClosetSet},
    },
    heuristic::Heuristic,
    directed::simple::SimpleGraph,
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
pub struct GoalSE2 {
    pub vertex: usize,
    pub orientation: Option<OrientationGoal>
}

impl Goal<Node> for GoalSE2 {
    fn is_satisfied(&self, node: &Node) -> bool {
        if node.partial_key().map(|k| k.vertex() != self.vertex).unwrap_or(false) {
            return false;
        }

        self.orientation.map(|r| {
            let delta_yaw = (node.state().position.rotation / r.target).angle().abs();
            delta_yaw <= r.threshold
        }).unwrap_or(true)
    }
}

impl PartialKeyed for GoalSE2 {
    type Key = usize;
    fn partial_key(&self) -> Option<&Self::Key> {
        Some(&self.vertex)
    }
}

impl Keyed for GoalSE2 { }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    Beginning,
    Finish,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KeySE2 {
    from_vertex: usize,
    to_vertex: usize,
    side: Side,
}

impl KeySE2 {
    pub fn vertex(&self) -> usize {
        match self.side {
            Side::Beginning => self.from_vertex,
            Side::Finish => self.to_vertex,
        }
    }
}

impl From<KeySE2> for usize {
    fn from(key: KeySE2) -> Self {
        key.vertex()
    }
}

impl StateKey<usize, se2::timed_position::Waypoint> for KeySE2 {
    fn make_child(&self, target_key: &usize, _: &se2::timed_position::Waypoint) -> Self {
        Self{
            from_vertex: self.to_vertex,
            to_vertex: *target_key,
            side: Side::Finish,
        }
    }
}

pub type Node = BuiltinNode<i64, KeySE2, se2::timed_position::Waypoint>;

pub struct ReachForLinearSE2 {
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
}

impl Reachable<Node, GoalSE2, se2::timed_position::Waypoint> for ReachForLinearSE2 {
    type ReachError = NoError;
    type Reaching<'a> = impl Iterator<Item=Result<se2::LinearTrajectory, NoError>>;

    fn reach_for<'a>(&'a self, parent: &'a Node, goal: &'a GoalSE2) -> Self::Reaching<'a> {
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
pub struct StartSE2 {
    pub vertex: usize,
    pub orientation: se2::Rotation,
}

pub struct LinearSE2Policy<G, S, C=DurationCostCalculator, H=DirectTravelHeuristic<G, C>>
where
    G: Graph<Vertex=se2::Point>,
    S: ClosedSet<Node>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64>
        + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2, GoalSE2, i64>,
{
    _ignore: std::marker::PhantomData<(G, S, C, H)>,
}

impl<G, S, C, H> Policy for LinearSE2Policy<G, S, C, H>
where
    G: Graph<Vertex=se2::Point, Key=usize>,
    S: ClosedSet<Node>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64>
        + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2, GoalSE2, i64>,
{
    type Waypoint = se2::timed_position::Waypoint;
    type ClosedSet = S;
    type Graph = G;
    type StateKey = KeySE2;
    type Node = Node;
    type Extrapolator = se2::timed_position::DifferentialDriveLineFollow;
    type Heuristic = H;
    type Reach = ReachForLinearSE2;
    type CostCalculator = C;
}

pub type TimeInvariantExpander<G, C, H> = Expander<LinearSE2Policy<G, PartialKeyedClosedSet<Node>, C, H>>;
pub type DefaultTimeInvariantExpander = TimeInvariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, DirectTravelHeuristic<SimpleGraph<se2::Point>, DurationCostCalculator>>;
pub fn make_default_time_invariant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> DefaultTimeInvariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(DirectTravelHeuristic{
        graph: graph.clone(),
        cost_calculator: cost_calculator.clone(),
        extrapolator: r2::timed_position::LineFollow::new(extrapolator.translational_speed()).unwrap(),
    });
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    DefaultTimeInvariantExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher,
    }
}

pub type TimeVariantExpander<G, C, H> =
    Chain<
        Expander<LinearSE2Policy<G, TimeVariantPartialKeyedClosetSet<Node>, C, H>>,
        Hold<se2::timed_position::Waypoint, C, Node>,
    >;
pub type DefaultTimeVariantExpander = TimeVariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, DirectTravelHeuristic<SimpleGraph<se2::Point>, DurationCostCalculator>>;

pub fn make_default_time_variant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> DefaultTimeVariantExpander {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(DirectTravelHeuristic{
        graph: graph.clone(),
        cost_calculator: cost_calculator.clone(),
        extrapolator: r2::timed_position::LineFollow::new(extrapolator.translational_speed()).unwrap(),
    });
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    Expander::<
        LinearSE2Policy<SimpleGraph<se2::Point>,
        TimeVariantPartialKeyedClosetSet<Node>,
        DurationCostCalculator,
        DirectTravelHeuristic<SimpleGraph<se2::Point>,
        DurationCostCalculator>>
    >{
        graph, extrapolator, cost_calculator: cost_calculator.clone(), heuristic, reacher,
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

impl<G, S, C, H> InitTargeted<StartSE2, GoalSE2> for Expander<LinearSE2Policy<G, S, C, H>>
where
    G: Graph<Vertex=r2::Position, Key=usize>,
    S: ClosedSet<Node>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64> + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2, GoalSE2, i64>,
{
    type InitTargetedError = InitErrorSE2<H::Error>;
    type InitialTargetedNodes<'a> where G: 'a, C: 'a, H: 'a, S: 'a = impl Iterator<Item=Result<Arc<Node>, Self::InitTargetedError>> + 'a;

    fn start<'a>(
        &'a self,
        start: &'a StartSE2,
        goal: &'a GoalSE2,
    ) -> Self::InitialTargetedNodes<'a> {
        [self.graph.vertex(start.vertex)].into_iter()
        .filter_map(|x| x)
        .flat_map(move |p0| {
            self.graph.edges_from_vertex(start.vertex).into_iter()
                .filter_map(|edge| -> Option<(&usize, &r2::Position)> {
                    let key = edge.endpoint_key();
                    self.graph.vertex(*key).map(|target| (key, target))
                })
                .map(move |(towards_key, towards_target)| {
                    let dx = towards_target - p0;
                    let wp0 = se2::timed_position::Waypoint{
                        time: TimePoint::zero(),
                        position: se2::Position::from_parts(p0.coords.into(), start.orientation)
                    };

                    if dx.norm() < 1e-8 {
                        // There is no direction to turn towards to reach the
                        // target waypoint.
                        return Ok((towards_key, wp0, None));
                    }

                    let trajectory = self.extrapolator.make_trajectory(
                        wp0.clone(),
                        &se2::Position::new(p0.coords, dx.y.atan2(dx.x))
                    ).map_err(InitErrorSE2::Extrapolator)?;

                    Ok((towards_key, wp0, trajectory))
                })
                .map(move |r| {
                    r.and_then(|(towards_vertex, state, trajectory)| {
                        let key = KeySE2{
                            from_vertex: start.vertex,
                            to_vertex: *towards_vertex,
                            side: Side::Beginning,
                        };

                        let h = self.heuristic.estimate_cost(
                            &key, goal,
                        ).map_err(InitErrorSE2::Heuristic)?;

                        Ok(h.map(|h| self.start_from(state, Some(key), h, trajectory)))
                    })
                })
                .filter_map(|r| r.transpose())
        })


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
        let expander = make_default_time_invariant_expander(
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
        let expander = make_default_time_variant_expander(
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
