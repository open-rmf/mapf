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
        Extrapolator, TimePoint, Trajectory,
        se2, r2,
        graph_search::{Policy, BuiltinNode, Expander, MakeBuiltinNode, Key as GraphSearchKeyTrait},
        trajectory::{CostCalculator, DurationCostCalculator},
        reach::Reachable,
    },
    expander::{Goal, Initializable},
    node::{
        Agent, PartialKeyed, Keyed,
        closed_set::{ClosedSet, PartialKeyedClosedSet, TimeVariantPartialKeyedClosetSet},
    },
    heuristic::Heuristic,
    directed::simple::SimpleGraph,
};
use std::sync::Arc;
use std::fmt::Debug;
use num::Zero;

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

impl GraphSearchKeyTrait<usize, se2::timed_position::Waypoint> for KeySE2 {
    fn make_child(&self, target_key: &usize, _: &se2::timed_position::Waypoint) -> Self {
        Self{
            from_vertex: self.to_vertex,
            to_vertex: *target_key,
            side: Side::Finish,
        }
    }
}

pub type Node = BuiltinNode<i64, KeySE2, se2::timed_position::Waypoint>;

#[derive(Debug)]
pub struct DirectTravelHeuristic<G: Graph<Vertex=r2::Position, Key=usize>, C: CostCalculator<r2::timed_position::Waypoint>> {
    pub graph: Arc<G>,
    pub cost_calculator: Arc<C>,
    pub extrapolator: r2::timed_position::LineFollow,
}

impl<G: Graph<Vertex=r2::Position, Key=usize>, C: CostCalculator<r2::timed_position::Waypoint, Cost=i64>> Heuristic<KeySE2, GoalSE2, i64> for DirectTravelHeuristic<G, C> {
    type Error = <r2::timed_position::LineFollow as Extrapolator<r2::timed_position::Waypoint, r2::Position>>::Error;

    fn estimate_cost(
        &self,
        from_state: &KeySE2,
        to_goal: &GoalSE2
    ) -> Result<Option<i64>, Self::Error> {
        let p0 = {
            // Should this actually be an error? The current state is off the
            // graph entirely.
            if let Some(p) = self.graph.vertex(from_state.vertex()) {
                p
            } else {
                return Ok(None);
            }
        };

        let p1 = {
            if let Some(p) = self.graph.vertex(to_goal.vertex) {
                p
            } else {
                return Ok(None);
            }
        };

        let wp0 = r2::timed_position::Waypoint{
            time: TimePoint::zero(),
            position: *p0,
        };

        let motion = self.extrapolator.extrapolate(&wp0, p1)?;
        let trajectory = r2::LinearTrajectory::from_iter(motion).ok();

        let cost = trajectory.map(|t| self.cost_calculator.compute_cost(&t)).unwrap_or(C::Cost::zero());
        Ok(Some(cost))
    }
}

pub struct ReachForLinearSE2 {
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
}

impl Reachable<Node, GoalSE2, se2::timed_position::Waypoint> for ReachForLinearSE2 {
    type ReachError = ();
    type Reaching<'a> = impl Iterator<Item=Result<se2::LinearTrajectory, ()>>;

    fn reach_for<'a>(&'a self, parent: &'a Node, goal: &'a GoalSE2) -> Self::Reaching<'a> {
        [parent].into_iter()
        .filter(|n| {
            if let Some(key) = n.partial_key() {
                return key.vertex() == goal.vertex;
            }

            return false;
        })
        .filter_map(|n| goal.orientation.map(|g| (n, g.target)))
        .map(|(n, target_orientation)| {
            let to_target = se2::Position::from_parts(
                n.state().position.translation, target_orientation
            );

            let motion = self.extrapolator.extrapolate(n.state(), &to_target)?;
            Ok(Trajectory::from_iter(
                [n.state().clone()].into_iter().chain(motion.into_iter())
            ).ok())
        })
        .filter_map(|r| r.transpose())
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
    type Key = KeySE2;
    type Node = Node;
    type Extrapolator = se2::timed_position::DifferentialDriveLineFollow;
    type Heuristic = H;
    type Reach = ReachForLinearSE2;
    type MakeNode = MakeBuiltinNode<se2::timed_position::Waypoint, Node>;
    type CostCalculator = C;
}

pub type TimeInvariantExpander<G, C, H> = Expander<LinearSE2Policy<G, PartialKeyedClosedSet<Node>, C, H>>;
pub type DefaultTimeInvariantExpander = TimeInvariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, DirectTravelHeuristic<SimpleGraph<se2::Point>, DurationCostCalculator>>;
pub fn make_default_time_invariant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> Arc<DefaultTimeInvariantExpander> {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(DirectTravelHeuristic{
        graph: graph.clone(),
        cost_calculator: cost_calculator.clone(),
        extrapolator: r2::timed_position::LineFollow::new(extrapolator.translational_speed()).unwrap(),
    });
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    Arc::new(DefaultTimeInvariantExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher, node_spawner: Default::default(),
    })
}

pub type TimeVariantExpander<G, C, H> = Expander<LinearSE2Policy<G, TimeVariantPartialKeyedClosetSet<Node>, C, H>>;
pub type DefaultTimeVariantExpander = TimeVariantExpander<SimpleGraph<se2::Point>, DurationCostCalculator, DirectTravelHeuristic<SimpleGraph<se2::Point>, DurationCostCalculator>>;

pub fn make_default_time_variant_expander(
    graph: Arc<SimpleGraph<se2::Point>>,
    extrapolator: Arc<se2::timed_position::DifferentialDriveLineFollow>,
) -> Arc<DefaultTimeVariantExpander> {
    let cost_calculator = Arc::new(DurationCostCalculator);
    let heuristic = Arc::new(DirectTravelHeuristic{
        graph: graph.clone(),
        cost_calculator: cost_calculator.clone(),
        extrapolator: r2::timed_position::LineFollow::new(extrapolator.translational_speed()).unwrap(),
    });
    let reacher = Arc::new(ReachForLinearSE2{
        extrapolator: extrapolator.clone(),
    });

    Arc::new(DefaultTimeVariantExpander{
        graph, extrapolator, cost_calculator, heuristic, reacher, node_spawner: Default::default(),
    })
}


#[derive(Debug)]
pub enum InitErrorSE2<H: Heuristic<KeySE2, GoalSE2, i64>> {
    Extrapolator(<se2::timed_position::DifferentialDriveLineFollow as Extrapolator<se2::timed_position::Waypoint, r2::Position>>::Error),
    Heuristic(H::Error)
}

impl<G, S, C, H> Initializable<StartSE2, GoalSE2> for Expander<LinearSE2Policy<G, S, C, H>>
where
    G: Graph<Vertex=r2::Position, Key=usize>,
    S: ClosedSet<Node>,
    C: CostCalculator<se2::timed_position::Waypoint, Cost=i64> + CostCalculator<r2::timed_position::Waypoint, Cost=i64>,
    H: Heuristic<KeySE2, GoalSE2, i64>,
{
    type InitError = InitErrorSE2<H>;
    type InitialNodes<'a> where G: 'a, C: 'a, H: 'a, S: 'a = impl Iterator<Item=Result<Arc<Node>, Self::InitError>> + 'a;

    fn start<'a>(
        &'a self,
        start: &'a StartSE2,
        goal: &'a GoalSE2,
    ) -> Self::InitialNodes<'a> {
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

                    let target = se2::Position::new(p0.coords, dx.y.atan2(dx.x));
                    let waypoints = self.extrapolator.extrapolate(
                        &wp0, &target
                    ).map_err(InitErrorSE2::Extrapolator)?;
                    let trajectory = Trajectory::from_iter(
                        [wp0.clone()].into_iter().chain(waypoints)
                    ).ok();

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

                        Ok(h.map(|h| self.make_node(state, key, h, trajectory, None)))
                    })
                })
                .filter_map(|r| r.transpose())
        })


    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use se2::{
        Point,
        timed_position::DifferentialDriveLineFollow,
    };
    use crate::{
        a_star, algorithm::Status,
        planner::make_planner,
        expander::{Constrainable, Chainable},
        motion::{
            self,
            graph_search::MakeNode,
            collide::CircleCollisionConstraint,
        },
        node::Informed,
    };
    use time_point::Duration;

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

        let planner = make_planner(expander, Arc::new(a_star::Algorithm));
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

        let cost_calculator = expander.cost_calculator.clone();

        let expander = Arc::new(expander.chain_fn_no_err(
            move |parent, _| {
                let motion = Trajectory::hold(parent.state().clone(), parent.state().time + Duration::from_secs_f64(1.0)).ok().unwrap();
                let cost = cost_calculator.compute_cost(&motion);
                let spawner = MakeBuiltinNode::<se2::timed_position::Waypoint, Node>::default();
                [Ok(spawner.make_node(
                    motion.finish().clone(),
                    parent.partial_key().unwrap().clone(),
                    cost,
                    parent.remaining_cost_estimate(),
                    Some(motion),
                    Some(parent.clone()),
                ))].into_iter()
            }
        ));

        let constraint = Arc::new(CircleCollisionConstraint{
            obstacles: vec![(0.5, make_test_trajectory())],
            agent_radius: 1.0,
        });

        let expander = Arc::new(expander.constrain(constraint));

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
