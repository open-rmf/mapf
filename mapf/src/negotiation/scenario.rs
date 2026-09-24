/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
    graph::occupancy::Cell,
    motion::{
        se2::{GoalSE2, Orientation, Position, StartSE2, WaypointSE2},
        TimePoint, Trajectory,
    },
};
use nalgebra::{Isometry2, Vector2};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

pub type LinearTrajectorySE2 = Trajectory<WaypointSE2>;

#[derive(Serialize, Deserialize, Clone, Debug, Copy)]
pub struct Agent {
    /// Start cell
    pub start: [i64; 2],
    /// Initial yaw of the robot
    pub yaw: f64,
    /// Goal cell
    pub goal: [i64; 2],
    /// Radius of the robot's footprint (meters)
    #[serde(default = " default_radius")]
    pub radius: f64,
    /// Translational speed of the robot (meters/sec)
    #[serde(default = "default_speed")]
    pub speed: f64,
    /// How fast the robot can spin (radians/sec)
    #[serde(default = "default_spin")]
    pub spin: f64,
    // TODO(@mxgrey): Allow parameters for travel effort cost
}

impl Agent {
    pub fn start_cell(&self) -> Cell {
        self.start.into()
    }

    pub fn goal_cell(&self) -> Cell {
        self.goal.into()
    }

    pub fn make_start(&self) -> StartSE2<Cell> {
        StartSE2 {
            time: TimePoint::zero(),
            key: Cell::from(self.start),
            orientation: Orientation::from_angle(self.yaw),
        }
    }

    pub fn make_goal(&self) -> GoalSE2<Cell> {
        GoalSE2::new(Cell::from(self.goal))
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Obstacle {
    /// Trajectory of the obstacle in terms of (time (s), x cell, y cell)
    pub trajectory: Vec<(f64, i64, i64)>,
    /// Radius of the obstacle
    #[serde(default = "default_radius")]
    pub radius: f64,
    #[serde(default = "bool_false", skip_serializing_if = "is_false")]
    pub indefinite_start: bool,
    #[serde(default = "bool_false", skip_serializing_if = "is_false")]
    pub indefinite_finish: bool,
}

impl Obstacle {
    pub fn new(radius: f64, trajectory: &LinearTrajectorySE2, cell_size: f64) -> Obstacle {
        Obstacle {
            trajectory: trajectory
                .iter()
                .map(|wp| {
                    let cell = Cell::from_point(wp.position.translation.vector.into(), cell_size);
                    (wp.time.as_secs_f64(), cell.x, cell.y)
                })
                .collect(),
            radius,
            indefinite_start: trajectory.has_indefinite_initial_time(),
            indefinite_finish: trajectory.has_indefinite_finish_time(),
        }
    }

    pub fn to_agent(&self) -> Option<Agent> {
        let first = self.trajectory.first()?;
        let last = self.trajectory.last()?;
        let yaw = if self.trajectory.len() > 1 {
            let second = &self.trajectory[1];
            let dx = (second.1 - first.1) as f64;
            let dy = (second.2 - first.2) as f64;
            dy.atan2(dx)
        } else {
            0.0
        };

        Some(Agent {
            start: [first.1, first.2],
            yaw,
            goal: [last.1, last.2],
            radius: self.radius,
            speed: default_speed(),
            spin: default_spin(),
        })
    }

    pub fn interpolate(&self, time: f64, cell_size: f64) -> Position {
        if self.trajectory.is_empty() {
            return Position::identity();
        }

        if time <= self.trajectory[0].0 {
            let (_, x, y) = self.trajectory[0];
            let p = Cell::new(x, y).center_point(cell_size);
            return Position::translation(p.x, p.y);
        }

        if time >= self.trajectory.last().unwrap().0 {
            let (_, x, y) = *self.trajectory.last().unwrap();
            let p = Cell::new(x, y).center_point(cell_size);
            return Position::translation(p.x, p.y);
        }

        // Binary search for the interval
        let result = self.trajectory.binary_search_by(|(t, _, _)| {
            t.partial_cmp(&time).unwrap_or(std::cmp::Ordering::Equal)
        });

        let idx = match result {
            Ok(i) => i,
            Err(i) => i,
        };

        if idx == 0 {
            let (_, x, y) = self.trajectory[0];
            let p = Cell::new(x, y).center_point(cell_size);
            return Position::translation(p.x, p.y);
        }

        let (t0, x0, y0) = self.trajectory[idx - 1];
        let (t1, x1, y1) = self.trajectory[idx];

        let f = (time - t0) / (t1 - t0);
        let p0 = Cell::new(x0, y0).center_point(cell_size);
        let p1 = Cell::new(x1, y1).center_point(cell_size);

        let x = p0.x + f * (p1.x - p0.x);
        let y = p0.y + f * (p1.y - p0.y);
        let yaw = (p1.y - p0.y).atan2(p1.x - p0.x);

        Isometry2::new(Vector2::new(x, y), yaw)
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Scenario {
    pub agents: BTreeMap<String, Agent>,
    pub obstacles: Vec<Obstacle>,
    // y -> [..x..]
    pub occupancy: HashMap<i64, Vec<i64>>,
    #[serde(default = "default_cell_size")]
    pub cell_size: f64,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub camera_bounds: Option<[[f32; 2]; 2]>,
}

pub fn default_radius() -> f64 {
    0.45
}

pub fn default_speed() -> f64 {
    0.75
}

pub fn default_spin() -> f64 {
    60_f64.to_radians()
}

pub fn default_cell_size() -> f64 {
    1.0
}

pub fn bool_false() -> bool {
    false
}

pub fn is_false(b: &bool) -> bool {
    !b
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::Cost;
    use crate::negotiation::NegotiationNode;
    use std::collections::HashMap;

    #[test]
    fn test_derive_mapf_result_with_obstacles() {
        let scenario = Scenario {
            agents: BTreeMap::new(),
            obstacles: vec![Obstacle {
                trajectory: vec![(0.0, 0, 0), (1.0, 1, 0)],
                radius: 0.5,
                indefinite_start: false,
                indefinite_finish: false,
            }],
            occupancy: HashMap::new(),
            cell_size: 1.0,
            camera_bounds: None,
        };

        // We need a mock solution node.
        // Proposals can be empty if there are no agents.
        let solution = NegotiationNode {
            negotiation: crate::negotiation::Negotiation::default(),
            proposals: HashMap::new(),
            environment: crate::motion::CcbsEnvironment::new(std::sync::Arc::new(
                crate::motion::DynamicEnvironment::new(
                    crate::motion::CircularProfile::new(0.0, 0.0, 0.0).unwrap(),
                ),
            )),
            keys: std::collections::HashSet::new(),
            conceded: None,
            cost: Cost(0.0),
            depth: 0,
            outcome: crate::negotiation::NodeOutcome::Success,
            id: 0,
            parent: None,
        };

        let timestep = 0.5;
        let mapf_result = scenario.derive_mapf_result(&solution, timestep);

        // One obstacle should result in one trajectory
        assert_eq!(mapf_result.trajectories.len(), 1);
        assert_eq!(mapf_result.footprints.len(), 1);

        // Obstacle trajectory from t=0 to t=1 with timestep 0.5 should have 3 poses (0.0, 0.5, 1.0)
        assert_eq!(mapf_result.trajectories[0].poses.len(), 3);

        // Check first and last poses
        let p0 = mapf_result.trajectories[0].poses[0].translation.vector;
        let p2 = mapf_result.trajectories[0].poses[2].translation.vector;

        // Cell (0,0) center is (0.5, 0.5)
        assert!((p0.x - 0.5).abs() < 1e-6);
        assert!((p0.y - 0.5).abs() < 1e-6);

        // Cell (1,0) center is (1.5, 0.5)
        assert!((p2.x - 1.5).abs() < 1e-6);
        assert!((p2.y - 0.5).abs() < 1e-6);
    }
}
