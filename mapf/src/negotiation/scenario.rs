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
        se2::{GoalSE2, Orientation, StartSE2, WaypointSE2},
        TimePoint, Trajectory,
    },
};
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

#[derive(Serialize, Deserialize)]
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
}

#[derive(Serialize, Deserialize)]
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
