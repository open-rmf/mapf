/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

use bevy::{
    ecs::system::{CommandQueue, SystemState},
    prelude::*,
    tasks::{block_on, AsyncComputeTaskPool, Task},
    time::Stopwatch,
};
use std::{
    collections::{BTreeMap, HashMap},
    fmt::Debug,
    time::Duration,
};

use librmf_site_editor::{
    interaction::{Select, Selection},
    occupancy::{Cell, Grid},
    site::{
        level, location, Anchor, Category, Change, ChangeCurrentScenario, CurrentLevel,
        CurrentScenario, Delete, DifferentialDrive, Group, LocationTags, MobileRobotMarker,
        ModelMarker, NameInSite, Point, Pose, Scenario, ScenarioMarker, SiteParent,
        Task as RobotTask, Tasks as RobotTasks,
    },
};

use mapf::negotiation::{Agent, Obstacle, Scenario as MapfScenario};

pub mod control_tile;
pub use control_tile::*;

// pub mod debug_panel;
// pub use debug_panel::*;

#[derive(Event)]
pub struct Negotiate;

#[derive(Component)]
pub struct ComputeNegotiateTask(Task<Duration>);

#[derive(Default, Debug, Clone, Resource)]
pub struct NegotiationData {
    pub is_generating: bool,
    pub cell_size: f32,
    pub planning_elapsed_time: Duration,
}

pub fn generate_plan(
    mut commands: Commands,
    mobile_robots: Query<
        (
            Entity,
            &NameInSite,
            &Pose,
            &DifferentialDrive,
            &RobotTasks<Entity>,
        ),
        (With<ModelMarker>, Without<Group>),
    >,
    locations: Query<(Entity, &Point<Entity>), With<LocationTags>>, // locations: Query<'w, 's, (&'static NameInSite, Option<&'static SiteID>)>,
    anchors: Query<(Entity, &Anchor, &Transform)>,
    negotiation_request: EventReader<Negotiate>,
    negotiation_data: Res<NegotiationData>,
    current_level: Res<CurrentLevel>,
    grids: Query<(Entity, &Grid)>,
    parents: Query<&Parent>,
) {
    if negotiation_request.len() == 0 {
        return;
    }

    // Occupancy
    let mut occupancy = HashMap::<i64, Vec<i64>>::new();
    let Some(grid) = grids
        .iter()
        .filter_map(|(grid_entity, grid)| {
            if let Some(level_entity) = current_level.0 {
                if parents
                    .get(grid_entity)
                    .is_ok_and(|parent_entity| parent_entity.get() == level_entity)
                {
                    Some(grid)
                } else {
                    None
                }
            } else {
                None
            }
        })
        .next()
    else {
        println!("No occupancy map available");
        return;
    };
    let cell_size = grid.cell_size;
    for cell in grid.occupied.iter() {
        occupancy.entry(cell.y).or_default().push(cell.x);
    }

    for (_, column) in &mut occupancy {
        column.sort_unstable();
    }
    println!("Y_MAX: {:?}", occupancy.len());
    println!("X_MAX: {:?}", occupancy.values().map(|x| x.len()).max());

    // Agent
    let mut agents = BTreeMap::<String, Agent>::new();
    for (robot_entity, robot_name, robot_pose, differential_drive, tasks) in mobile_robots.iter() {
        let Some(location_entity) = tasks
            .0
            .iter()
            .filter_map(|task| {
                if let RobotTask::GoToPlace(SiteParent(Some(location_entity))) = task {
                    Some(location_entity)
                } else {
                    None
                }
            })
            .next()
        else {
            continue;
        };
        let Ok((_, Point(anchor_entity))) = locations.get(*location_entity) else {
            continue;
        };
        let Ok((_, _, goal_transform)) = anchors.get(*anchor_entity) else {
            continue;
        };

        let agent = Agent {
            start: to_cell(
                robot_pose.trans[0],
                robot_pose.trans[1],
                cell_size,
            ),
            yaw: f64::from(robot_pose.rot.yaw().radians()),
            goal: to_cell(
                goal_transform.translation.x,
                goal_transform.translation.y,
                cell_size,
            ),
            radius: 0.5,
            speed: f64::from(differential_drive.translational_speed),
            spin: f64::from(differential_drive.rotational_speed),
        };
        agents.insert(format!("{:?}", robot_entity), agent);
    }
    if agents.len() == 0 {
        println!("No active agents");
        return;
    }

    // Obstacles
    let mut obstacles = Vec::<Obstacle>::new();

    let scenario = MapfScenario {
        agents: agents,
        obstacles: obstacles,
        occupancy: occupancy,
        cell_size: f64::from(cell_size),
        camera_bounds: None,
    };

    // Execute asynchronously
    let thread_pool = AsyncComputeTaskPool::get();
    let start_time = std::time::Instant::now();
    let task = thread_pool.spawn(async move {
        let res = mapf::negotiation::negotiate(&scenario, Some(1_000_000));
        let elapsed_time = start_time.elapsed();
        println!("Elapsed time: {:?}", elapsed_time);
        elapsed_time
    });
    let task_entity = commands.spawn_empty().id();
    commands.entity(task_entity).insert(ComputeNegotiateTask(task));

}

pub fn to_cell(x: f32, y: f32, cell_size: f32) -> [i64; 2] {
    println!("TO_CELL: {:?}, {:?}, {:?}", x, y, cell_size);
    let cell = Cell::from_point(Vec2::new(x, y), cell_size);
    println!("CELL: {:?}", cell);
    [cell.x, cell.y]
}
