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
    ecs::query::QueryEntityError,
    prelude::*,
    tasks::{block_on, AsyncComputeTaskPool, Task},
};
use futures_lite::future;
use std::{
    collections::{BTreeMap, HashMap},
    fmt::Debug,
    time::{Duration, Instant},
};

use mapf::negotiation::*;
use rmf_site_editor::{
    occupancy::{Cell, Grid},
    site::{
        Anchor, CurrentLevel, DifferentialDrive, Group, LocationTags, ModelMarker, NameInSite,
        Point, Pose, Task as RobotTask, Tasks as RobotTasks,
    },
};

use mapf::negotiation::{Agent, Obstacle, Scenario as MapfScenario};

pub mod debug_panel;
pub use debug_panel::*;

pub mod visual;
pub use visual::*;

#[derive(Default)]
pub struct NegotiationPlugin;

impl Plugin for NegotiationPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<NegotiationRequest>()
            .init_resource::<NegotiationParams>()
            .init_resource::<NegotiationData>()
            .add_plugins(NegotiationDebugPlugin::default())
            .add_systems(
                Update,
                (
                    start_compute_negotiation,
                    handle_compute_negotiation_complete,
                    visualise_selected_node,
                ),
            );
    }
}

#[derive(Event)]
pub struct NegotiationRequest;

#[derive(Debug, Clone, Resource)]
pub struct NegotiationParams {
    pub queue_length_limit: usize,
    pub cell_size: f32,
}

impl Default for NegotiationParams {
    fn default() -> Self {
        Self {
            queue_length_limit: 1_000_000,
            cell_size: 0.2,
        }
    }
}

#[derive(Debug, Default, Clone, Resource)]
pub enum NegotiationData {
    #[default]
    NotStarted,
    InProgress {
        start_time: Instant,
    },
    Complete {
        elapsed_time: Duration,
        solution: Option<NegotiationNode>,
        negotiation_history: Vec<NegotiationNode>,
        entity_id_map: HashMap<usize, Entity>,
        error_message: Option<String>,
        conflicting_endpoints: Vec<(Entity, Entity)>,
    },
}

impl NegotiationData {
    pub fn is_in_progress(&self) -> bool {
        matches!(self, NegotiationData::InProgress { .. })
    }
}

#[derive(Resource)]
pub struct NegotiationDebugData {
    pub show_debug_panel: bool,
    pub selected_negotiation_node: Option<usize>,
    pub visualize_keys: bool,
    pub visualize_conflicts: bool,
    pub visualize_trajectories: bool,
}

impl Default for NegotiationDebugData {
    fn default() -> Self {
        Self {
            show_debug_panel: false,
            selected_negotiation_node: None,
            visualize_keys: false,
            visualize_conflicts: true,
            visualize_trajectories: true,
        }
    }
}

#[derive(Component)]
pub struct ComputeNegotiateTask(
    Task<
        Result<
            (
                NegotiationNode,
                Vec<NegotiationNode>,
                HashMap<usize, String>,
            ),
            NegotiationError,
        >,
    >,
);

pub fn handle_compute_negotiation_complete(
    mut commands: Commands,
    mut compute_negotiation_task: Query<(Entity, &mut ComputeNegotiateTask)>,
    mut negotiation_debug_data: ResMut<NegotiationDebugData>,
    mut negotiation_data: ResMut<NegotiationData>,
) {
    let NegotiationData::InProgress { start_time } = *negotiation_data else {
        return;
    };

    fn bits_string_to_entity(bits_string: &str) -> Entity {
        let bits = u64::from_str_radix(bits_string, 10).expect("Invalid entity id");
        Entity::from_bits(bits)
    }

    for (mut entity, mut task) in &mut compute_negotiation_task {
        if let Some(result) = block_on(future::poll_once(&mut task.0)) {
            let elapsed_time = start_time.elapsed();

            match result {
                Ok((solution, negotiation_history, name_map)) => {
                    negotiation_debug_data.selected_negotiation_node = Some(solution.id);
                    *negotiation_data = NegotiationData::Complete {
                        elapsed_time,
                        solution: Some(solution),
                        negotiation_history,
                        entity_id_map: name_map
                            .into_iter()
                            .map(|(id, bits_string)| (id, bits_string_to_entity(&bits_string)))
                            .collect(),
                        error_message: None,
                        conflicting_endpoints: Vec::new(),
                    };
                }
                Err(err) => match err {
                    NegotiationError::PlanningImpossible(msg) => {
                        *negotiation_data = NegotiationData::Complete {
                            elapsed_time,
                            solution: None,
                            negotiation_history: Vec::new(),
                            entity_id_map: HashMap::new(),
                            error_message: Some(msg),
                            conflicting_endpoints: Vec::new(),
                        };
                    }
                    NegotiationError::ConflictingEndpoints(conflicts) => {
                        *negotiation_data = NegotiationData::Complete {
                            elapsed_time,
                            solution: None,
                            negotiation_history: Vec::new(),
                            entity_id_map: HashMap::new(),
                            error_message: None,
                            conflicting_endpoints: conflicts
                                .into_iter()
                                .map(|(a, b)| {
                                    (bits_string_to_entity(&a), bits_string_to_entity(&b))
                                })
                                .collect(),
                        };
                    }
                    NegotiationError::PlanningFailed((negotiation_history, name_map)) => {
                        *negotiation_data = NegotiationData::Complete {
                            elapsed_time,
                            solution: None,
                            negotiation_history,
                            entity_id_map: name_map
                                .into_iter()
                                .map(|(id, bits_string)| (id, bits_string_to_entity(&bits_string)))
                                .collect(),
                            error_message: None,
                            conflicting_endpoints: Vec::new(),
                        };
                    }
                },
            };
            commands.entity(entity).remove::<ComputeNegotiateTask>();
        }
    }
}

pub fn start_compute_negotiation(
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
    locations: Query<(Entity, &Point<Entity>), With<LocationTags>>,
    anchors: Query<(Entity, &Anchor, &Transform)>,
    negotiation_request: EventReader<NegotiationRequest>,
    negotiation_params: Res<NegotiationParams>,
    mut negotiation_data: ResMut<NegotiationData>,
    mut negotiation_debug_data: ResMut<NegotiationDebugData>,
    current_level: Res<CurrentLevel>,
    grids: Query<(Entity, &Grid)>,
    parents: Query<&Parent>,
) {
    if negotiation_request.len() == 0 {
        return;
    }
    if let NegotiationData::InProgress { .. } = *negotiation_data {
        warn!("Negotiation requested while another negotiation is in progress");
        return;
    }
    negotiation_debug_data.selected_negotiation_node = None;

    // Occupancy
    let mut occupancy = HashMap::<i64, Vec<i64>>::new();
    let mut cell_size = 1.0;
    let grid = grids
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
        .next();
    match grid {
        Some(grid) => {
            cell_size = grid.cell_size;
            for cell in grid.occupied.iter() {
                occupancy.entry(cell.y).or_default().push(cell.x);
            }
            for (_, column) in &mut occupancy {
                column.sort_unstable();
            }
        }
        None => {
            occupancy.entry(0).or_default().push(0);
            warn!("No occupancy grid found, defaulting to empty");
        }
    }

    // Agent
    let mut agents = BTreeMap::<String, Agent>::new();
    for (robot_entity, _, robot_pose, differential_drive, tasks) in mobile_robots.iter() {
        let Some(location_entity) = tasks
            .0
            .iter()
            .filter_map(|task| {
                if let RobotTask::GoToPlace(go_to_place) = task {
                    if let Some(location) = go_to_place.location {
                        Some(location.0)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .next()
        else {
            continue;
        };
        let Ok((_, Point(anchor_entity))) = locations.get(location_entity) else {
            continue;
        };
        let Ok((_, _, goal_transform)) = anchors.get(*anchor_entity) else {
            continue;
        };
        let agent = Agent {
            start: to_cell(robot_pose.trans[0], robot_pose.trans[1], cell_size),
            yaw: f64::from(robot_pose.rot.yaw().radians()),
            goal: to_cell(
                goal_transform.translation.x,
                goal_transform.translation.y,
                cell_size,
            ),
            radius: f64::from(differential_drive.collision_radius),
            speed: f64::from(differential_drive.translational_speed),
            spin: f64::from(differential_drive.rotational_speed),
        };
        let agent_id = robot_entity.to_bits().to_string();
        agents.insert(agent_id, agent);
    }
    if agents.len() == 0 {
        warn!("No agents with valid GoToPlace task");
        return;
    }

    let scenario = MapfScenario {
        agents: agents,
        obstacles: Vec::<Obstacle>::new(),
        occupancy: occupancy,
        cell_size: f64::from(cell_size),
        camera_bounds: None,
    };
    let queue_length_limit = negotiation_params.queue_length_limit;

    // Execute asynchronously
    let start_time = Instant::now();
    *negotiation_data = NegotiationData::InProgress { start_time };

    let thread_pool = AsyncComputeTaskPool::get();
    let task = thread_pool.spawn(async move { negotiate(&scenario, Some(queue_length_limit)) });

    commands.spawn(ComputeNegotiateTask(task));
}

fn to_cell(x: f32, y: f32, cell_size: f32) -> [i64; 2] {
    let cell = Cell::from_point(Vec2::new(x, y), cell_size);
    [cell.x, cell.y]
}
