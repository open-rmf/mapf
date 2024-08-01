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

use super::*;
use bevy::render::mesh::{shape::Quad, Mesh};
use rmf_site_editor::site::line_stroke_transform;

pub const DEFAULT_PATH_WIDTH: f32 = 0.2;

#[derive(Component)]
pub struct PathVisualMarker;

pub fn visualise_selected_node(
    mut commands: Commands,
    negotiation_data: Res<NegotiationData>,
    mut debug_data: ResMut<NegotiationDebugData>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut path_visuals: Query<Entity, With<PathVisualMarker>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mobile_robots: Query<(Entity, &DifferentialDrive), (With<ModelMarker>, Without<Group>)>,
    current_level: Res<CurrentLevel>,
) {
    // Return unless complete
    let NegotiationData::Complete {
        elapsed_time,
        solution,
        negotiation_history,
        entity_id_map,
        error_message,
        conflicting_endpoints,
    } = negotiation_data.as_ref()
    else {
        return;
    };
    if !debug_data.is_changed() && !negotiation_data.is_changed() {
        return;
    }
    let Some(selected_node) = debug_data
        .selected_negotiation_node
        .and_then(|selected_id| {
            if negotiation_history.is_empty() {
                solution.clone()
            } else {
                negotiation_history
                    .iter()
                    .find(|node| node.id == selected_id)
                    .map(|node| node.clone())
            }
        })
    else {
        return;
    };

    let Some(level_entity) = current_level.0 else {
        return;
    };

    for path_visual in path_visuals.iter() {
        commands.entity(path_visual).despawn_recursive();
    }
    let mut spawn_path_mesh = |lane_tf, lane_material: Handle<StandardMaterial>, lane_mesh| {
        commands
            .spawn(PbrBundle {
                mesh: lane_mesh,
                material: lane_material.clone(),
                transform: lane_tf,
                ..default()
            })
            .insert(PathVisualMarker)
            .set_parent(level_entity);
    };

    if debug_data.visualize_trajectories {
        for proposal in selected_node.proposals.iter() {
            let collision_radius = entity_id_map
                .get(&proposal.0)
                .and_then(|e| {
                    mobile_robots
                        .get(*e)
                        .map(|(_, dd)| dd.collision_radius)
                        .ok()
                })
                .unwrap_or(DEFAULT_PATH_WIDTH / 2.0);

            for (i, _waypoint) in proposal.1.meta.trajectory.iter().enumerate().skip(2) {
                let start_pos = proposal.1.meta.trajectory[i - 1].position.translation;
                let end_pos = proposal.1.meta.trajectory[i].position.translation;
                let start_pos = Vec3::new(start_pos.x as f32, start_pos.y as f32, 0.1);
                let end_pos = Vec3::new(end_pos.x as f32, end_pos.y as f32, 0.1);

                spawn_path_mesh(
                    line_stroke_transform(&start_pos, &end_pos, collision_radius * 2.0),
                    materials.add(StandardMaterial {
                        base_color: Color::rgb(0.0, 1.0, 0.0),
                        unlit: true,
                        ..Default::default()
                    }),
                    meshes.add(Mesh::from(shape::Quad {
                        size: Vec2::new(collision_radius * 10.0, collision_radius * 2.0),
                        flip: false,
                    })),
                );
            }
        }
    }
}
