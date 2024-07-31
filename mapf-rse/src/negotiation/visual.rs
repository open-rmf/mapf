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
use bevy::prelude::*;
use mapf::motion::se2::WaypointSE2;

pub fn visualise_selected_node(
    mut gizmos: Gizmos,
    negotiation_data: Res<NegotiationData>,
    mut debug_data: ResMut<NegotiationDebugData>,
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

    let selected_node = if let Some(solution_node) = solution {
        debug_data.selected_negotiation_node = Some(solution_node.id.clone());
        solution_node
    } else if let Some(selected_node_id) = debug_data.selected_negotiation_node {
        let node = negotiation_history
            .iter()
            .find(|node| node.id == selected_node_id);
        if node.is_none() {
            warn!("Selected negotiation node not found");
            return;
        }
        node.unwrap()
    } else {
        return;
    };

    if debug_data.visualize_trajectories {
        for proposal in selected_node.proposals.iter() {
            let line: Vec<(Vec3, Color)> = proposal
                .1
                .meta
                .trajectory
                .iter()
                .map(|WaypointSE2 { time: _, position }| {
                    (
                        Vec3::new(
                            position.translation.x as f32,
                            position.translation.y as f32,
                            0.1,
                        ),
                        Color::GREEN,
                    )
                })
                .collect();
            gizmos.linestrip_gradient(line);
        }
    }

    return;
}
