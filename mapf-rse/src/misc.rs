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

use bevy::{ecs::hierarchy::ChildOf, prelude::*};
use rmf_site_editor::{
    site::{
        CircleCollision, Collision, DifferentialDrive, Mobility, RobotProperty, RobotPropertyKind,
    },
    workspace::CurrentWorkspace,
};
use rmf_site_format::{
    AssetSource, Category, Group, IsStatic, ModelDescriptionBundle, ModelMarker, ModelProperty,
    NameInSite, Robot, Scale,
};

/// Loads the TinyRobot model into the scene with differential drive and a mobile robot marker for convenience
pub fn load_tiny_robot(
    mut commands: Commands,
    robots: Query<&NameInSite, (With<Group>, With<Robot>)>,
    current_workspace: Res<CurrentWorkspace>,
) {
    if !current_workspace.is_changed() {
        return;
    }
    let Some(site_entity) = current_workspace.root else {
        return;
    };
    if robots.iter().find(|name| name.0 == "TinyRobot").is_some() {
        return;
    }

    let description = ModelDescriptionBundle {
        name: NameInSite("TinyRobot".to_string()),
        source: ModelProperty(AssetSource::Remote(
            "Open-RMF/TinyRobot/model.sdf".to_string(),
        )),
        is_static: ModelProperty(IsStatic(false)),
        ..default()
    };

    let mut robot = Robot::default();
    let differential_drive = DifferentialDrive {
        bidirectional: false,
        rotation_center_offset: [0.0, 0.0],
        translational_speed: 1.0,
        rotational_speed: 1.0,
    };
    if let Ok(serialized_mobility) = serde_json::to_value(differential_drive)
        .map(|diff_drive| Mobility::new(DifferentialDrive::label(), diff_drive))
        .and_then(|mobility| serde_json::to_value(mobility))
    {
        robot
            .properties
            .insert(Mobility::label(), serialized_mobility);
    }

    let circle_collision = CircleCollision {
        radius: 0.3,
        offset: [0.0, 0.0],
    };
    if let Ok(serialized_collision) = serde_json::to_value(circle_collision)
        .map(|circle| Mobility::new(CircleCollision::label(), circle))
        .and_then(|collision| serde_json::to_value(collision))
    {
        robot
            .properties
            .insert(Collision::label(), serialized_collision);
    }

    commands
        .spawn(description)
        .insert(Category::ModelDescription)
        .insert(robot)
        .insert(ChildOf(site_entity));
}
