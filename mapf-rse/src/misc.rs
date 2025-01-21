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

use bevy::prelude::*;
use rmf_site_editor::{site::site, workspace::CurrentWorkspace};
use rmf_site_format::{
    AssetSource, Category, DifferentialDrive, Group, IsStatic, MobileRobotMarker,
    ModelDescriptionBundle, ModelMarker, ModelProperty, NameInSite, Scale,
};

/// Loads the TinyRobot model into the scene with differential drive and a mobile robot marker for convenience
pub fn load_tiny_robot(
    mut commands: Commands,
    mobile_robots: Query<&NameInSite, (With<Group>, With<MobileRobotMarker>)>,
    current_workspace: Res<CurrentWorkspace>,
) {
    if !current_workspace.is_changed() {
        return;
    }
    let Some(site_entity) = current_workspace.root else {
        return;
    };
    if mobile_robots
        .iter()
        .find(|name| name.0 == "TinyRobot")
        .is_some()
    {
        return;
    }

    let description = ModelDescriptionBundle {
        name: NameInSite("TinyRobot".to_string()),
        source: ModelProperty(AssetSource::Remote("Luca/TinyRobot/model.sdf".to_string())),
        is_static: ModelProperty(IsStatic(false)),
        scale: ModelProperty(Scale::default()),
        marker: ModelMarker,
        group: Group,
    };
    // Wrapping differential_drive and mobile_robot_marker in ModelProperty allows us
    // to apply the same properties to its instances
    let differential_drive = ModelProperty(DifferentialDrive {
        translational_speed: 1.0,
        rotational_speed: 1.0,
        bidirectional: false,
        collision_radius: 0.3,
        rotation_center_offset: [0.0, 0.0],
    });
    let mobile_robot_marker = ModelProperty::<MobileRobotMarker>::default();
    commands
        .spawn(description)
        .insert(differential_drive)
        .insert(Category::ModelDescription)
        .insert(mobile_robot_marker)
        .set_parent(site_entity);
}
