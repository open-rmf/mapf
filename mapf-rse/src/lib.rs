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

pub mod simulation;
pub use simulation::*;

pub mod negotiation;
use librmf_site_editor::widgets::PropertiesTilePlugin;
pub use negotiation::*;

use bevy::prelude::*;

#[derive(Default)]
pub struct MapfRsePlugin;

impl Plugin for MapfRsePlugin {
    fn build(&self, app: &mut App) {
        app.add_state::<DebugMode>()
            .init_resource::<SimulationConfig>()
            .add_plugins(PropertiesTilePlugin::<SimulationControlTile>::new());

        app.add_event::<Negotiate>()
            .init_resource::<NegotiationData>()
            .add_plugins(PropertiesTilePlugin::<NegotiationControlTile>::new())
            .add_systems(Update, generate_plan);
    }
}
