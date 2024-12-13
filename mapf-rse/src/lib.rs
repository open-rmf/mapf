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

pub mod negotiation;
pub use negotiation::*;

pub mod config_widget;
pub use config_widget::*;

pub mod simulation;
pub use simulation::*;

pub mod misc;
pub use misc::*;

use rmf_site_editor::widgets::PropertiesTilePlugin;

use bevy::prelude::*;

#[derive(Default)]
pub struct MapfRsePlugin;

impl Plugin for MapfRsePlugin {
    fn build(&self, app: &mut App) {
        app.add_state::<DebugMode>()
            .init_resource::<SimulationConfig>()
            .add_plugins(NegotiationPlugin)
            .add_plugins(PropertiesTilePlugin::<MapfConfigWidget>::new())
            .add_systems(Update, load_tiny_robot);
    }
}

#[derive(Clone, Default, Eq, PartialEq, Debug, Hash, States)]
pub enum DebugMode {
    #[default]
    Negotiation,
    Planner,
}

impl DebugMode {
    pub fn labels() -> Vec<&'static str> {
        vec!["Negotiation", "Planner"]
    }

    pub fn label(&self) -> &str {
        match self {
            DebugMode::Negotiation => Self::labels()[0],
            DebugMode::Planner => Self::labels()[1],
        }
    }

    pub fn from_label(label: &str) -> Self {
        if label == Self::labels()[0] {
            return DebugMode::Negotiation;
        } else {
            return DebugMode::Planner;
        }
    }
}
