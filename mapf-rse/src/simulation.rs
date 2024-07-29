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

use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_egui::egui::{
    Button, CollapsingHeader, Color32, ComboBox, DragValue, Frame, ScrollArea, Slider, Stroke, Ui,
};
use librmf_site_editor::{
    interaction::{Select, Selection},
    site::{
        Category, Change, ChangeCurrentScenario, CurrentScenario, Delete, Group, MobileRobotMarker,
        NameInSite, Scenario, ScenarioMarker, Tasks,
    },
    widgets::prelude::*,
    widgets::{view_scenarios::ScenarioDisplay, Icons},
};

#[derive(SystemParam)]
pub struct SimulationControlTile<'w> {
    simulation_config: ResMut<'w, SimulationConfig>,
    debug_mode: Res<'w, State<DebugMode>>,
    debug_mode_next: ResMut<'w, NextState<DebugMode>>,
}

impl<'w> WidgetSystem<Tile> for SimulationControlTile<'w> {
    fn show(_: Tile, ui: &mut Ui, state: &mut SystemState<Self>, world: &mut World) -> () {
        let mut params = state.get_mut(world);
        ui.separator();

        CollapsingHeader::new("Simulation")
            .default_open(true)
            .show(ui, |ui| {
                // Play/pause, speed, stepo
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(!params.simulation_config.is_playing, "⏸")
                        .clicked()
                    {
                        params.simulation_config.is_playing = !params.simulation_config.is_playing;
                    };
                    if ui
                        .selectable_label(params.simulation_config.is_playing, "▶")
                        .clicked()
                    {
                        params.simulation_config.is_playing = !params.simulation_config.is_playing;
                    };

                    ui.add(
                        DragValue::new(&mut params.simulation_config.speed)
                            .clamp_range(0_f32..=10.0)
                            .suffix(" ×")
                            .speed(0.01),
                    );
                    ui.horizontal(|ui| match params.debug_mode.get() {
                        DebugMode::Negotation => {
                            let end_time = params.simulation_config.end_time.clone();
                            ui.add(
                                DragValue::new(&mut params.simulation_config.current_time)
                                    .clamp_range(0_f32..=end_time)
                                    .suffix(format!(" / {} s", end_time.to_string()))
                                    .speed(0.01),
                            );
                        }
                        DebugMode::Planner => {
                            let end_step = params.simulation_config.end_step.clone();
                            ui.add(
                                DragValue::new(&mut params.simulation_config.current_step)
                                    .clamp_range(0_u32..=end_step)
                                    .suffix(format!(" / {} steps", end_step.to_string()))
                                    .speed(0.01),
                            );
                        }
                    });
                });

                // Time/step control
                ui.scope(|ui| {
                    ui.spacing_mut().slider_width = ui.available_width();
                    ui.horizontal(|ui| match params.debug_mode.get() {
                        DebugMode::Negotation => {
                            let end_time = params.simulation_config.end_time.clone();
                            ui.add(
                                Slider::new(
                                    &mut params.simulation_config.current_time,
                                    0.0..=end_time,
                                )
                                .show_value(false)
                                .clamp_to_range(true),
                            );
                        }
                        DebugMode::Planner => {
                            let end_step = params.simulation_config.end_step.clone();
                            ui.add(
                                Slider::new(
                                    &mut params.simulation_config.current_step,
                                    0..=end_step,
                                )
                                .show_value(false)
                                .step_by(1.0)
                                .clamp_to_range(true),
                            );
                        }
                    });
                });
            });
    }
}

#[derive(Resource, Debug, Clone)]
pub struct SimulationConfig {
    pub is_playing: bool,
    pub speed: f32,
    pub current_time: f32,
    pub end_time: f32,
    pub current_step: u32,
    pub end_step: u32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            is_playing: false,
            speed: 1.0,
            current_time: 0.0,
            end_time: 0.0,
            current_step: 0,
            end_step: 0,
        }
    }
}

#[derive(Clone, Default, Eq, PartialEq, Debug, Hash, States)]
pub enum DebugMode {
    #[default]
    Negotation,
    Planner,
}

impl DebugMode {
    pub fn labels() -> Vec<&'static str> {
        vec!["Negotation", "Planner"]
    }

    pub fn label(&self) -> &str {
        match self {
            DebugMode::Negotation => Self::labels()[0],
            DebugMode::Planner => Self::labels()[1],
        }
    }

    pub fn from_label(label: &str) -> Self {
        if label == Self::labels()[0] {
            return DebugMode::Negotation;
        } else {
            return DebugMode::Planner;
        }
    }
}
