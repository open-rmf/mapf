/*
 * Copyright (C) 2024 active Source Robotics Foundation
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
use bevy::{ecs::system::SystemParam, prelude::*, ui};
use bevy_egui::egui::{
    self, Align, CollapsingHeader, Color32, Frame, Grid, ImageSource, InnerResponse, Response,
    RichText, ScrollArea, Stroke, Ui, Window,
};
use mapf::negotiation::Negotiation;
use rmf_site_editor::{
    interaction::{outline, ChangeMode, ModelPreviewCamera, SelectAnchor3D},
    site::{
        AssetSource, FuelClient, Group, IsStatic, Model, ModelDescriptionBundle, ModelMarker,
        ModelProperty, NameInSite, Scale, SetFuelApiKey, UpdateFuelCache,
    },
    widgets::prelude::*,
};

#[derive(Default)]
pub struct NegotiationDebugPlugin;

impl Plugin for NegotiationDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NegotiationDebugData>();
        let panel = PanelWidget::new(negotiation_debug_panel, &mut app.world);
        let widget = Widget::new::<NegotiationDebugWidget>(&mut app.world);
        app.world.spawn((panel, widget));
    }
}

#[derive(SystemParam)]
pub struct NegotiationDebugWidget<'w, 's> {
    commands: Commands<'w, 's>,
    negotiation_data: ResMut<'w, NegotiationData>,
    negotiation_debug_data: ResMut<'w, NegotiationDebugData>,
}

fn negotiation_debug_panel(In(input): In<PanelWidgetInput>, world: &mut World) {
    if world.resource::<NegotiationDebugData>().show_debug_panel {
        egui::SidePanel::left("negotiation_debug_panel")
            .resizable(true)
            .min_width(320.0)
            .show(&input.context, |ui| {
                if let Err(err) = world.try_show(input.id, ui) {
                    error!("Unable to display asset gallery: {err:?}");
                }
            });
    }
}

impl<'w, 's> WidgetSystem for NegotiationDebugWidget<'w, 's> {
    fn show(_: (), ui: &mut Ui, state: &mut SystemState<Self>, world: &mut World) {
        let mut params = state.get_mut(world);

        ui.heading("Negotiation Debugger");
        match params.negotiation_data.as_ref() {
            NegotiationData::Complete { .. } => {
                params.show_completed(ui);
            }
            NegotiationData::InProgress { start_time } => {
                ui.label(format!(
                    "In Progresss: {} s",
                    start_time.elapsed().as_secs_f32()
                ));
            }
            NegotiationData::NotStarted => {
                ui.label("No negotiation started");
            }
        }
    }
}

impl<'w, 's> NegotiationDebugWidget<'w, 's> {
    pub fn show_completed(&mut self, ui: &mut Ui) {
        let NegotiationData::Complete {
            elapsed_time,
            solution,
            negotiation_history,
            entity_id_map,
            error_message,
            conflicting_endpoints,
        } = self.negotiation_data.as_ref()
        else {
            return;
        };
        // Solution node
        ui.add_space(10.0);
        ui.label(format!(
            "Solution [found in {} s]",
            elapsed_time.as_secs_f32()
        ));
        match solution {
            Some(solution) => {
                show_negotiation_node(
                    ui,
                    &mut HashMap::new(),
                    &mut self.negotiation_debug_data,
                    solution,
                );
            }
            None => {
                outline_frame(ui, |ui| {
                    ui.label("No solution found");
                });
            }
        }
        // Error display
        ui.add_space(10.0);
        ui.label("Errors");
        if let Some(error_message) = error_message {
            outline_frame(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label(error_message.clone());
                });
            });
        } else {
            outline_frame(ui, |ui| {
                ui.label("No errors");
            });
        }
        // Negotiatio history
        ui.add_space(10.0);
        ui.label("Negotiation History");

        let mut id_response_map = HashMap::<usize, &mut Response>::new();
        ScrollArea::vertical().show(ui, |ui| {
            for negotiation_node in negotiation_history {
                let id = negotiation_node.id;
                let mut response = show_negotiation_node(
                    ui,
                    &mut id_response_map,
                    &mut self.negotiation_debug_data,
                    negotiation_node,
                );
                // id_response_map.insert(id, &mut response);
            }
        });
    }
}

fn show_negotiation_node(
    mut ui: &mut Ui,
    mut id_response_map: &mut HashMap<usize, &mut Response>,
    mut negotiation_debug_data: &mut ResMut<NegotiationDebugData>,
    node: &NegotiationNode,
) -> Response {
    Frame::default()
        .inner_margin(4.0)
        .fill(Color32::DARK_GRAY)
        .rounding(2.0)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());

            let id = node.id;
            ui.horizontal(|ui| {
                let selected = negotiation_debug_data.selected_negotiation_node == Some(id);
                if ui.radio(selected, format!("#{}", node.id)).clicked() {
                    negotiation_debug_data.selected_negotiation_node = Some(id);
                }
                ui.label("|");
                ui.label(format!("Keys: {}", node.keys.len()));
                ui.label("|");
                ui.label(format!("Conflicts: {}", node.negotiation.conflicts.len()));
                ui.label("|");
                ui.label("Parent");
                match node.parent {
                    Some(parent) => {
                        if ui.button(format!("#{}", parent)).clicked() {
                            if let Some(response) = id_response_map.get_mut(&parent) {
                                response.scroll_to_me(Some(Align::Center));
                            }
                        }
                    }
                    None => {
                        ui.label("None");
                    }
                }
            });

            CollapsingHeader::new("Information")
                .id_source(id.to_string() + "node_info")
                .default_open(false)
                .show(ui, |ui| {
                    ui.label("Keys");
                    for key in &node.keys {
                        ui.label(format!("{:?}", key));
                    }
                });
        })
        .response
}

fn outline_frame<R>(ui: &mut Ui, add_body: impl FnOnce(&mut Ui) -> R) -> Response {
    Frame::default()
        .inner_margin(4.0)
        .stroke(Stroke::new(1.0, Color32::GRAY))
        .rounding(2.0)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.add_enabled_ui(true, add_body);
        })
        .response
}
