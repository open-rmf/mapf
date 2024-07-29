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

use super::{Negotiate, NegotiationData};
use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_egui::egui::{
    Align, CollapsingHeader, Color32, ComboBox, DragValue, Frame, Layout, ScrollArea, Slider,
    Stroke, Ui,
};
use rmf_site_editor::{
    interaction::{Select, Selection},
    site::{
        location, mobile_robot, Category, Change, ChangeCurrentScenario, CurrentScenario, Delete,
        Group, MobileRobotMarker, NameInSite, Scenario, ScenarioMarker, SiteParent, Task, Tasks,
    },
    widgets::{prelude::*, view_scenarios::ScenarioDisplay, Icons},
};
use rmf_site_format::SiteID;

#[derive(SystemParam)]
pub struct NegotiationControlTile<'w, 's> {
    commands: Commands<'w, 's>,
    selection: Res<'w, Selection>,
    change_tasks: EventWriter<'w, Change<Tasks<Entity>>>,
    select: EventWriter<'w, Select>,
    mobile_robots: Query<
        'w,
        's,
        (
            Entity,
            &'static NameInSite,
            Option<&'static SiteID>,
            &'static mut Tasks<Entity>,
        ),
        (With<MobileRobotMarker>, Without<Group>),
    >,
    negotiation_data: Res<'w, NegotiationData>,
    negotiation_request: EventWriter<'w, Negotiate>,
    locations: Query<'w, 's, (&'static NameInSite, Option<&'static SiteID>)>,
}

impl<'w, 's> WidgetSystem<Tile> for NegotiationControlTile<'w, 's> {
    fn show(_: Tile, ui: &mut Ui, state: &mut SystemState<Self>, world: &mut World) -> () {
        let mut params = state.get_mut(world);
        CollapsingHeader::new("Negotiation")
            .default_open(false)
            .show(ui, |ui| {
                // Add button
                ui.add_enabled_ui(!params.negotiation_data.is_generating, |ui| {
                    if ui.button("Generate Plan").clicked() {
                        params.negotiation_request.send(Negotiate);
                    }
                });

                // Tasks
                ui.label("Tasks");
                task_frame(ui, |ui| {
                    let mut count = 0;
                    for (robot_entity, robot_name, robot_site_id, robot_tasks) in
                        params.mobile_robots.iter()
                    {
                        for task in robot_tasks.0.iter() {
                            if let Task::GoToPlace(SiteParent(Some(location_entity))) = task {
                                if let Ok((location_name, location_site_id)) =
                                    params.locations.get(*location_entity)
                                {
                                    show_task(
                                        ui,
                                        robot_name.0.as_str(),
                                        &robot_entity,
                                        robot_site_id.unwrap(),
                                        location_name.0.as_str(),
                                        location_entity,
                                        location_site_id.unwrap(),
                                        &params.selection,
                                        &mut params.select,
                                    );
                                    count += 1;
                                    break;
                                }
                            }
                        }
                    }
                    if count == 0 {
                        ui.label("No Tasks");
                    }
                });
            });
    }
}

fn task_frame<R>(ui: &mut Ui, add_contents: impl FnOnce(&mut Ui) -> R) {
    Frame::default()
        .inner_margin(4.0)
        .rounding(2.0)
        .stroke(Stroke::new(1.0, Color32::GRAY))
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            add_contents(ui);
        });
}

fn show_task(
    ui: &mut Ui,
    robot_name: &str,
    robot_entity: &Entity,
    robot_site_id: &u32,
    location_name: &str,
    location_entity: &Entity,
    location_site_id: &u32,
    selected: &Selection,
    select: &mut EventWriter<Select>,
) {
    let mut is_deleted = false;
    Frame::default()
        .inner_margin(4.0)
        .fill(Color32::DARK_GRAY)
        .rounding(2.0)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            // Mobile Robot
            ui.horizontal(|ui| {
                ui.label("Robot");
                if ui
                    .selectable_label(
                        selected.0.is_some_and(|s| s == *robot_entity),
                        format!("Model #{} [{}]", robot_site_id, robot_name).to_string(),
                    )
                    .clicked()
                {
                    select.send(Select(Some(*robot_entity)));
                }
            });
            // Goal
            ui.horizontal(|ui| {
                ui.label("To: ");
                if ui
                    .selectable_label(
                        selected.0.is_some_and(|s| s == *location_entity),
                        format!("Location #{} [{}]", location_site_id, location_name).to_string(),
                    )
                    .clicked()
                {
                    select.send(Select(Some(*location_entity)));
                }
            });
        });
}
