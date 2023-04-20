/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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


use super::{
    spatial_canvas::{InclusionZone, SpatialCache, SpatialCanvasProgram},
    toggle::{DragToggler, FillToggler, Toggle, Toggler},
};
use derivative::Derivative;
use iced::{
    canvas::{
        event::{self, Event},
        Cursor, Frame, Path,
    },
    keyboard, mouse, Rectangle,
};
use mapf::graph::occupancy::{
    Cell, Grid, SparseGrid, Accessibility,
    accessibility_graph::CellAccessibility,
};

#[derive(Derivative)]
#[derivative(Debug)]
pub struct AccessibilityVisual<Message, G: Grid> {
    #[derivative(Debug = "ignore")]
    accessibility: Accessibility<G>,

    // NOTE: After changing one of the public fields below, you must clear the
    // cache of the SpatialCanvas that this program belongs to before the change
    // will be rendered.
    pub occupancy_color: iced::Color,
    pub inaccessible_color: iced::Color,
    pub open_corner_color: iced::Color,
    pub show_accessibility: bool,

    #[derivative(Debug = "ignore")]
    pub cell_toggler: Option<Box<dyn Toggler>>,

    #[derivative(Debug = "ignore")]
    pub on_occupancy_change: Option<Box<dyn Fn() -> Message>>,

    _ignore: std::marker::PhantomData<Message>,
}

impl<Message, G: Grid> AccessibilityVisual<Message, G> {
    pub fn new(
        grid: G,
        robot_radius: f32,
        on_occupancy_change: Option<Box<dyn Fn() -> Message>>,
    ) -> Self {
        Self {
            accessibility: Accessibility::new(grid, robot_radius as f64),
            occupancy_color: iced::Color::from_rgb8(0x40, 0x44, 0x4B),
            inaccessible_color: iced::Color::from_rgb8(204, 210, 219),
            open_corner_color: iced::Color::from_rgb8(240, 240, 240),
            show_accessibility: true,
            cell_toggler: Some(Box::new(FillToggler::new(
                DragToggler::new(
                    None,
                    Some((keyboard::Modifiers::SHIFT, mouse::Button::Left)),
                ),
                DragToggler::new(
                    None,
                    Some((keyboard::Modifiers::SHIFT, mouse::Button::Right)),
                ),
            ))),
            on_occupancy_change,
            _ignore: Default::default(),
        }
    }

    pub fn showing_accessibility(mut self, showing: bool) -> Self {
        self.show_accessibility = showing;
        self
    }

    pub fn set_robot_radius(&mut self, radius: f32) {
        self.accessibility.change_agent_radius(radius as f64);
    }

    pub fn accessibility(&self) -> &Accessibility<G> {
        &self.accessibility
    }

    pub fn accessibiltiy_mut(&mut self) -> &mut Accessibility<G> {
        &mut self.accessibility
    }

    pub fn grid(&self) -> &G {
        &self.accessibility.grid()
    }

    pub fn set_grid(&mut self, grid: G) {
        self.accessibility = Accessibility::new(grid, self.accessibility.agent_radius());
    }

    fn toggle(&mut self, p: iced::Point) -> bool {
        if let Some(cell_toggler) = &self.cell_toggler {
            let cell = Cell::from_point(
                [p.x as f64, p.y as f64].into(),
                self.accessibility.grid().cell_size(),
            );
            match cell_toggler.state() {
                Toggle::On => {
                    return self.accessibility.change_cells([(cell, true)].into());
                }
                Toggle::Off => {
                    return self.accessibility.change_cells([(cell, false)].into());
                }
                Toggle::NoChange => {
                    return false;
                }
            }
        }

        return false;
    }
}

impl<Message, G: Grid> SpatialCanvasProgram<Message> for AccessibilityVisual<Message, G> {
    fn update(
        &mut self,
        event: Event,
        cursor: Cursor,
    ) -> (SpatialCache, event::Status, Option<Message>) {
        if let Some(cell_toggler) = &mut self.cell_toggler {
            cell_toggler.toggle(event);
        }

        if let Some(p) = cursor.position() {
            if self.toggle(p) {
                return (
                    SpatialCache::Refresh,
                    event::Status::Captured,
                    self.on_occupancy_change.as_ref().map(|x| x()),
                );
            }
        }

        (SpatialCache::Unchanged, event::Status::Ignored, None)
    }

    fn draw_in_space(&self, frame: &mut Frame, _: Rectangle, _: Cursor) {
        let cell_size = self.accessibility.grid().cell_size();
        for cell in self.accessibility.grid().occupied_cells() {
            let p = cell.bottom_left_point(cell_size);
            frame.fill_rectangle(
                [p.x as f32, p.y as f32].into(),
                iced::Size::new(cell_size as f32, cell_size as f32),
                self.occupancy_color,
            );
        }

        if self.show_accessibility {
            for (cell, a) in self.accessibility.iter_accessibility() {
                if a.is_inaccessible() {
                    let p = cell.bottom_left_point(cell_size);
                    frame.fill_rectangle(
                        [p.x as f32, p.y as f32].into(),
                        iced::Size::new(cell_size as f32, cell_size as f32),
                        self.inaccessible_color,
                    );
                }
            }

            for (cell, a) in self.accessibility.iter_accessibility() {
                if let CellAccessibility::Accessible(directions) = a {
                    let p0 = cell.center_point(cell_size);
                    let p0 = iced::Point::new(p0.x as f32, p0.y as f32);
                    for [i, j] in directions.iter() {
                        if i == 0 || j == 0 {
                            continue;
                        }

                        let i_bl = (i+1) / 2;
                        let j_bl = (j+1) / 2;

                        let show_corner = 'corner: {
                            for [k, m] in [[0, 0], [-1, 0], [-1, -1], [0, -1]] {
                                let check = cell.shifted(i_bl, j_bl).shifted(k, m);
                                if self.accessibility.is_inaccessible(&check) {
                                    break 'corner true;
                                }
                            }

                            false
                        };

                        if show_corner {
                            let p1 = cell.shifted(i, j).center_point(cell_size);
                            let p1 = iced::Point::new(p1.x as f32, p1.y as f32);
                            let ratio = 1.0/4.0 as f32;
                            let w = cell_size as f32 * ratio;
                            let path = Path::new(|builder| {
                                builder.move_to(p0 + iced::Vector::new(w, 0.0));
                                builder.line_to(p1 + iced::Vector::new(w, 0.0));
                                builder.line_to(p1 + iced::Vector::new(-w, 0.0));
                                builder.line_to(p0 + iced::Vector::new(-w, 0.0));
                            });

                            frame.fill(&path, self.open_corner_color);
                        }
                    }
                }
            }
        }
    }

    fn estimate_bounds(&self) -> InclusionZone {
        let mut zone = InclusionZone::Empty;
        let r = self.accessibility.agent_radius() + self.accessibility.grid().cell_size();
        let r = r as f32;
        for cell in self.accessibility.grid().occupied_cells().into_iter().chain(
            self.accessibility.iter_accessibility().map(|(c, _)| c)
        ) {
            let p = cell.center_point(self.accessibility.grid().cell_size());
            let p: iced::Point = [p.x as f32, p.y as f32].into();
            let d = iced::Vector::new(r, r);
            zone.include(p + d);
            zone.include(p - d);
        }

        zone
    }
}

pub type SparseGridAccessibilityVisual<Message> = AccessibilityVisual<Message, SparseGrid>;
