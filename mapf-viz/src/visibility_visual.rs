/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
    spatial_canvas::{self, InclusionZone, SpatialCache, SpatialCanvasProgram},
    toggle::{DragToggler, FillToggler, Toggle, Toggler},
};
use derivative::Derivative;
use iced::{
    canvas::{
        event::{self, Event},
        Cursor, Frame, Path, Stroke,
    },
    keyboard, mouse, Rectangle,
};
use mapf::graph::occupancy::{Cell, Grid, Point, SparseGrid, Visibility};
use std::collections::HashMap;

#[derive(Derivative)]
#[derivative(Debug)]
pub struct VisibilityVisual<Message, G: Grid> {
    #[derivative(Debug = "ignore")]
    visibility: Visibility<G>,

    // NOTE: After changing one of the public fields below, you must clear the
    // cache of the SpatialCanvas that this program belongs to before the change
    // will be rendered.
    pub occupancy_color: iced::Color,
    pub default_visibility_color: iced::Color,
    pub special_visibility_color: HashMap<Cell, iced::Color>,
    pub show_visibility_graph: bool,
    pub show_details: bool,

    #[derivative(Debug = "ignore")]
    pub cell_toggler: Option<Box<dyn Toggler>>,

    #[derivative(Debug = "ignore")]
    pub corner_select_toggler: Option<Box<dyn Toggler>>,

    #[derivative(Debug = "ignore")]
    pub on_corner_select: Option<Box<dyn Fn(Cell, bool) -> Message>>,

    #[derivative(Debug = "ignore")]
    pub on_occupancy_change: Option<Box<dyn Fn() -> Message>>,

    _ignore: std::marker::PhantomData<Message>,
}

impl<Message, G: Grid> VisibilityVisual<Message, G> {
    pub fn new(
        grid: G,
        robot_radius: f32,
        on_corner_select: Option<Box<dyn Fn(Cell, bool) -> Message>>,
        on_occupancy_change: Option<Box<dyn Fn() -> Message>>,
    ) -> Self {
        Self {
            visibility: Visibility::new(grid, robot_radius as f64),
            occupancy_color: iced::Color::from_rgb8(0x40, 0x44, 0x4B),
            default_visibility_color: iced::Color::from_rgb8(230, 166, 33),
            special_visibility_color: Default::default(),
            show_visibility_graph: true,
            show_details: false,
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
            corner_select_toggler: Some(Box::new(FillToggler::new(
                DragToggler::new(None, Some((keyboard::Modifiers::ALT, mouse::Button::Left))),
                DragToggler::new(None, Some((keyboard::Modifiers::ALT, mouse::Button::Right))),
            ))),
            on_corner_select,
            on_occupancy_change,
            _ignore: Default::default(),
        }
    }

    pub fn showing_visibility_graph(mut self, showing: bool) -> Self {
        self.show_visibility_graph = showing;
        self
    }

    pub fn set_robot_radius(&mut self, radius: f32) {
        self.visibility.change_agent_radius(radius as f64);
    }

    pub fn visibility(&self) -> &Visibility<G> {
        &self.visibility
    }

    pub fn visibility_mut(&mut self) -> &mut Visibility<G> {
        &mut self.visibility
    }

    pub fn grid(&self) -> &G {
        &self.visibility.grid()
    }

    pub fn set_grid(&mut self, grid: G) {
        self.visibility = Visibility::new(grid, self.visibility.agent_radius());
    }

    /// The cache of the spatial canvas needs to be cleared after changing this
    /// value or else the change will not show.
    pub fn show_details(&mut self, show: bool) {
        self.show_details = show;
    }

    fn toggle(&mut self, p: iced::Point) -> bool {
        if let Some(cell_toggler) = &self.cell_toggler {
            let cell = Cell::from_point(
                [p.x as f64, p.y as f64].into(),
                self.visibility.grid().cell_size(),
            );
            match cell_toggler.state() {
                Toggle::On => {
                    return self.visibility.change_cells(&[(cell, true)].into());
                }
                Toggle::Off => {
                    return self.visibility.change_cells(&[(cell, false)].into());
                }
                Toggle::NoChange => {
                    return false;
                }
            }
        }

        return false;
    }

    fn find_closest(&self, p: iced::Point) -> Option<Cell> {
        let mut closest: Option<(Cell, f64)> = None;
        let r = self.visibility.agent_radius();
        let r_squared = r * r;
        let p = Point::new(p.x as f64, p.y as f64);

        for (cell, _) in self.visibility.iter_points() {
            let p_cell = cell.center_point(self.grid().cell_size());
            let dist = (p_cell - p).norm_squared();
            if dist <= r_squared {
                if let Some((_, old_dist)) = closest {
                    if dist < old_dist {
                        closest = Some((*cell, dist));
                    }
                } else {
                    closest = Some((*cell, dist));
                }
            }
        }

        return closest.map(|x| x.0);
    }
}

impl<Message, G: Grid> SpatialCanvasProgram<Message> for VisibilityVisual<Message, G> {
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

            if let Some(corner_select_toggler) = &mut self.corner_select_toggler {
                if let Some(on_corner_select) = &self.on_corner_select {
                    corner_select_toggler.toggle(event);
                    match corner_select_toggler.state() {
                        Toggle::On => {
                            if let Some(cell) = self.find_closest(p) {
                                return (
                                    SpatialCache::Unchanged,
                                    event::Status::Captured,
                                    Some(on_corner_select(cell, true)),
                                );
                            }
                        }
                        Toggle::Off => {
                            if let Some(cell) = self.find_closest(p) {
                                return (
                                    SpatialCache::Unchanged,
                                    event::Status::Captured,
                                    Some(on_corner_select(cell, false)),
                                );
                            }
                        }
                        Toggle::NoChange => {
                            // Do nothing
                        }
                    }
                }
            }
        }

        (SpatialCache::Unchanged, event::Status::Ignored, None)
    }

    fn draw_in_space(&self, frame: &mut Frame, _: Rectangle, _: Cursor) {
        let cell_size = self.visibility.grid().cell_size();
        let robot_radius = self.visibility.agent_radius() as f32;
        for cell in self.visibility.grid().occupied_cells() {
            let p = cell.bottom_left_point(cell_size);
            frame.fill_rectangle(
                [p.x as f32, p.y as f32].into(),
                iced::Size::new(cell_size as f32, cell_size as f32),
                self.occupancy_color,
            );
        }

        if self.show_visibility_graph {
            for (cell, _) in self.visibility.iter_points() {
                let p = cell.center_point(cell_size);
                let color = self
                    .special_visibility_color
                    .get(cell)
                    .unwrap_or(&self.default_visibility_color);

                frame.fill(
                    &Path::circle([p.x as f32, p.y as f32].into(), robot_radius),
                    *color,
                );
            }

            for (cell_i, cell_j) in self.visibility.iter_edges() {
                let p_i = cell_i.center_point(cell_size);
                let p_j = cell_j.center_point(cell_size);
                frame.stroke(
                    &Path::line(
                        [p_i.x as f32, p_i.y as f32].into(),
                        [p_j.x as f32, p_j.y as f32].into(),
                    ),
                    Stroke {
                        color: self.default_visibility_color,
                        width: 5_f32,
                        ..Default::default()
                    },
                );

                // if self.show_details {
                //     let p0 = Point::new(p0.x, p0.y);
                //     let p1 = Point::new(p1.x, p1.y);
                //     let dist = (p1 - p0).length();
                //     if dist < 1e-8 {
                //         continue;
                //     }
                //     let v = (p1 - p0)/dist;
                //     let n = Vector::new(-v.y, v.x);

                //     let points = [
                //         p0 + n*self.robot_radius,
                //         p0 - n*self.robot_radius,
                //         p1 + n*self.robot_radius,
                //         p1 - n*self.robot_radius,
                //     ];

                //     let lines = [
                //         (points[0], points[1]),
                //         (points[0], points[2]),
                //         (points[1], points[3]),
                //         (points[2], points[3]),
                //     ];

                //     let cell_x_min = (
                //         points.iter().min_by(
                //             |p_l, p_r| p_l.x.partial_cmp(&p_r.x).unwrap()
                //         ).unwrap().x / self.cell_size
                //     ).floor() as i64;

                //     let cell_x_max = (
                //         points.iter().max_by(
                //             |p_l, p_r| p_l.x.partial_cmp(&p_r.x).unwrap()
                //         ).unwrap().x / self.cell_size
                //     ).ceil() as i64;

                //     for line in &lines {
                //         frame.stroke(
                //             &Path::line(
                //                 iced::Point::new(line.0.x, line.0.y),
                //                 iced::Point::new(line.1.x, line.1.y),
                //             ),
                //             Stroke{
                //                 color: iced::Color::from_rgb(0.0, 1.0, 0.0),
                //                 width: 8_f32,
                //                 ..Default::default()
                //             }
                //         );
                //     }

                //     for (cell_x, _) in self.occupancy_map.range(cell_x_min .. cell_x_max) {
                //         let x_low = *cell_x as f32 * self.cell_size;
                //         let x_high = (cell_x + 1) as f32 * self.cell_size;
                //         for x in [x_low, x_high] {
                //             for line in &lines {
                //                 for y in line.vertical_intersect(x) {
                //                     frame.fill(
                //                         &Path::circle(iced::Point::new(x, y), self.robot_radius/10_f32),
                //                         iced::Color::from_rgb(0.0, 0.0, 1.0)
                //                     );
                //                 }
                //             }
                //         }
                //     }
                // }
            }
        }
    }

    fn draw_on_hud<'a, 'b: 'a>(
        &self,
        hud: &'a mut spatial_canvas::SpatialHUD<'b>,
        bound: Rectangle,
        _: Cursor,
    ) {
        if !self.show_details {
            return;
        }

        for (cell, _) in self.visibility.iter_points() {
            let p = cell.center_point(self.visibility.grid().cell_size());
            let r = self.visibility.agent_radius() as f32 / 2_f32.sqrt();
            let delta = iced::Vector::new(r, -r);
            let p = iced::Point::new(p.x as f32, p.y as f32) + delta;

            if bound.contains(p) {
                hud.at(p, |frame| {
                    frame.fill_text(format!("({}, {})", cell.x, cell.y).to_string());
                });
            }
        }
    }

    fn estimate_bounds(&self) -> InclusionZone {
        let mut zone = InclusionZone::Empty;
        let r = self.visibility.agent_radius() as f32;
        for (cell, _) in self.visibility.iter_points() {
            let p = cell.center_point(self.visibility.grid().cell_size());
            let p: iced::Point = [p.x as f32, p.y as f32].into();
            let d = iced::Vector::new(r, r);
            zone.include(p + d);
            zone.include(p - d);
        }

        zone
    }
}

pub type SparseGridVisibilityVisual<Message> = VisibilityVisual<Message, SparseGrid>;
