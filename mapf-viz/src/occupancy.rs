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

use derivative::Derivative;
use iced::{
    Rectangle,
    canvas::{Path, Stroke, Frame, Cursor, event::{self, Event}},
    keyboard, mouse,
};
use super::spatial_canvas::{self, InclusionZone, SpatialCanvasProgram, SpatialCache, Toggle};
use mapf::occupancy::{Grid, Visibility, SparseGrid, Cell};

#[derive(Derivative, Clone)]
#[derivative(Debug)]
pub struct OccupancyVisual<Message, G: Grid> {
    #[derivative(Debug="ignore")]
    occupancy: Visibility<G>,
    pressed: bool,
    change: Toggle,
    // NOTE: After changing one of the public fields below, you must clear the
    // cache of the SpatialCanvas that this program belongs to before the change
    // will be rendered.
    pub occupancy_color: iced::Color,
    pub visibility_color: iced::Color,
    pub show_visibility_graph: bool,
    pub show_details: bool,
    _msg: std::marker::PhantomData<Message>,
}

impl<Message, G: Grid> OccupancyVisual<Message, G> {

    pub fn new(grid: G, robot_radius: f32) -> Self {
        Self{
            occupancy: Visibility::new(grid, robot_radius as f64),
            pressed: false,
            change: Toggle::On,
            occupancy_color: iced::Color::from_rgb8(0x40, 0x44, 0x4B),
            visibility_color: iced::Color::from_rgb8(230, 166, 33),
            show_visibility_graph: true,
            show_details: false,
            _msg: Default::default(),
        }
    }

    pub fn set_robot_radius(&mut self, radius: f32) {
        self.occupancy.change_agent_radius(radius as f64);
    }

    pub fn visibility(&self) -> &Visibility<G> {
        &self.occupancy
    }

    pub fn visibility_mut(&mut self) -> &mut Visibility<G> {
        &mut self.occupancy
    }

    pub fn grid(&self) -> &G {
        &self.occupancy.grid()
    }

    fn toggle(&mut self, p: iced::Point) -> bool {
        let cell = Cell::from_point([p.x as f64, p.y as f64].into(), self.occupancy.grid().cell_size());
        match self.change {
            Toggle::On => {
                return self.occupancy.change_cells(&[(cell, true)].into());
            },
            Toggle::Off => {
                return self.occupancy.change_cells(&[(cell, false)].into());
            },
            Toggle::NoChange => {
                return false;
            }
        }
    }
}

impl<Message, G: Grid> SpatialCanvasProgram<Message> for OccupancyVisual<Message, G> {
    fn update(
        &mut self,
        event: Event,
        cursor: Cursor,
    ) -> (SpatialCache, event::Status, Option<Message>) {
        if let Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) = event {
            if modifiers.shift() {
                self.change = Toggle::Off;
            } else if modifiers.is_empty() {
                self.change = Toggle::On;
            } else {
                self.change = Toggle::NoChange;
            }

            if modifiers.alt() {
                self.show_details = true;
                return (SpatialCache::Refresh, event::Status::Ignored, None);
            } else {
                self.show_details = false;
                return (SpatialCache::Refresh, event::Status::Ignored, None);
            }
        }

        if let Some(p) = cursor.position() {
            if let Event::Mouse(mouse::Event::ButtonPressed(button)) = event {
                if mouse::Button::Left == button {
                    self.pressed = true;
                    if self.toggle(p) {
                        return (SpatialCache::Refresh, event::Status::Captured, None);
                    }
                }
            }

            if let Event::Mouse(mouse::Event::CursorMoved{..}) = event {
                if self.pressed {
                    if self.toggle(p) {
                        return (SpatialCache::Refresh, event::Status::Ignored, None);
                    }
                }
            }
        }

        if let Event::Mouse(mouse::Event::ButtonReleased(button)) = event {
            if mouse::Button::Left == button {
                self.pressed = false;
            }
        }

        (SpatialCache::Unchanged, event::Status::Ignored, None)
    }

    fn draw_in_space(
        &self,
        frame: &mut Frame,
        _: Rectangle,
        _: Cursor
    ) {
        let cell_size = self.occupancy.grid().cell_size();
        let robot_radius = self.occupancy.agent_radius() as f32;
        for cell in self.occupancy.grid().occupied_cells() {
            let p = cell.to_bottom_left_point(cell_size);
            frame.fill_rectangle(
                [p.x as f32, p.y as f32].into(),
                iced::Size::new(cell_size as f32, cell_size as f32),
                self.occupancy_color,
            );
        }

        if self.show_visibility_graph {
            for (cell, _) in self.occupancy.iter_points() {
                let p = cell.to_center_point(cell_size);
                frame.fill(
                    &Path::circle([p.x as f32, p.y as f32].into(), robot_radius),
                    self.visibility_color,
                );
            }

            for (cell_i, cell_j) in self.occupancy.iter_edges() {
                let p_i = cell_i.to_center_point(cell_size);
                let p_j = cell_j.to_center_point(cell_size);
                frame.stroke(
                    &Path::line(
                        [p_i.x as f32, p_i.y as f32].into(),
                        [p_j.x as f32, p_j.y as f32].into(),
                    ),
                    Stroke{
                        color: self.visibility_color,
                        width: 5_f32,
                        ..Default::default()
                    }
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
        _: Cursor
    ) {
        if !self.show_details {
            return;
        }

        for (cell, _) in self.occupancy.iter_points() {
            let p = cell.to_center_point(self.occupancy.grid().cell_size());
            let r = self.occupancy.agent_radius() as f32 / 2_f32.sqrt();
            let delta = iced::Vector::new(r, -r);
            let p = iced::Point::new(p.x as f32, p.y as f32) + delta;

            if bound.contains(p) {
                hud.at(
                    p,
                    |frame| {
                        frame.fill_text(format!("({}, {})", cell.x, cell.y).to_string());
                    }
                );
            }
        }
    }

    fn estimate_bounds(&self) -> InclusionZone {
        let mut zone = InclusionZone::Empty;
        let r = self.occupancy.agent_radius() as f32;
        for (cell, _) in self.occupancy.iter_points() {
            let p = cell.to_center_point(self.occupancy.grid().cell_size());
            let p: iced::Point = [p.x as f32, p.y as f32].into();
            let d = iced::Vector::new(r, r);
            zone.include(p + d);
            zone.include(p - d);
        }

        zone
    }
}

pub type SparseGridOccupancyVisual<Message> = OccupancyVisual<Message, SparseGrid>;
