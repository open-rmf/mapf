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

use iced::{
    Point, Rectangle,
    canvas::{Frame, Cursor}
};

use super::{SpatialCanvasProgram, spatial_canvas::InclusionZone};

#[derive(Debug)]
pub struct InfiniteGrid<Message> {
    origin: Point,
    cell_size: f32,
    width: f32,
    _msg: std::marker::PhantomData<Message>,
}

impl<Message> InfiniteGrid<Message> {

    pub fn new(cell_size: f32) -> Self {
        Self{
            origin: Point::ORIGIN,
            cell_size,
            width: cell_size/10_f32,
            _msg: Default::default(),
        }
    }

    fn traverse(
        &self,
        frame: &mut Frame,
        origin_v: f32,
        v_min: f32,
        v_max: f32,
        f: impl Fn(f32, &mut Frame)
    ) {
        let mut v = origin_v;
        while v >= v_min {
            if v <= v_max {
                f(v, frame);
            }

            v -= self.cell_size;
        }

        v = origin_v + self.cell_size;
        while v <= v_max {
            if v >= v_min {
                f(v, frame);
            }

            v += self.cell_size;
        }
    }
}

impl<Message: std::fmt::Debug> SpatialCanvasProgram<Message> for InfiniteGrid<Message> {
    fn draw_in_space(
        &self,
        frame: &mut Frame,
        spatial_bounds: Rectangle,
        _: Cursor
    ) {
        let x_min = spatial_bounds.x;
        let x_max = spatial_bounds.x + spatial_bounds.width;
        let y_min = spatial_bounds.y;
        let y_max = spatial_bounds.y + spatial_bounds.height;
        let line_color = iced::Color::from_rgb8(200, 204, 213);

        self.traverse(
            frame, self.origin.x, x_min, x_max,
            |x, frame| {
                frame.fill_rectangle(
                    iced::Point::new(x - self.width/2.0, y_min),
                    iced::Size::new(self.width, y_max - y_min),
                    line_color,
                );
            }
        );

        self.traverse(
            frame, self.origin.y, y_min, y_max,
            |y, frame| {
                frame.fill_rectangle(
                    iced::Point::new(x_min, y - self.width/2.0),
                    iced::Size::new(x_max - x_min, self.width),
                    line_color,
                );
            }
        );
    }

    fn estimate_bounds(&self) -> InclusionZone {
        InclusionZone::Empty
    }
}
