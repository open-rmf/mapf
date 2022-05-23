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
    Element, Length, Rectangle,
    canvas::{self, Canvas, Cache, Frame, Geometry, Cursor, Path, event::{self, Event}},
    mouse, keyboard,
};
use lyon::math::{Point, Vector, Transform};
use super::{Toggle, Toggler, toggle::DragToggler};

pub struct SpatialHUD<'a> {
    base_frame: &'a mut Frame,
    zoom: f32,
}

impl<'a> SpatialHUD<'a> {
    pub fn at<'b>(&'b mut self, position: iced::Point, f: impl FnOnce(&mut Frame))
    where 'a: 'b {
        self.base_frame.with_save(|frame| {
            frame.translate(position - iced::Point::ORIGIN);
            frame.asymmetric_scale(1.0/self.zoom, -1.0/self.zoom);
            frame.with_save(f);
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SpatialCache {
    Refresh,
    Unchanged,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InclusionZone {
    Empty,
    Some{
        lower: iced::Point,
        upper: iced::Point,
    }
}

pub trait SpatialCanvasProgram<Message>: std::fmt::Debug {
    fn update(
        &mut self,
        _event: Event,
        _cursor: Cursor,
    ) -> (SpatialCache, event::Status, Option<Message>) {
        // Default: Do nothing
        (SpatialCache::Unchanged, event::Status::Ignored, None)
    }

    fn draw_in_space(
        &self,
        _frame: &mut Frame,
        _spatial_bounds: Rectangle,
        _spatial_cursor: Cursor
    ) {
        // Default: Do nothing
    }

    fn draw_on_hud<'a, 'b: 'a>(
        &self,
        _hud: &'a mut SpatialHUD<'b>,
        _spatial_bounds: Rectangle,
        _spatial_cursor: Cursor
    ) {
        // Default: Do nothing
    }

    fn estimate_bounds(&self) -> InclusionZone;
}

impl InclusionZone {
    pub fn include(&mut self, p: iced::Point) {
        match self {
            Self::Empty => {
                *self = Self::Some{lower: p, upper: p};
            },
            Self::Some{lower, upper} => {
                lower.x = lower.x.min(p.x);
                lower.y = lower.y.min(p.y);
                upper.x = upper.x.max(p.x);
                upper.y = upper.y.max(p.y);
            }
        }
    }

    pub fn merge(&mut self, other: Self) {
        match self {
            Self::Empty => *self = other,
            Self::Some{lower, upper} => {
                if let Self::Some{lower: other_lower, upper: other_upper} = other {
                    lower.x = lower.x.min(other_lower.x);
                    lower.y = lower.y.min(other_lower.y);
                    upper.x = upper.x.max(other_upper.x);
                    upper.y = upper.y.max(other_upper.y);
                }
            }
        }
    }
}

#[derive(Derivative)]
#[derivative(Debug)]
pub struct SpatialCanvas<Message, Program: SpatialCanvasProgram<Message>> {
    pub program: Program,
    pub cache: Cache,
    pub zoom: f32,
    #[derivative(Debug="ignore")]
    pub pan_toggler: Option<Box<dyn Toggler>>,
    pub scroll_to_zoom: bool,
    offset: Vector,
    bounds: Option<Rectangle>,
    drag_start_point: Option<Point>,
    _msg: std::marker::PhantomData<Message>,
}

impl<Message, Program: SpatialCanvasProgram<Message>> SpatialCanvas<Message, Program> {

    pub fn new(program: Program) -> Self {
        Self{
            program,
            cache: Cache::new(),
            zoom: 1.0,
            pan_toggler: Some(Box::new(DragToggler::new(
                Some(mouse::Button::Middle),
                Some((keyboard::Modifiers::CTRL, mouse::Button::Left)),
            ))),
            scroll_to_zoom: true,
            offset: Vector::new(0.0, 0.0),
            bounds: None,
            drag_start_point: None,
            _msg: Default::default(),
        }
    }

    pub fn fit_to_bounds(&mut self) {
        self.fit_to_zone(self.program.estimate_bounds());
    }

    pub fn fit_to_zone(&mut self, zone: InclusionZone) {
        let bounds =
            if let Some(bounds) = &mut self.bounds {
                bounds
            } else {
                return;
            };

        if let InclusionZone::Some{lower, upper} = zone {
            let x_zoom = bounds.width / (upper.x - lower.x);
            let y_zoom = bounds.height / (upper.y - lower.y);
            self.zoom = x_zoom.min(y_zoom);

            let bound_center = Point::new(bounds.width/2.0, bounds.height/2.0);

            let space_center = Point::new(
                (lower.x + upper.x)/2.0,
                (lower.y + upper.y)/2.0
            );
            let s = Transform::scale(self.zoom, -self.zoom);
            self.offset = bound_center - s.transform_point(space_center);
            self.cache.clear();
        }
    }

    pub fn view<'a>(&'a mut self) -> Element<'a, Message> where Message: 'static {
        Canvas::new(self)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    pub fn transform(&self) -> Transform {
        Transform::translation(self.offset.x, self.offset.y)
            .pre_scale(self.zoom, -self.zoom)
            .inverse().unwrap()
    }
}

impl<'a, Message, Program: SpatialCanvasProgram<Message>> canvas::Program<Message> for SpatialCanvas<Message, Program> {
    fn update(
        &mut self,
        event: Event,
        bounds: Rectangle,
        cursor: Cursor,
    ) -> (event::Status, Option<Message>) {

        self.bounds = Some(bounds);

        if let Some(p_raw) = cursor.position_in(&bounds) {
            let p = self.transform().transform_point(Point::new(p_raw.x, p_raw.y));
            let spatial_cursor = Cursor::Available(iced::Point::new(p.x, p.y));

            let (cache, status, message) = self.program.update(event, spatial_cursor);
            if cache == SpatialCache::Refresh {
                self.cache.clear();
            }

            if status == event::Status::Captured {
                return (status, message);
            }

            if message.is_some() {
                // TODO(MXG): This implementation only allows us to generate
                // one message per update. It would be better if the
                // canvas::Program::update pub trait function could return a
                // command instead of just an Option<Message>
                return (status, message);
            }

            if let Some(pan_toggler) = &mut self.pan_toggler {
                match pan_toggler.toggle(event) {
                    Toggle::On => {
                        if self.drag_start_point.is_none() {
                            self.drag_start_point = Some(p);
                        }
                        return (event::Status::Captured, None);
                    },
                    Toggle::Off => {
                        self.drag_start_point = None;
                        return (event::Status::Captured, None);
                    },
                    Toggle::NoChange => {
                        // Do nothing
                    }
                }
            }

            if let Event::Mouse(mouse::Event::CursorMoved{..}) = event {
                if let Some(p0) = self.drag_start_point {
                    let p_raw = Point::new(p_raw.x, p_raw.y);
                    self.offset = p_raw - Transform::scale(self.zoom, -self.zoom).transform_point(p0);

                    self.cache.clear();
                    return (event::Status::Captured, None);
                }
            }

            if self.scroll_to_zoom {
                if let Event::Mouse(mouse::Event::WheelScrolled{delta}) = event {
                    match delta {
                        mouse::ScrollDelta::Lines{y, ..} | mouse::ScrollDelta::Pixels{y, ..} => {
                            let p0 = p;
                            self.zoom = self.zoom*(1.0 + y/10.0);
                            let p_raw = Point::new(p_raw.x, p_raw.y);
                            self.offset = p_raw - Transform::scale(self.zoom, -self.zoom).transform_point(p0);
                            self.cache.clear();
                        }
                    }
                }
            }
        } else {
            // Release the canvas drag if the cursor has left the panel
            self.drag_start_point = None;
            let spatial_cursor = Cursor::Unavailable;

            let (cache, status, message) = self.program.update(event, spatial_cursor);
            if cache == SpatialCache::Refresh {
                self.cache.clear();
            }

            if status == event::Status::Captured {
                return (status, message);
            }

            if let Some(message) = message {
                // TODO(MXG): This implementation only allows us to generate
                // one message per update. It would be better if the
                // canvas::Program::update pub trait function could return a
                // command instead of just an Option<Message>
                return (status, Some(message));
            }
        }

        return (event::Status::Ignored, None);
    }

    fn draw(&self, bounds: Rectangle, mut cursor: Cursor) -> Vec<Geometry> {
        if let Some(p_raw) = cursor.position_in(&bounds) {
            let p = self.transform().transform_point(Point::new(p_raw.x, p_raw.y));
            cursor = Cursor::Available(iced::Point::new(p.x, p.y));
        }

        let tf = self.transform();
        let origin = tf.transform_point(Point::origin());
        let corner = tf.transform_point(Point::origin() + Vector::new(bounds.width, bounds.height));
        let spatial_bounds = Rectangle{
            x: origin.x,
            y: corner.y,
            width: (corner.x - origin.x).abs(),
            height: (corner.y - origin.y).abs(),
        };

        let background = self.cache.draw(bounds.size(), |frame| {
            let background = Path::rectangle(iced::Point::ORIGIN, frame.size());
            frame.fill(&background, iced::Color::from_rgb8(240, 240, 240));

            frame.with_save(|frame| {
                frame.translate(iced::Vector::new(self.offset.x, self.offset.y));
                frame.asymmetric_scale(self.zoom, -self.zoom);

                self.program.draw_in_space(frame, spatial_bounds, cursor);
                self.program.draw_on_hud(
                    &mut SpatialHUD{
                        base_frame: frame,
                        zoom: self.zoom,
                    },
                    spatial_bounds,
                    cursor,
                );
            });
        });

        return vec![background];
    }

    fn mouse_interaction(&self, _: Rectangle, _: Cursor) -> mouse::Interaction {
        if let Some(_) = self.drag_start_point {
            return mouse::Interaction::Grabbing;
        }

        mouse::Interaction::default()
    }
}

#[macro_export]
macro_rules! spatial_layers {
    ($name:ident<$message:ident>: $($types:ident),+) => {
        #[derive(Debug)]
        struct $name {
            layers: ($($types<$message>),+)
        }

        impl mapf_viz::spatial_canvas::SpatialCanvasProgram<$message> for $name {

            #[allow(non_snake_case, unused_assignments)]
            fn update(
                &mut self,
                event: iced::canvas::event::Event,
                cursor: iced::canvas::Cursor,
            ) -> (mapf_viz::spatial_canvas::SpatialCache, iced::canvas::event::Status, Option<$message>) {
                let mut refresh = mapf_viz::spatial_canvas::SpatialCache::Unchanged;
                let ($(ref mut $types,)+) = &mut self.layers;
                $(
                    let (cache, status, message) = $types.update(event, cursor);
                    if (cache == mapf_viz::spatial_canvas::SpatialCache::Refresh) {
                        refresh = cache;
                    }

                    if status == iced::canvas::event::Status::Captured {
                        return (refresh, status, message);
                    }

                    if message.is_some() {
                        return (refresh, status, message);
                    }
                )+

                return (refresh, iced::canvas::event::Status::Ignored, None);
            }

            #[allow(non_snake_case, unused_assignments)]
            fn draw_in_space(
                &self,
                frame: &mut iced::canvas::Frame,
                spatial_bounds: iced::Rectangle,
                spatial_cursor: iced::canvas::Cursor,
            ) {
                let ($(ref $types,)+) = &self.layers;
                $(
                    $types.draw_in_space(frame, spatial_bounds, spatial_cursor);
                )+
            }

            #[allow(non_snake_case, unused_assignments)]
            fn draw_on_hud<'a, 'b: 'a>(
                &self,
                hud: &'a mut mapf_viz::spatial_canvas::SpatialHUD<'b>,
                spatial_bounds: iced::Rectangle,
                spatial_cursor: iced::canvas::Cursor,
            ) {
                let ($(ref $types,)+) = &self.layers;
                $(
                    $types.draw_on_hud(&mut *hud, spatial_bounds, spatial_cursor);
                )+
            }

            #[allow(non_snake_case, unused_assignments)]
            fn estimate_bounds(&self) -> mapf_viz::spatial_canvas::InclusionZone {
                let mut bounds = mapf_viz::spatial_canvas::InclusionZone::Empty;
                let ($(ref $types,)+) = &self.layers;
                $(
                    bounds.merge($types.estimate_bounds());
                )+
                return bounds;
            }
        }
    };
}
