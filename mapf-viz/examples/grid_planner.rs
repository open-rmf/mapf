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
#![feature(array_windows)]

use iced::{
    Application, Alignment, Element, Length, Command, Column, Row, Text, Container,
    slider::{self, Slider},
    scrollable::{self, Scrollable},
    text_input::{self, TextInput},
    button::{self, Button},
    canvas::{self, Event, Path, Stroke},
    executor, keyboard, mouse,
};
use iced_native;
use nalgebra::Vector2;
use mapf::{
    Planner,
    motion::{
        Trajectory,
        se2::timed_position::Waypoint,
    },
    directed::{line_follow_se2, simple},
    occupancy::{Grid, SparseGrid, Cell, CornerStatus, Point, Vector}
};
use mapf_viz::{
    SparseGridOccupancyVisual, InfiniteGrid,
    spatial_canvas::{SpatialCanvas, SpatialCanvasProgram, SpatialCache, InclusionZone},
    toggle::{Toggle, Toggler, KeyToggler},
};
use mapf_viz::spatial_layers;
use std::collections::{HashSet, HashMap};
use arrayvec::ArrayVec;

type Visibility = mapf::occupancy::Visibility<SparseGrid>;

#[derive(Debug, Clone)]
struct EndpointSelector<Message> {
    pub agent_radius: f64,
    pub cell_size: f64,
    pub invalid_color: iced::Color,
    pub start_color: iced::Color,
    pub goal_color: iced::Color,
    pub start_cell: Option<Cell>,
    pub start_angle: Option<f64>,
    pub start_valid: bool,
    pub goal_cell: Option<Cell>,
    pub goal_angle: Option<f64>,
    pub goal_valid: bool,
    start_visibility: Vec<Cell>,
    goal_visibility: Vec<Cell>,
    start_sees_goal: bool,
    pressed: bool,
    shift: KeyToggler,
    _msg: std::marker::PhantomData<Message>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Endpoint {
    Start,
    Goal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EndpointSelection {
    Cell(Endpoint),
    Orientation(Endpoint),
    Finalize(Endpoint),
}

impl<Message> EndpointSelector<Message> {

    fn new(
        agent_radius: f64,
        cell_size: f64,
        invalid_color: iced::Color,
        start_color: iced::Color,
        goal_color: iced::Color,
    ) -> Self {
        Self{
            agent_radius,
            cell_size,
            invalid_color,
            start_color,
            goal_color,
            start_cell: None,
            start_angle: None,
            start_valid: true,
            goal_cell: None,
            goal_angle: None,
            goal_valid: true,
            start_visibility: Vec::new(),
            goal_visibility: Vec::new(),
            start_sees_goal: false,
            pressed: false,
            shift: KeyToggler::for_key(keyboard::KeyCode::LShift),
            _msg: Default::default(),
        }
    }

    fn endpoint_cell(&self, choice: Endpoint) -> &Option<Cell> {
        match choice {
            Endpoint::Start => &self.start_cell,
            Endpoint::Goal => &self.goal_cell,
        }
    }

    fn endpoint_cell_mut(&mut self, choice: Endpoint) -> &mut Option<Cell> {
        match choice {
            Endpoint::Start => &mut self.start_cell,
            Endpoint::Goal => &mut self.goal_cell,
        }
    }

    fn set_endpoint_color(&mut self, choice: Endpoint, color: iced::Color) {
        match choice {
            Endpoint::Start => { self.start_color = color; },
            Endpoint::Goal => { self.goal_color = color; },
        }
    }

    fn set_endpoint_valid(&mut self, choice: Endpoint, valid: bool) {
        match choice {
            Endpoint::Start => { self.start_valid = valid; },
            Endpoint::Goal => { self.goal_valid = valid; },
        }
    }

    pub fn calculate_visibility(&mut self, visibility: &Visibility) {
        self.start_visibility = self.calculate_endpoint_visibility(
            self.start_cell, self.start_valid, visibility
        );

        self.goal_visibility = self.calculate_endpoint_visibility(
            self.goal_cell, self.goal_valid, visibility
        );

        self.start_sees_goal = false;
        if self.start_valid && self.goal_valid {
            if let (Some(cell_start), Some(cell_goal)) = (self.start_cell, self.goal_cell) {
                let p_start = cell_start.to_center_point(self.cell_size);
                let p_goal = cell_goal.to_center_point(self.cell_size);
                self.start_sees_goal = visibility.grid().is_sweep_occupied(
                    p_start, p_goal, 2.0*self.agent_radius
                ).is_none();
            }
        }
    }

    fn calculate_endpoint_visibility(
        &self,
        cell_opt: Option<Cell>,
        valid: bool,
        visibility: &Visibility,
    ) -> Vec<Cell> {
        if valid {
            if let Some(cell) = cell_opt {
                return visibility.calculate_visibility(cell).map(|c| *c).collect();
            }
        }

        return Vec::new();
    }
}

impl SpatialCanvasProgram<Message> for EndpointSelector<Message> {
    fn update(
        &mut self,
        event: iced::canvas::Event,
        cursor: iced::canvas::Cursor
    ) -> (SpatialCache, iced::canvas::event::Status, Option<Message>) {
        if let Some(p) = cursor.position() {
            match event {
                Event::Mouse(event) => {
                    if mouse::Event::ButtonPressed(mouse::Button::Right) == event {
                        self.pressed = true;
                        let endpoint = if self.shift.state() == Toggle::On { Endpoint::Goal } else { Endpoint::Start };
                        *self.endpoint_cell_mut(endpoint) = Some(
                            Cell::from_point([p.x as f64, p.y as f64].into(), self.cell_size)
                        );

                        return (
                            SpatialCache::Refresh,
                            canvas::event::Status::Captured,
                            Some(Message::EndpointSelected(
                                EndpointSelection::Cell(endpoint)
                            ))
                        );
                    }

                    if mouse::Event::ButtonReleased(mouse::Button::Right) == event {
                        self.pressed = false;
                        let endpoint = if self.shift.state() == Toggle::On { Endpoint::Goal } else { Endpoint::Start };
                        return (
                            SpatialCache::Unchanged,
                            canvas::event::Status::Captured,
                            Some(Message::EndpointSelected(
                                EndpointSelection::Finalize(endpoint)
                            ))
                        );
                    }
                },
                Event::Keyboard(event) => {
                    self.shift.key_toggle(event);
                }
            }
        }

        return (SpatialCache::Unchanged, canvas::event::Status::Ignored, None);
    }

    fn draw_in_space(&self, frame: &mut canvas::Frame, _spatial_bounds: iced::Rectangle, _spatial_cursor: canvas::Cursor) {
        for (cell, angle, color, valid) in [
            (self.start_cell, self.start_angle, self.start_color, self.start_valid),
            (self.goal_cell, self.goal_angle, self.goal_color, self.goal_valid)
        ] {
            let radius = self.agent_radius as f32;
            if let Some(cell) = cell {
                let p = cell.to_center_point(self.cell_size);
                let circle = Path::circle([p.x as f32, p.y as f32].into(), radius);
                frame.fill(&circle, color);

                if !valid {
                    frame.stroke(
                        &circle,
                        Stroke{
                            color: self.invalid_color,
                            width: 5_f32,
                            ..Default::default()
                        },
                    );
                }
            }

            for (cell_opt, visible, color) in [
                (self.start_cell, &self.start_visibility, self.start_color),
                (self.goal_cell, &self.goal_visibility, self.goal_color)
            ] {
                if let Some(cell) = cell_opt {
                    let p = cell.to_center_point(self.cell_size);
                    for v_cell in visible {
                        let p_v = v_cell.to_center_point(self.cell_size);
                        frame.stroke(
                            &Path::line(
                                [p.x as f32, p.y as f32].into(),
                                [p_v.x as f32, p_v.y as f32].into()
                            ),
                            Stroke{
                                color,
                                width: 5_f32,
                                ..Default::default()
                            }
                        );
                    }
                }
            }

            if self.start_sees_goal {
                if let (Some(cell_s), Some(cell_g)) = (self.start_cell, self.goal_cell) {
                    let p_start = cell_s.to_center_point(self.cell_size);
                    let p_goal = cell_g.to_center_point(self.cell_size);
                    frame.stroke(
                        &Path::line(
                            [p_start.x as f32, p_start.y as f32].into(),
                            [p_goal.x as f32, p_goal.y as f32].into()
                        ),
                        Stroke{
                            color: iced::Color::from_rgb(0.1, 1.0, 1.0),
                            width: 5_f32,
                            ..Default::default()
                        }
                    );
                }
            }
        }
    }

    fn estimate_bounds(&self) -> mapf_viz::spatial_canvas::InclusionZone {
        let mut zone = InclusionZone::Empty;
        for endpoint in [self.start_cell, self.goal_cell] {
            if let Some(cell) = endpoint {
                let p = cell.to_center_point(self.cell_size);
                for v in [[1.0, 1.0], [-1.0, -1.0]] {
                    let v: Vector = v.into();
                    let r = p + self.agent_radius*v;
                    zone.include([r.x as f32, r.y as f32].into());
                }
            }
        }

        return zone;
    }
}

#[derive(Debug, Clone)]
struct SolutionVisual<Message> {
    pub path_color: iced::Color,
    pub solution: Option<Trajectory<Waypoint>>,
    _msg: std::marker::PhantomData<Message>,
}

impl SolutionVisual<Message> {
    fn new(path_color: iced::Color) -> Self {
        Self{
            path_color,
            solution: None,
            _msg: Default::default(),
        }
    }
}

impl SpatialCanvasProgram<Message> for SolutionVisual<Message> {
    fn draw_in_space(
        &self,
        frame: &mut canvas::Frame,
        _spatial_bounds: iced::Rectangle,
        _spatial_cursor: canvas::Cursor
    ) {
        if let Some(trajectory) = &self.solution {
            for [wp0, wp1] in trajectory.array_windows() {
                let p0 = wp0.position.translation;
                let p1 = wp1.position.translation;
                frame.stroke(
                    &Path::line(
                        [p0.x as f32, p0.y as f32].into(),
                        [p1.x as f32, p1.y as f32].into(),
                    ),
                    Stroke{
                        color: self.path_color,
                        width: 4_f32,
                        ..Default::default()
                    }
                );
            }
        }
    }

    fn estimate_bounds(&self) -> InclusionZone {
        // This layer should always be contained within the other layers,
        // so we don't need to provide any bounds for this.
        return InclusionZone::Empty;
    }
}

spatial_layers!(GridLayers<Message>: InfiniteGrid, SparseGridOccupancyVisual, EndpointSelector, SolutionVisual);

struct App {
    robot_size_slider: slider::State,
    max_robot_radius: f32,
    input: text_input::State,
    input_value: String,
    reset_size_button: button::State,
    canvas: SpatialCanvas<Message, GridLayers>,
    scroll: scrollable::State,
    show_details: KeyToggler,
    debug_text: String,
    debug_corners: HashSet<Cell>,
}

impl App {
    const TICK_COUNT: u32 = 1000;

    fn set_debug_text(&mut self) {
        self.debug_text.clear();
        let occupancy = &self.canvas.program.layers.1;
        let mut corner_info_map: HashMap<Cell, (Option<Cell>, CornerStatus)> = HashMap::new();
        for (corner_cell, info) in occupancy.visibility().unstable().points() {
            if self.debug_corners.contains(corner_cell) {
                corner_info_map.insert(*corner_cell, *info);
            }
        }

        let mut edge_info_map: HashMap<Cell, HashMap<Cell, Option<Cell>>> = HashMap::new();
        let mut reverse_edge_info_map: HashMap<Cell, HashMap<Cell, Option<Cell>>> = HashMap::new();
        for (cell, info) in occupancy.visibility().unstable().edges() {
            if self.debug_corners.contains(cell) {
                edge_info_map.insert(*cell, info.clone());
            } else {
                for (other, _) in info {
                    if self.debug_corners.contains(other) {
                        reverse_edge_info_map.insert(*cell, info.clone());
                    }
                }
            }
        }

        self.debug_text = format!(
            "Point info:\n{:#?}\n\nEdge Info:\n{:#?}\n\nReverse Edge Info:\n{:#?}",
            corner_info_map,
            edge_info_map,
            reverse_edge_info_map,
        );
    }

    fn default_invalid_color() -> iced::Color {
        iced::Color::from_rgb(1.0, 0.0, 0.0)
    }

    fn default_endpoint_color(endpoint: Endpoint) -> iced::Color {
        match endpoint {
            Endpoint::Start => iced::Color::from_rgba(0.1, 0.8, 0.1, 0.9),
            Endpoint::Goal => iced::Color::from_rgba(0.1, 0.1, 1.0, 0.9),
        }
    }

    fn default_solution_color() -> iced::Color {
        iced::Color::from_rgb(1.0, 1.0, 1.0)
    }

    fn visibility(&self) -> &Visibility {
        self.canvas.program.layers.1.visibility()
    }

    fn grid(&self) -> &SparseGrid {
        self.canvas.program.layers.1.grid()
    }

    fn endpoint_selector(&self) -> &EndpointSelector<Message> {
        &self.canvas.program.layers.2
    }

    fn endpoint_selector_mut(&mut self) -> &mut EndpointSelector<Message> {
        &mut self.canvas.program.layers.2
    }

    fn is_valid_endpoint(&self, endpoint: Endpoint) -> bool {
        if let Some(cell) = self.endpoint_selector().endpoint_cell(endpoint) {
            let p = cell.to_center_point(self.grid().cell_size());
            if self.grid().is_square_occupied(p, 2.0*self.visibility().agent_radius()).is_some() {
                return false;
            } else {
                return true;
            }
        }

        return false;
    }

    fn set_robot_radius(&mut self, radius: f32) {
        self.canvas.program.layers.1.set_robot_radius(radius);
        let endpoint_selector = &mut self.canvas.program.layers.2;
        endpoint_selector.agent_radius = radius as f64;
        for endpoint in [Endpoint::Start, Endpoint::Goal] {
            if let Some(cell) = endpoint_selector.endpoint_cell(endpoint) {
                let p = cell.to_center_point(endpoint_selector.cell_size);
                let valid = self.canvas.program.layers.1.grid().is_square_occupied(p, 2.0*endpoint_selector.agent_radius).is_none();
                endpoint_selector.set_endpoint_valid(endpoint, valid);
            }
        }
        self.recalculate_visibility();
        self.canvas.cache.clear();
    }

    fn recalculate_visibility(&mut self) {
        self.canvas.program.layers.2.calculate_visibility(self.canvas.program.layers.1.visibility());
    }

    fn generate_plan(&mut self) {
        self.canvas.program.layers.3.solution = None;
        let endpoints = &self.canvas.program.layers.2;
        let visibility = self.canvas.program.layers.1.visibility();
        let cell_size = endpoints.cell_size;
        if let (Some(start_cell), Some(goal_cell)) = (endpoints.start_cell, endpoints.goal_cell) {
            if start_cell == goal_cell {
                // No plan is needed
                return;
            }

            if endpoints.start_valid && endpoints.goal_valid {
                let mut vertices = Vec::new();
                let mut edges: Vec<Vec<usize>> = Vec::new();
                let mut lookup = HashMap::new();
                vertices.push(start_cell.to_center_point(cell_size));
                vertices.push(goal_cell.to_center_point(cell_size));

                for (cell_i, cell_j) in visibility.iter_edges() {
                    let indices: ArrayVec<[usize; 2]> = [cell_i, cell_j]
                        .iter().map(
                            |cell| {
                                lookup.entry(**cell).or_insert_with(
                                    || {
                                        let index = vertices.len();
                                        vertices.push(cell.to_center_point(cell_size));
                                        index
                                });
                            }).collect();

                    let min_size = i.max(j) + 1;
                    if edges.len() < min_size {
                        edges.resize(min_size, Vec::new());
                    }

                    edges.get_mut(i).unwrap().push(j);
                    edges.get_mut(j).unwrap().push(i);
                }
            }
        }
    }
}

impl Application for App {
    type Message = Message;
    type Executor = executor::Default;
    type Flags = ();

    fn new(_flags: Self::Flags) -> (Self, Command<Self::Message>) {
        let cell_size = 1.0_f32;
        let robot_radius = 0.75_f32;

        let mut canvas = SpatialCanvas::new(
            GridLayers{
                layers: (
                    InfiniteGrid::new(cell_size),
                    SparseGridOccupancyVisual::new(
                        SparseGrid::new(cell_size as f64),
                        robot_radius,
                        None,
                        Some(Box::new(|| { Message::OccupancyChanged })),
                    ),
                    EndpointSelector::new(
                        robot_radius as f64,
                        cell_size as f64,
                        Self::default_invalid_color(),
                        Self::default_endpoint_color(Endpoint::Start),
                        Self::default_endpoint_color(Endpoint::Goal),
                    ),
                    SolutionVisual::new(Self::default_solution_color()),
                )
            }
        );
        canvas.zoom = 20.0;

        (
            Self{
                robot_size_slider: slider::State::new(),
                max_robot_radius: 10_f32*cell_size,
                input: text_input::State::default(),
                input_value: String::new(),
                reset_size_button: button::State::new(),
                canvas,
                scroll: scrollable::State::new(),
                show_details: KeyToggler::for_key(keyboard::KeyCode::LAlt),
                debug_text: Default::default(),
                debug_corners: Default::default(),
            },
            Command::none()
        )
    }

    fn title(&self) -> String {
        "Test App".to_owned()
    }

    fn update(
        &mut self,
        message: Self::Message
    ) -> Command<Self::Message> {
        match message {
            Message::TextInputChanged(value) => {
                self.input_value = value;
                if let Ok(radius) = self.input_value.parse::<f32>() {
                    if radius > 0.0 {
                        self.max_robot_radius = radius;

                        let current_robot_radius = self.canvas.program.layers.1.visibility().agent_radius() as f32;
                        if self.max_robot_radius < current_robot_radius {
                            self.set_robot_radius(radius);
                        }
                    }
                }
            },
            Message::RobotSizeSlide(value) => {
                let min_size = self.canvas.program.layers.1.grid().cell_size() as f32/10_f32;
                let new_robot_radius = value as f32 * (self.max_robot_radius - min_size)/(Self::TICK_COUNT as f32) + min_size;
                self.set_robot_radius(new_robot_radius);
            },
            Message::ResetView => {
                self.canvas.fit_to_bounds();
            },
            Message::EventOccurred(event) => {
                if let iced_native::Event::Keyboard(event) = event {
                    if let iced_native::keyboard::Event::KeyPressed{key_code, modifiers} = event {
                        if keyboard::KeyCode::D == key_code {
                            if modifiers.shift() {
                                self.debug_text.clear();
                            } else {
                                self.debug_text = format!(
                                    "Debug visibility:\n{:#?}",
                                    self.canvas.program.layers.1.visibility(),
                                );
                            }
                        }
                    }

                    match self.show_details.key_toggle(event) {
                        Toggle::On => {
                            self.canvas.program.layers.1.show_details(true);
                            self.canvas.cache.clear();
                        },
                        Toggle::Off => {
                            self.canvas.program.layers.1.show_details(false);
                            self.canvas.cache.clear();
                        },
                        Toggle::NoChange => {
                            // Do nothing
                        }
                    }
                }
            },
            Message::EndpointSelected(selection) => {
                match selection {
                    EndpointSelection::Cell(endpoint) => {
                        let valid = self.is_valid_endpoint(endpoint);
                        self.endpoint_selector_mut().set_endpoint_valid(
                            endpoint, valid
                        );
                        self.recalculate_visibility();
                    },
                    EndpointSelection::Orientation(_) => {
                        // Do nothing
                    },
                    EndpointSelection::Finalize(_) => {
                        self.generate_plan();
                    }
                }
            },
            Message::OccupancyChanged => {
                for endpoint in [Endpoint::Start, Endpoint::Goal] {
                    let cell_opt = self.canvas.program.layers.2.endpoint_cell(endpoint);
                    if let Some(cell) = cell_opt {
                        let p = cell.to_center_point(self.grid().cell_size());
                        let valid = self.grid().is_square_occupied(p, 2.0*self.visibility().agent_radius()).is_none();
                        self.endpoint_selector_mut().set_endpoint_valid(endpoint, valid);
                        self.recalculate_visibility();
                    }
                }
            }
            Message::Tick => {

            }
        }

        Command::none()
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        iced_native::Subscription::batch([
            iced_native::subscription::events().map(Message::EventOccurred),
            iced::time::every(std::time::Duration::from_millis(100)).map(|_|{ Message::Tick }),
        ])
    }

    fn view(&mut self) -> Element<Self::Message> {
        let mut content = Column::new()
            .spacing(20)
            .align_items(Alignment::Start)
            .width(Length::Fill)
            .height(Length::Fill);

        let min_size = self.canvas.program.layers.1.grid().cell_size() as f32/10_f32;
        let robot_radius = self.canvas.program.layers.1.visibility().agent_radius() as f32;
        let robot_tick = ((robot_radius - min_size)/(self.max_robot_radius - min_size) * Self::TICK_COUNT as f32) as u32;
        let file_row = Row::new()
            .spacing(20)
            .align_items(Alignment::Center)
            .width(Length::Fill)
            .push(
                Button::new(
                    &mut self.reset_size_button,
                    iced::Text::new("Reset View")
                ).on_press(Message::ResetView)
            )
            .push(
                Slider::new(
                    &mut self.robot_size_slider,
                    0..=Self::TICK_COUNT,
                    robot_tick,
                    Message::RobotSizeSlide,
                )
            )
            .push(Text::new(format!("{:.2}", robot_radius)))
            .push(
                TextInput::new(
                    &mut self.input,
                    "Max Robot Radius",
                    &mut self.input_value,
                    Message::TextInputChanged
                )
                .padding(10)
                .width(Length::Fill)
            );

        let instruction_row = Row::<Message>::new()
            .spacing(40)
            .align_items(Alignment::Start)
            .width(Length::Shrink)
            .push(Text::new("Left click: Add occupancy"))
            .push(Text::new("Shift + Left click: Remove occupancy"))
            .push(Text::new("Middle click: Pan view"))
            .push(Text::new("Scroll: Zoom"));

        content = content
            .push(file_row)
            .push(instruction_row);

        if self.debug_text.is_empty() {
            content = content.push(self.canvas.view());
        } else {

            content = content.push(
                Row::new()
                .push(self.canvas.view())
                .push(Scrollable::new(&mut self.scroll).push(
                    Text::new(&self.debug_text)
                    .size(13)
                ))
            );
        }

        Container::new(content)
            .width(Length::Fill)
            .height(Length::Fill)
            .padding(5)
            .center_x()
            .center_y()
            .into()
    }
}

#[derive(Debug, Clone)]
enum Message {
    TextInputChanged(String),
    RobotSizeSlide(u32),
    ResetView,
    EventOccurred(iced_native::Event),
    EndpointSelected(EndpointSelection),
    OccupancyChanged,
    Tick,
}

fn main() -> iced::Result {
    App::run(iced::Settings::default())
}

mod style {
    use iced::container;
    pub struct Pane;
    impl container::StyleSheet for Pane {
        fn style(&self) -> container::Style {
            container::Style{
                text_color: None,
                background: None,
                border_radius: 0.0,
                border_width: 3.0,
                border_color: iced::Color{r: 0.9, g: 0.9, b: 0.9, a: 0.9},
            }
        }
    }
}
