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
#![feature(array_windows, binary_heap_into_iter_sorted)]

use time_point::{TimePoint, Duration};
use iced::{
    Application, Alignment, Element, Length, Command, Column, Row, Text, Container, Radio,
    slider::{self, Slider},
    scrollable::{self, Scrollable},
    text_input::{self, TextInput},
    button::{self, Button},
    canvas::{self, Event, Path, Stroke},
    executor, keyboard, mouse,
};
use iced_native;
use mapf::{
    graph::{
        SharedGraph,
        occupancy::{
            Visibility, VisibilityGraph, SparseGrid, Cell, NeighborhoodGraph, Grid
        },
    },
    motion::{
        Trajectory, Motion, OverlayedDynamicEnvironment, DynamicEnvironment,
        CircularProfile, DynamicCircularObstacle, TravelEffortCost,
        se2::{
            Point, LinearTrajectorySE2, Vector, WaypointSE2, GoalSE2,
            DifferentialDriveLineFollow, StartSE2, Orientation,
        },
    },
    templates::InformedSearch,
    planner::{Planner, Search},
    premade::SippSE2,
    algorithm::{AStarConnect, SearchStatus, tree::NodeContainer},
};
use mapf_viz::{
    SparseGridOccupancyVisual, InfiniteGrid,
    spatial_canvas::{SpatialCanvas, SpatialCanvasProgram, SpatialCache, InclusionZone},
    toggle::{Toggle, Toggler, KeyToggler},
};
use mapf_viz::spatial_layers;
use std::collections::HashMap;
use std::sync::Arc;

type SparseVisibility = Visibility<SparseGrid>;

pub(crate) struct Minimum<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> {
    value: Option<T>,
    f: F,
}

impl<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> Minimum<T, F> {
    pub(crate) fn new(f: F) -> Self {
        Self{value: None, f}
    }

    pub(crate) fn consider(&mut self, other: &T) -> bool {
        if let Some(value) = &self.value {
            if std::cmp::Ordering::Less == (self.f)(other, value) {
                self.value = Some(other.clone());
                return true;
            }
        } else {
            self.value = Some(other.clone());
            return true;
        }

        return false;
    }

    pub(crate) fn consider_take(&mut self, other: T) -> bool {
        if let Some(value) = &self.value {
            if std::cmp::Ordering::Less == (self.f)(&other, value) {
                self.value = Some(other);
                return true;
            }
        } else {
            self.value = Some(other);
            return true;
        }

        return false;
    }

    pub(crate) fn result(self) -> Option<T> {
        self.value
    }

    pub(crate) fn has_value(&self) -> bool {
        self.value.is_some()
    }
}


fn draw_agent(
    frame: &mut canvas::Frame,
    point: Point,
    angle: Option<f64>,
    agent_radius: f32,
    color: iced::Color,
) {
    frame.with_save(
        |frame| {
            frame.translate([point.x as f32, point.y as f32].into());
            frame.fill(&Path::circle([0, 0].into(), agent_radius), color);

            if let Some(angle) = angle {
                frame.rotate(angle as f32);
                let r = 0.8 * agent_radius;
                let angle_to_point = |angle: f32| {
                    iced::Point::new(r * f32::cos(angle), r * f32::sin(angle))
                };

                let points: [iced::Point; 3] = [
                    angle_to_point(-150_f32.to_radians()),
                    angle_to_point(0_f32.to_radians()),
                    angle_to_point(150_f32.to_radians()),
                ];

                frame.fill(
                    &Path::new(
                        |builder| {
                            builder.move_to(points[0]);
                            builder.line_to(points[1]);
                            builder.line_to(points[2]);
                            builder.line_to(points[0]);
                        }
                    ),
                    iced::Color::from_rgb(1.0, 1.0, 1.0)
                );
            }
        }
    );
}

fn draw_trajectory(
    frame: &mut canvas::Frame,
    trajectory: &LinearTrajectorySE2,
    color: iced::Color,
) {
    for [wp0, wp1] in trajectory.array_windows() {
        let p0 = wp0.position.translation;
        let p1 = wp1.position.translation;
        frame.stroke(
            &Path::line(
                [p0.x as f32, p0.y as f32].into(),
                [p1.x as f32, p1.y as f32].into(),
            ),
            Stroke{
                color,
                width: 3_f32,
                ..Default::default()
            }
        );
    }
}

#[derive(Debug, Clone)]
struct EndpointSelector<Message> {
    pub agent_radius: f64,
    pub cell_size: f64,
    pub invalid_color: iced::Color,
    pub start_color: iced::Color,
    pub goal_color: iced::Color,
    pub show_details: bool,
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
            show_details: false,
            // start_cell: None,
            // start_cell: Some(Cell::new(-7, -4)),
            // start_cell: Some(Cell::new(0, 0)),
            start_cell: Some(Cell::new(-1, -13)),
            start_angle: None,
            start_valid: true,
            // goal_cell: None,
            // goal_cell: Some(Cell::new(18, 3)),
            // goal_cell: Some(Cell::new(10, 0)),
            goal_cell: Some(Cell::new(-1, 6)),
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

    fn set_endpoint_valid(&mut self, choice: Endpoint, valid: bool) {
        match choice {
            Endpoint::Start => { self.start_valid = valid; },
            Endpoint::Goal => { self.goal_valid = valid; },
        }
    }

    pub fn calculate_visibility(&mut self, visibility: &SparseVisibility) {
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
        visibility: &SparseVisibility,
    ) -> Vec<Cell> {
        if valid {
            if let Some(cell) = cell_opt {
                return visibility.calculate_visibility(cell).map(|c| c).collect();
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
                draw_agent(frame, p, angle, self.agent_radius as f32, color);
                if !valid {
                    frame.stroke(
                        &Path::circle([p.x as f32, p.y as f32].into(), radius),
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

    fn draw_on_hud<'a, 'b: 'a>(
        &self,
        hud: &'a mut mapf_viz::spatial_canvas::SpatialHUD<'b>,
        bounds: iced::Rectangle,
        _cursor: canvas::Cursor
    ) {
        if self.show_details {
            for cell_opt in [self.start_cell, self.goal_cell] {
                if let Some(cell) = cell_opt {
                    let p = cell.to_center_point(self.cell_size);
                    let r = self.agent_radius as f32;
                    let delta = iced::Vector::new(r, -r);
                    let p = iced::Point::new(p.x as f32, p.y as f32) + delta;

                    if bounds.contains(p) {
                        hud.at(
                            p,
                            |frame| {
                                frame.fill_text(format!("({}, {})", cell.x, cell.y).to_string());
                            }
                        );
                    }
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
    pub cell_size: f64,
    pub agent_radius: f32,
    pub solution_color: iced::Color,
    pub solution: Option<LinearTrajectorySE2>,
    pub search_color: iced::Color,
    pub searches: Vec<LinearTrajectorySE2>,
    pub obstacles: Vec<(f64, LinearTrajectorySE2)>,
    pub vertex_lookup: HashMap<Cell, usize>,
    tick_start: Option<TimePoint>,
    now: Option<Duration>,
    _msg: std::marker::PhantomData<Message>,
}

impl SolutionVisual<Message> {
    fn new(
        cell_size: f64,
        agent_radius: f32,
        solution_color: iced::Color,
        search_color: iced::Color,
    ) -> Self {

        let t_obs = Trajectory::from_iter([
            WaypointSE2::new(TimePoint::from_secs_f64(0.0), 10.0, 0.0, 180_f64.to_radians()),
            // WaypointSE2::new_f64(10.0, -5.0, 0.0, 180_f64.to_radians()),
            WaypointSE2::new_f64(5.0, 0.0, 0.0, 180_f64.to_radians()),
            WaypointSE2::new_f64(8.0, 0.0, 0.0, -90_f64.to_radians()),
            WaypointSE2::new_f64(18.0, 0.0, -10.0, -90_f64.to_radians()),
        ]).unwrap();

        Self{
            cell_size,
            agent_radius,
            solution_color,
            solution: None,
            search_color,
            searches: Vec::new(),
            obstacles: vec![(1.0, t_obs)],
            // obstacles: Vec::new(),
            vertex_lookup: HashMap::new(),
            tick_start: None,
            now: None,
            _msg: Default::default(),
        }
    }

    fn reset_time(&mut self) {
        self.tick_start = Some(TimePoint::from_std_instant(std::time::Instant::now()));
        self.now = Some(Duration::zero());
    }

    fn time_range(&self) -> Option<(TimePoint, TimePoint)> {
        let mut earliest = Minimum::new(|l: &TimePoint, r: &TimePoint| l.cmp(r));
        let mut latest = Minimum::new(|l: &TimePoint, r: &TimePoint| r.cmp(l));

        for traj in &self.searches {
            earliest.consider(&traj.initial_motion_time());
            latest.consider(&traj.finish_motion_time());
        }

        if !earliest.has_value() || !latest.has_value() {
            if let Some(solution) = &self.solution {
                return Some((solution.initial_motion_time(), solution.finish_motion_time()));
            }

            for (_, obs) in &self.obstacles {
                earliest.consider(&obs.initial_motion_time());
                latest.consider(&obs.finish_motion_time());
            }
        }

        if let (Some(earliest), Some(latest)) = (earliest.result(), latest.result()) {
            return Some((earliest, latest));
        }

        return None;
    }

    fn tick(&mut self) -> bool {

        if let Some((start, end)) = self.time_range() {
            let duration = end - start;
            if let Some(start) = self.tick_start {
                let dt = TimePoint::from_std_instant(std::time::Instant::now()) - start;
                if dt > duration + Duration::from_secs(1) {
                    self.reset_time();
                } else if dt > duration {
                    self.now = Some(duration);
                } else {
                    self.now = Some(dt);
                }

                return true;
            } else {
                self.reset_time();
            }
        }

        return false;
    }
}

impl SpatialCanvasProgram<Message> for SolutionVisual<Message> {
    fn draw_in_space(
        &self,
        frame: &mut canvas::Frame,
        _spatial_bounds: iced::Rectangle,
        _spatial_cursor: canvas::Cursor
    ) {
        if let Some((t0, _)) = self.time_range() {

            for trajectory in &self.searches {
                draw_trajectory(frame, trajectory, self.search_color);
            }

            if let Some(trajectory) = &self.solution {
                draw_trajectory(frame, trajectory, self.solution_color);
            }

            for (r, obs) in &self.obstacles {
                let red = iced::Color::from_rgb(1.0, 0.0, 0.0);
                draw_trajectory(frame, obs, red);

                if let Some(now) = self.now {
                    if let Ok(p) = obs.motion().compute_position(&(t0 + now)) {
                        draw_agent(
                            frame,
                            Point::from(p.translation.vector),
                            Some(p.rotation.angle()),
                            *r as f32,
                            red,
                        );
                    }
                }
            }

            if let Some(now) = self.now {
                for trajectory in &self.searches {
                    if let Ok(p) = trajectory.motion().compute_position(&(t0 + now)) {
                        draw_agent(
                            frame,
                            p.translation.vector.into(),
                            Some(p.rotation.angle()),
                            self.agent_radius,
                            self.search_color,
                        );
                    }
                }

                if let Some(trajectory) = &self.solution {
                    if let Ok(p) = trajectory.motion().compute_position(&(t0 + now)) {
                        draw_agent(
                            frame,
                            Point::from(p.translation.vector),
                            Some(p.rotation.angle()),
                            self.agent_radius,
                            self.solution_color,
                        );
                    }
                }
            }
        }
    }

    fn draw_on_hud<'a, 'b: 'a>(
        &self,
        hud: &'a mut mapf_viz::spatial_canvas::SpatialHUD<'b>,
        bound: iced::Rectangle,
        _spatial_cursor: canvas::Cursor
    ) {
        if let Some(trajectory) = &self.solution {
            let mut sequence: HashMap<Cell, Vec<String>> = HashMap::new();
            for (i, wp) in trajectory.iter().enumerate() {
                sequence.entry(
                    Cell::from_point(
                        Point::from(wp.position.translation.vector), self.cell_size
                    )
                ).or_default().push(i.to_string());
            }

            let r = self.agent_radius / 2_f32.sqrt();
            let delta = iced::Vector::new(r, r);
            for (cell, seq) in sequence {
                let p = cell.to_center_point(self.cell_size);
                let p = iced::Point::new(p.x as f32, p.y as f32) + delta;
                if bound.contains(p) {
                    hud.at(
                        p,
                        |frame| {
                            frame.translate([0_f32, -16_f32].into());
                            frame.fill_text(seq.join(", "));
                        }
                    );
                }
            }
        }

        for (cell, v) in &self.vertex_lookup {
            let p = cell.to_center_point(self.cell_size);
            let p = iced::Point::new(p.x as f32, p.y as f32);
            if bound.contains(p) {
                hud.at(
                    p,
                    |frame| {
                        frame.fill_text(format!("{v}"));
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

type MyAlgo = AStarConnect<SippSE2<NeighborhoodGraph<SparseGrid>, VisibilityGraph<SparseGrid>>>;
type TreeTicket = mapf::algorithm::tree::TreeQueueTicket<mapf::domain::Cost<f64>>;

struct App {
    robot_size_slider: slider::State,
    max_robot_radius: f32,
    input: text_input::State,
    input_value: String,
    reset_size_button: button::State,
    canvas: SpatialCanvas<Message, GridLayers>,
    node_list_scroll: scrollable::State,
    debug_text_scroll: scrollable::State,
    show_details: KeyToggler,
    search: Option<Search<MyAlgo, GoalSE2<Cell>, ()>>,
    step_progress: button::State,
    debug_on: bool,
    memory: Vec<TreeTicket>,
    debug_step_count: u64,
    debug_node_selected: Option<usize>,
}

impl App {
    const TICK_COUNT: u32 = 1000;

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
        iced::Color::from_rgb(1.0, 0.1, 1.0)
    }

    fn default_search_color() -> iced::Color {
        iced::Color::from_rgb8(191, 148, 228)
    }

    fn visibility(&self) -> &SparseVisibility {
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
        self.canvas.program.layers.3.agent_radius = radius;
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

    fn step_progress(&mut self) {
        if let Some(search) = &mut self.search {
            self.debug_step_count += 1;

            self.memory = search.memory().0
                .queue.clone().into_iter_sorted()
                .map(|n| n.0.clone()).collect();

            for ticket in &self.memory {
                if let Some(traj) = search.memory().0
                    .arena.retrace(ticket.node_id).unwrap()
                    .make_trajectory().unwrap()
                {
                    self.canvas.program.layers.3.searches.push(traj);
                }
            }

            if let SearchStatus::Solved(solution) = search.step().unwrap() {
                println!("Solution: {:#?}", solution);
                self.canvas.program.layers.3.solution = solution.make_trajectory().unwrap();
                self.debug_node_selected = None;
                self.search = None;
            } else {
                if let Some(selection) = self.debug_node_selected {
                    if let Some(node) = self.memory.get(selection) {
                        self.canvas.program.layers.3.solution = search
                            .memory().0.arena
                            .retrace(node.node_id)
                            .unwrap()
                            .make_trajectory()
                            .unwrap();
                    }
                }
            }

            self.canvas.cache.clear();
        }
    }

    fn generate_plan(&mut self) {
        self.canvas.program.layers.3.solution = None;
        let endpoints = &self.canvas.program.layers.2;
        let visibility = self.canvas.program.layers.1.visibility();
        if let (Some(start_cell), Some(goal_cell)) = (endpoints.start_cell, endpoints.goal_cell) {
            if start_cell == goal_cell {
                // No plan is needed
                return;
            }

            if endpoints.start_valid && endpoints.goal_valid {
                self.canvas.program.layers.3.vertex_lookup.clear();

                let shared_visibility = Arc::new(visibility.clone());
                let heuristic_graph = SharedGraph::new(VisibilityGraph::new(
                    shared_visibility.clone(), [],
                ));

                let activity_graph = SharedGraph::new(NeighborhoodGraph::new(
                    shared_visibility, [],
                ));

                let extrapolator = DifferentialDriveLineFollow::new(3.0, 1.0).expect("Bad speeds");
                let agent_radius = self.canvas.program.layers.2.agent_radius;
                let profile = CircularProfile::new(
                    agent_radius, agent_radius, agent_radius,
                ).expect("Bad profile sizes");

                let environment = Arc::new(
                    OverlayedDynamicEnvironment::new(
                        Arc::new({
                            let mut env = DynamicEnvironment::new(profile);
                            for (obs_size, obs_traj) in &self.canvas.program.layers.3.obstacles {
                                env.obstacles.push(
                                    DynamicCircularObstacle::new(
                                        CircularProfile::new(*obs_size, 0.0, 0.0).unwrap()
                                    ).with_trajectory(Some(obs_traj.clone()))
                                );
                            }
                            env
                        })
                    )
                );

                let domain = InformedSearch::new_sipp_se2(
                    activity_graph,
                    heuristic_graph,
                    extrapolator,
                    environment,
                    TravelEffortCost::save_one_second_with_detour_up_to(
                        5.0,
                        360_f64.to_radians(),
                    ),
                ).unwrap();

                let start = StartSE2 {
                    time: TimePoint::from_secs_f64(0.0),
                    key: start_cell,
                    orientation: Orientation::new(0_f64),
                };

                let goal = GoalSE2 {
                    key: goal_cell,
                    orientation: None,
                };

                println!("About to plan:\nStart: {start:#?}\nGoal: {goal:#?}");

                let start_time = std::time::Instant::now();
                let planner = Planner::new(AStarConnect(domain));
                let mut search = planner.plan(start, goal).unwrap();

                self.canvas.program.layers.3.searches.clear();
                self.debug_step_count = 0;
                if self.debug_on {
                    self.search = Some(search);
                    self.memory = self.search.as_ref().unwrap().memory().0
                        .queue.clone().into_iter_sorted()
                        .map(|n| n.0.clone()).collect();
                } else {
                    let result = search.solve().unwrap();
                    let elapsed_time = start_time.elapsed();
                    match result {
                        SearchStatus::Solved(solution) => {

                            println!("Solution: {:#?}", solution);
                            println!(" ======================= ");
                            self.canvas.program.layers.3.solution = solution.make_trajectory().unwrap();
                            println!(" ======================= ");
                            println!("Trajectory: {:#?}", self.canvas.program.layers.3.solution);
                            println!(
                                "Arrival time: {:?}, cost: {:?}",
                                self.canvas.program.layers.3.solution.as_ref()
                                    .map(|t| t.motion_duration().as_secs_f64()).unwrap_or(0.0),
                                solution.total_cost.0,
                            );
                            self.canvas.cache.clear();
                        },
                        SearchStatus::Impossible => {
                            println!("Impossible to solve!");
                        },
                        SearchStatus::Incomplete => {
                            println!("Planning is incomplete..?");
                        }
                    }
                    println!("Planning took {:?}s", elapsed_time.as_secs_f64());
                }
                self.debug_node_selected = None;

                return;
            }
        }

        // If the attempt to plan falls through, then clear out all these fields.
        self.memory.clear();
        self.search = None;
        self.canvas.program.layers.3.solution = None;
        self.canvas.program.layers.3.searches.clear();
        self.canvas.cache.clear();
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
                    SolutionVisual::new(
                        cell_size as f64,
                        robot_radius,
                        Self::default_solution_color(),
                        Self::default_search_color(),
                    ),
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
                node_list_scroll: scrollable::State::new(),
                debug_text_scroll: scrollable::State::new(),
                show_details: KeyToggler::for_key(keyboard::KeyCode::LAlt),
                search: None,
                step_progress: button::State::new(),
                debug_on: false,
                memory: Default::default(),
                debug_step_count: 0,
                debug_node_selected: None,
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
                self.generate_plan();
            },
            Message::ResetView => {
                self.canvas.fit_to_bounds();
            },
            Message::EventOccurred(event) => {
                if let iced_native::Event::Keyboard(event) = event {
                    match self.show_details.key_toggle(event) {
                        Toggle::On => {
                            self.canvas.program.layers.1.show_details(true);
                            self.canvas.program.layers.2.show_details = true;
                            self.canvas.cache.clear();
                        },
                        Toggle::Off => {
                            self.canvas.program.layers.1.show_details(false);
                            self.canvas.program.layers.2.show_details = false;
                            self.canvas.cache.clear();
                        },
                        Toggle::NoChange => {
                            // Do nothing
                        }
                    }

                    if let keyboard::Event::KeyPressed{key_code: keyboard::KeyCode::D, modifiers} = event {
                        if modifiers.shift() {
                            self.debug_on = false;
                            self.generate_plan();
                        } else {
                            self.debug_on = true;
                            self.generate_plan();
                        }
                    }

                    if let keyboard::Event::KeyPressed{key_code: keyboard::KeyCode::S, ..} = event {
                        self.step_progress();
                    }

                    if let keyboard::Event::KeyPressed { key_code: keyboard::KeyCode::Down, .. } = event {
                        let next_debug_node = self.debug_node_selected.map(|n| n+1).unwrap_or(0);
                        return Command::perform(async move  {}, move |_| {
                            Message::DebugNodeSelected(next_debug_node)
                        });
                    }

                    if let keyboard::Event::KeyPressed { key_code: keyboard::KeyCode::Up, .. } = event {
                        let next_debug_node = self.debug_node_selected.map(|n| if n > 0 { n-1 } else { 0 }).unwrap_or(0);
                        return Command::perform(async move  {}, move |_| {
                            Message::DebugNodeSelected(next_debug_node)
                        });
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

                self.generate_plan();
            },
            Message::DebugNodeSelected(value) => {
                if self.memory.is_empty() {
                    self.debug_node_selected = Some(0);
                } else {
                    self.debug_node_selected = Some(usize::min(value, self.memory.len() - 1));
                }

                if let Some(node) = self.memory.get(value) {
                    if let Some(search) = &self.search {
                        self.canvas.program.layers.3.solution = search
                            .memory().0.arena
                            .retrace(node.node_id)
                            .unwrap()
                            .make_trajectory()
                            .unwrap();
                        self.canvas.cache.clear();
                    }
                }
            },
            Message::StepProgress => {
                self.step_progress();
            }
            Message::Tick => {
                if self.canvas.program.layers.3.tick() {
                    self.canvas.cache.clear();
                }
            }
        }

        Command::none()
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        iced_native::Subscription::batch([
            iced_native::subscription::events().map(Message::EventOccurred),
            iced::time::every(std::time::Duration::from_millis(50)).map(|_|{ Message::Tick }),
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

        if self.debug_on {
            content = content.push(
                Row::new()
                .push(self.canvas.view())
                .push(
                    Column::new()
                    .push(
                        Row::new()
                        .push(
                            Button::new(&mut self.step_progress, Text::new("Step"))
                            .on_press(Message::StepProgress)
                        )
                        .push(iced::Space::with_width(Length::Units(16)))
                        .push(Text::new(format!("Steps: {}", self.debug_step_count)))
                        .push(iced::Space::with_width(Length::Units(16)))
                        .push(Text::new(format!("Queue size: {}", self.memory.len())))
                        .align_items(Alignment::Center)
                    )
                    .push({
                        let mut scroll = Scrollable::<Message>::new(&mut self.node_list_scroll);
                        if let Some(search) = &self.search {
                            for (i, ticket) in self.memory.iter().enumerate() {
                                let node = search.memory().0.arena.get(ticket.node_id).unwrap();
                                scroll = scroll.push(Radio::new(
                                    i, format!(
                                        "{i}: {:?} + {:?} = {:?}",
                                        node.cost().0,
                                        node.remaining_cost_estimate().0,
                                        ticket.evaluation.0,
                                    ), self.debug_node_selected,
                                    Message::DebugNodeSelected
                                ));
                            }
                        }
                        scroll
                    }.height(Length::Fill))
                    .push({
                        Scrollable::<Message>::new(&mut self.debug_text_scroll)
                        .push(
                            Text::new({
                                if let Some(selection) = self.debug_node_selected {
                                    if let Some(node) = self.memory.get(selection) {
                                        if let Some(search) = &self.search {
                                            let node = search.memory().0.arena.get(node.node_id).unwrap();
                                            format!("{node:#?}")
                                        } else {
                                            String::new()
                                        }
                                    } else {
                                        String::new()
                                    }
                                } else {
                                    String::new()
                                }
                            }).size(10)
                        )
                    }.height(Length::Fill))
                )
            );
        } else {
            content = content.push(self.canvas.view());
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
    DebugNodeSelected(usize),
    StepProgress,
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
