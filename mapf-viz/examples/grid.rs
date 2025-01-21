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

use clap::Parser;
use iced::{
    button::{self, Button},
    canvas::{self, Event, Path, Stroke},
    executor, keyboard, mouse,
    pick_list::{self, PickList},
    scrollable::{self, Scrollable},
    slider::{self, Slider},
    text_input::{self, TextInput},
    Alignment, Application, Column, Command, Container, Element, Length, Radio, Row, Text,
};
use iced_native;
use mapf::{
    algorithm::{tree::NodeContainer, AStarConnect, QueueLength, SearchStatus},
    graph::{
        occupancy::{Accessibility, AccessibilityGraph, Cell, Grid, SparseGrid},
        SharedGraph,
    },
    motion::{
        se2::{
            DifferentialDriveLineFollow, GoalSE2, LinearTrajectorySE2, Point, Vector, WaypointSE2,
        },
        CcbsEnvironment, CircularProfile, DynamicCircularObstacle, DynamicEnvironment, Motion,
        Trajectory, TravelEffortCost,
    },
    negotiation::*,
    planner::halt::QueueLengthLimit,
    planner::{Planner, Search},
    premade::SippSE2,
    templates::InformedSearch,
};
use mapf_viz::spatial_layers;
use mapf_viz::{
    spatial_canvas::{InclusionZone, SpatialCache, SpatialCanvas, SpatialCanvasProgram},
    toggle::{KeyToggler, Toggle, Toggler},
    InfiniteGrid, SparseGridAccessibilityVisual,
};
use native_dialog::FileDialog;
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
};
use time_point::{Duration, TimePoint};

type SparseAccessibility = Accessibility<SparseGrid>;

const ASCII_UPPER: [char; 26] = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S',
    'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
];

fn generate_robot_name(mut index: usize) -> String {
    let mut chars = Vec::new();
    let mut offset = 0;
    while index >= 26 {
        chars.push(index % 26);
        index = index / 26;
        offset = 1;
    }
    chars.push(index % 26 - offset);
    chars.reverse();
    String::from_iter(chars.into_iter().map(|i| ASCII_UPPER[i]))
}

const MIN_AGENT_RADIUS: f64 = 0.05;
const MAX_AGENT_RADIUS: f64 = 5.0;
const MIN_AGENT_SPEED: f64 = 0.01;
const MAX_AGENT_SPEED: f64 = 10.0;
const MIN_AGENT_SPIN: f64 = 1.0 * std::f64::consts::PI / 180.0;
const MAX_AGENT_SPIN: f64 = 360.0 * std::f64::consts::PI / 180.0;
const MIN_AGENT_YAW: f64 = -std::f64::consts::PI;
const MAX_AGENT_YAW: f64 = std::f64::consts::PI;

fn triangular_for<Item>(
    mut outer_iter: impl Iterator<Item = Item> + Clone,
    mut f: impl FnMut(&Item, Item),
) {
    while let Some(outer_value) = outer_iter.next() {
        let mut inner_iter = outer_iter.clone();
        while let Some(inner_value) = inner_iter.next() {
            f(&outer_value, inner_value);
        }
    }
}

fn serialize_grid(grid: &SparseGrid) -> HashMap<i64, Vec<i64>> {
    let mut ser: HashMap<i64, Vec<i64>> = HashMap::new();
    for cell in grid.occupied_cells() {
        ser.entry(cell.y).or_default().push(cell.x);
    }

    for (_, column) in &mut ser {
        column.sort_unstable();
    }

    ser
}

pub(crate) struct Minimum<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> {
    value: Option<T>,
    f: F,
}

impl<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> Minimum<T, F> {
    pub(crate) fn new(f: F) -> Self {
        Self { value: None, f }
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
    frame.with_save(|frame| {
        frame.translate([point.x as f32, point.y as f32].into());
        frame.fill(&Path::circle([0, 0].into(), agent_radius), color);

        if let Some(angle) = angle {
            frame.rotate(angle as f32);
            let r = 0.8 * agent_radius;
            let angle_to_point =
                |angle: f32| iced::Point::new(r * f32::cos(angle), r * f32::sin(angle));

            let points: [iced::Point; 3] = [
                angle_to_point(-150_f32.to_radians()),
                angle_to_point(0_f32.to_radians()),
                angle_to_point(150_f32.to_radians()),
            ];

            frame.fill(
                &Path::new(|builder| {
                    builder.move_to(points[0]);
                    builder.line_to(points[1]);
                    builder.line_to(points[2]);
                    builder.line_to(points[0]);
                }),
                iced::Color::from_rgb(1.0, 1.0, 1.0),
            );
        }
    });
}

fn draw_trajectory(
    frame: &mut canvas::Frame,
    trajectory: &LinearTrajectorySE2,
    color: iced::Color,
    width: Option<f32>,
) {
    for [wp0, wp1] in trajectory.array_windows() {
        let p0 = wp0.position.translation;
        let p1 = wp1.position.translation;
        frame.stroke(
            &Path::line(
                [p0.x as f32, p0.y as f32].into(),
                [p1.x as f32, p1.y as f32].into(),
            ),
            Stroke {
                color,
                width: width.unwrap_or(3_f32),
                ..Default::default()
            },
        );
    }
}

#[derive(Debug, Clone)]
struct EndpointSelector<Message> {
    pub agents: BTreeMap<String, AgentContext>,
    pub cell_size: f64,
    pub invalid_color: iced::Color,
    pub selected_color: iced::Color,
    pub start_color: iced::Color,
    pub goal_color: iced::Color,
    pub show_details: bool,
    pub selected_agent: Option<String>,
    shift: KeyToggler,
    _ignore: std::marker::PhantomData<Message>,
}

#[derive(Debug, Clone)]
struct AgentContext {
    agent: Agent,
    start_open: bool,
    start_available: bool,
    goal_open: bool,
    goal_available: bool,
    start_sees_goal: bool,
}

impl AgentContext {
    fn new(agent: Agent) -> Self {
        Self {
            agent,
            start_open: false,
            start_available: false,
            goal_open: false,
            goal_available: false,
            start_sees_goal: false,
        }
    }

    fn endpoints_valid(&self) -> bool {
        self.start_valid() && self.goal_valid()
    }

    fn start_valid(&self) -> bool {
        self.start_open && self.start_available
    }

    fn goal_valid(&self) -> bool {
        self.goal_open && self.goal_available
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Endpoint {
    Start,
    Goal,
}

impl Endpoint {
    fn of(&self, agent: &Agent) -> Cell {
        match self {
            Endpoint::Start => agent.start.into(),
            Endpoint::Goal => agent.goal.into(),
        }
    }

    fn set_availability(&self, ctx: &mut AgentContext, value: bool) {
        match self {
            Endpoint::Start => ctx.start_available = value,
            Endpoint::Goal => ctx.goal_available = value,
        }
    }

    fn set_accessibility(&self, ctx: &mut AgentContext, value: bool) {
        match self {
            Endpoint::Start => ctx.start_open = value,
            Endpoint::Goal => ctx.goal_open = value,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EndpointSelection {
    Cell(Endpoint),
}

impl<Message> EndpointSelector<Message> {
    fn new(
        agents: BTreeMap<String, Agent>,
        cell_size: f64,
        invalid_color: iced::Color,
        selected_color: iced::Color,
        start_color: iced::Color,
        goal_color: iced::Color,
    ) -> Self {
        let selected_agent = agents.iter().next().map(|r| r.0.clone());
        Self {
            agents: agents
                .into_iter()
                .map(|(name, a)| (name, AgentContext::new(a)))
                .collect(),
            selected_agent,
            cell_size,
            invalid_color,
            selected_color,
            start_color,
            goal_color,
            show_details: false,
            shift: KeyToggler::for_key(keyboard::KeyCode::LShift),
            _ignore: Default::default(),
        }
    }

    fn selected_agent(&self) -> Option<&AgentContext> {
        if let Some(selected_agent) = &self.selected_agent {
            self.agents.get(selected_agent)
        } else {
            None
        }
    }

    fn selected_agent_mut(&mut self) -> Option<&mut AgentContext> {
        if let Some(selected_agent) = &self.selected_agent {
            self.agents.get_mut(selected_agent)
        } else {
            None
        }
    }

    fn update_endpoint_conflicts(
        &mut self,
        endpoint: Endpoint,
        accessibility: &SparseAccessibility,
    ) {
        let Some(selected) = &self.selected_agent else {
            return;
        };
        let cell: Cell = if let Some(ctx) = self.agents.get(selected) {
            endpoint.of(&ctx.agent)
        } else {
            return;
        };

        let cs = accessibility.grid().cell_size();
        let p = cell.center_point(cs);

        // Start by assuming all are valid
        for ctx in self.agents.values_mut() {
            endpoint.set_availability(ctx, true);

            endpoint.set_accessibility(
                ctx,
                accessibility
                    .grid()
                    .is_circle_occupied(p, ctx.agent.radius)
                    .is_none(),
            );
        }

        // Test O(N^2) for invalidity. We could make this way more efficient.
        let mut invalidated = Vec::new();
        triangular_for(self.agents.iter(), |(name_i, ctx_i), (name_j, ctx_j)| {
            let p_i = endpoint.of(&ctx_i.agent).center_point(cs);
            let p_j = endpoint.of(&ctx_j.agent).center_point(cs);
            let dist = (p_i - p_j).norm();
            let min_dist = ctx_i.agent.radius + ctx_j.agent.radius;
            if dist < min_dist {
                invalidated.push((*name_i).clone());
                invalidated.push(name_j.clone());
            }
        });

        for name in invalidated {
            let Some(ctx) = self.agents.get_mut(&name) else {
                continue;
            };
            endpoint.set_availability(ctx, false);
        }
    }

    fn endpoint_cell(&self, choice: Endpoint) -> Option<&[i64; 2]> {
        if let Some(ctx) = self.selected_agent() {
            match choice {
                Endpoint::Start => Some(&ctx.agent.start),
                Endpoint::Goal => Some(&ctx.agent.goal),
            }
        } else {
            None
        }
    }

    fn endpoint_cell_mut(&mut self, choice: Endpoint) -> Option<&mut [i64; 2]> {
        if let Some(ctx) = self.selected_agent_mut() {
            match choice {
                Endpoint::Start => Some(&mut ctx.agent.start),
                Endpoint::Goal => Some(&mut ctx.agent.goal),
            }
        } else {
            None
        }
    }

    fn set_endpoint_valid(&mut self, choice: Endpoint, valid: bool) {
        if let Some(ctx) = self.selected_agent_mut() {
            match choice {
                Endpoint::Start => {
                    ctx.start_open = valid;
                }
                Endpoint::Goal => {
                    ctx.goal_open = valid;
                }
            }
        }
    }

    fn set_agent_radius(&mut self, radius: f64) -> bool {
        match self.selected_agent_mut() {
            Some(ctx) => {
                ctx.agent.radius = radius;
                true
            }
            None => false,
        }
    }
}

impl SpatialCanvasProgram<Message> for EndpointSelector<Message> {
    fn update(
        &mut self,
        event: iced::canvas::Event,
        cursor: iced::canvas::Cursor,
    ) -> (SpatialCache, iced::canvas::event::Status, Option<Message>) {
        if let Some(p) = cursor.position() {
            match event {
                Event::Mouse(event) => {
                    if self.shift.state() == Toggle::Off {
                        let endpoint = match event {
                            mouse::Event::ButtonPressed(mouse::Button::Left) => Endpoint::Start,
                            mouse::Event::ButtonPressed(mouse::Button::Right) => Endpoint::Goal,
                            _ => {
                                return (
                                    SpatialCache::Unchanged,
                                    canvas::event::Status::Ignored,
                                    None,
                                )
                            }
                        };

                        let cell_size = self.cell_size;
                        if let Some(cell) = self.endpoint_cell_mut(endpoint) {
                            *cell =
                                Cell::from_point([p.x as f64, p.y as f64].into(), cell_size).into();
                        }

                        return (
                            SpatialCache::Refresh,
                            canvas::event::Status::Captured,
                            Some(Message::EndpointSelected(EndpointSelection::Cell(endpoint))),
                        );
                    }
                }
                Event::Keyboard(event) => {
                    self.shift.key_toggle(event);
                }
            }
        }

        return (
            SpatialCache::Unchanged,
            canvas::event::Status::Ignored,
            None,
        );
    }

    fn draw_in_space(
        &self,
        frame: &mut canvas::Frame,
        _spatial_bounds: iced::Rectangle,
        _spatial_cursor: canvas::Cursor,
    ) {
        for (name, ctx) in &self.agents {
            if self.show_details {
                if ctx.start_sees_goal {
                    let cell_s: Cell = ctx.agent.start.into();
                    let cell_g: Cell = ctx.agent.goal.into();
                    let p_start = cell_s.center_point(self.cell_size);
                    let p_goal = cell_g.center_point(self.cell_size);
                    frame.stroke(
                        &Path::line(
                            [p_start.x as f32, p_start.y as f32].into(),
                            [p_goal.x as f32, p_goal.y as f32].into(),
                        ),
                        Stroke {
                            color: self.selected_color,
                            width: 5_f32,
                            ..Default::default()
                        },
                    );
                }
            }

            for (cell, angle, color, valid) in [
                (
                    ctx.agent.start,
                    Some(ctx.agent.yaw),
                    self.start_color,
                    ctx.start_valid(),
                ),
                (ctx.agent.goal, None, self.goal_color, ctx.goal_valid()),
            ] {
                let radius = ctx.agent.radius as f32;
                let cell: Cell = cell.into();
                let p = cell.center_point(self.cell_size);

                draw_agent(frame, p, angle, radius, color);
                let is_selected = self
                    .selected_agent
                    .as_ref()
                    .filter(|n| **n == *name)
                    .is_some();
                if !valid || is_selected {
                    let color = if !valid {
                        self.invalid_color
                    } else {
                        self.selected_color
                    };

                    frame.stroke(
                        &Path::circle([p.x as f32, p.y as f32].into(), radius),
                        Stroke {
                            color,
                            width: 5_f32,
                            ..Default::default()
                        },
                    );
                }
            }
        }
    }

    fn draw_on_hud<'a, 'b: 'a>(
        &self,
        hud: &'a mut mapf_viz::spatial_canvas::SpatialHUD<'b>,
        bounds: iced::Rectangle,
        _cursor: canvas::Cursor,
    ) {
        for (name, ctx) in &self.agents {
            for (kind, cell, offset) in
                [("Start", ctx.agent.start, -16), ("Goal", ctx.agent.goal, 0)]
            {
                let cell: Cell = cell.into();
                let p = cell.center_point(self.cell_size);
                let r = ctx.agent.radius as f32;
                let delta = iced::Vector::new(r, -r);
                let p = iced::Point::new(p.x as f32, p.y as f32) + delta;

                if bounds.contains(p) {
                    hud.at(p, |frame| {
                        frame.translate(iced::Vector::new(0.0, offset as f32));
                        frame.fill_text(
                            format!("{} {}: ({}, {})", name, kind, cell.x, cell.y).to_owned(),
                        );
                    })
                }
            }
        }
    }

    fn estimate_bounds(&self) -> mapf_viz::spatial_canvas::InclusionZone {
        let mut zone = InclusionZone::Empty;
        for (_, ctx) in &self.agents {
            for endpoint in [ctx.agent.start, ctx.agent.goal] {
                let cell: Cell = endpoint.into();
                let p = cell.center_point(self.cell_size);
                for v in [[1.0, 1.0], [-1.0, -1.0]] {
                    let v: Vector = v.into();
                    let r = p + ctx.agent.radius * v;
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
    pub solution_color: iced::Color,
    pub solutions: Vec<(f64, LinearTrajectorySE2)>,
    pub search_color: iced::Color,
    pub searches: Vec<(f64, LinearTrajectorySE2)>,
    pub obstacles: Vec<(f64, LinearTrajectorySE2)>,
    pub vertex_lookup: HashMap<Cell, usize>,
    tick_start: Option<TimePoint>,
    now: Option<Duration>,
    _msg: std::marker::PhantomData<Message>,
}

impl SolutionVisual<Message> {
    fn new(
        cell_size: f64,
        solution_color: iced::Color,
        search_color: iced::Color,
        obstacles: Vec<(f64, LinearTrajectorySE2)>,
    ) -> Self {
        Self {
            cell_size,
            solution_color,
            solutions: Vec::new(),
            search_color,
            searches: Vec::new(),
            obstacles,
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

        for (_, traj) in &self.searches {
            earliest.consider(&traj.initial_motion_time());
            latest.consider(&traj.finish_motion_time());
        }

        if !earliest.has_value() || !latest.has_value() {
            for (_, solution) in &self.solutions {
                earliest.consider(&solution.initial_motion_time());
                latest.consider(&solution.finish_motion_time());
            }
        }

        if !earliest.has_value() || !latest.has_value() {
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
        _spatial_cursor: canvas::Cursor,
    ) {
        if let Some((t0, _)) = self.time_range() {
            for (_, trajectory) in &self.searches {
                draw_trajectory(frame, trajectory, self.search_color, None);
            }

            for (_, trajectory) in &self.solutions {
                draw_trajectory(frame, trajectory, self.solution_color, None);
            }

            for (r, obs) in &self.obstacles {
                let red = iced::Color::from_rgb(1.0, 0.0, 0.0);
                draw_trajectory(frame, obs, red, Some(8_f32));

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
                for (radius, trajectory) in &self.searches {
                    if let Ok(p) = trajectory.motion().compute_position(&(t0 + now)) {
                        draw_agent(
                            frame,
                            p.translation.vector.into(),
                            Some(p.rotation.angle()),
                            *radius as f32,
                            self.search_color,
                        );
                    }
                }

                for (radius, trajectory) in &self.solutions {
                    if let Ok(p) = trajectory.motion().compute_position(&(t0 + now)) {
                        draw_agent(
                            frame,
                            Point::from(p.translation.vector),
                            Some(p.rotation.angle()),
                            *radius as f32,
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
        _spatial_cursor: canvas::Cursor,
    ) {
        for (cell, v) in &self.vertex_lookup {
            let p = cell.center_point(self.cell_size);
            let p = iced::Point::new(p.x as f32, p.y as f32);
            if bound.contains(p) {
                hud.at(p, |frame| {
                    frame.fill_text(format!("{v}"));
                });
            }
        }
    }

    fn estimate_bounds(&self) -> InclusionZone {
        // This layer should always be contained within the other layers,
        // so we don't need to provide any bounds for this.
        return InclusionZone::Empty;
    }
}

spatial_layers!(GridLayers<Message>: InfiniteGrid, SparseGridAccessibilityVisual, EndpointSelector, SolutionVisual);

type SparseAccessibilityGraph = AccessibilityGraph<SparseGrid>;

type MyAlgo = AStarConnect<SippSE2<SparseAccessibilityGraph, SparseAccessibilityGraph>>;
type TreeTicket = mapf::algorithm::tree::TreeQueueTicket<mapf::domain::Cost<f64>>;

struct App {
    save_button: button::State,
    load_button: button::State,
    select_file_button: button::State,
    file_text_input: text_input::State,
    file_text_input_value: String,
    agent_yaw_slider: slider::State,
    agent_radius_slider: slider::State,
    agent_speed_slider: slider::State,
    agent_spin_slider: slider::State,
    add_agent_button: button::State,
    remove_agent_button: button::State,
    pick_agent_state: pick_list::State<String>,
    reset_view_button: button::State,
    reset_time_button: button::State,
    canvas: SpatialCanvas<Message, GridLayers>,
    node_list_scroll: scrollable::State,
    debug_text_scroll: scrollable::State,
    show_details: KeyToggler,
    search: Option<(f64, Search<MyAlgo, GoalSE2<Cell>, QueueLengthLimit>)>,
    step_progress: button::State,
    debug_planner_on: bool,
    debug_negotiation_on: bool,
    search_memory: Vec<TreeTicket>,
    negotiation_history: Vec<NegotiationNode>,
    name_map: HashMap<usize, String>,
    debug_step_count: u64,
    debug_node_selected: Option<usize>,
    negotiation_node_selected: Option<usize>,
    next_robot_name_index: usize,
}

impl App {
    const TICK_COUNT: u32 = 1000;

    fn get_tick(x: f64, min: f64, max: f64) -> u32 {
        ((x - min) / (max - min) * Self::TICK_COUNT as f64) as u32
    }

    fn from_tick(x: u32, min: f64, max: f64) -> f64 {
        x as f64 * (max - min) / (Self::TICK_COUNT as f64) + min
    }

    fn default_invalid_color() -> iced::Color {
        iced::Color::from_rgb(1.0, 0.0, 0.0)
    }

    fn default_selected_color() -> iced::Color {
        iced::Color::from_rgb(0.1, 1.0, 1.0)
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

    fn save_file(&self, filename: &String) {
        let out_file = match std::fs::File::create(filename) {
            Ok(r) => r,
            Err(err) => {
                println!(
                    "Unable to save to file {}: {:?}",
                    self.file_text_input_value, err
                );
                return;
            }
        };

        let scenario = self.make_scenario();

        match serde_yaml::to_writer(out_file, &scenario) {
            Ok(()) => {}
            Err(err) => {
                println!(
                    "Unable to save to file {}: {:?}",
                    self.file_text_input_value, err
                );
                return;
            }
        }
    }

    fn set_robot_radius(&mut self, radius: f64) {
        self.canvas.program.layers.1.set_robot_radius(radius as f32);
        let endpoint_selector = &mut self.canvas.program.layers.2;
        if endpoint_selector.set_agent_radius(radius) {
            for endpoint in [Endpoint::Start, Endpoint::Goal] {
                if let Some(cell) = endpoint_selector.endpoint_cell(endpoint) {
                    let cell: Cell = (*cell).into();
                    let p = cell.center_point(endpoint_selector.cell_size);
                    let valid = self
                        .canvas
                        .program
                        .layers
                        .1
                        .grid()
                        .is_circle_occupied(p, radius)
                        .is_none();
                    endpoint_selector.set_endpoint_valid(endpoint, valid);
                }
            }
            self.canvas.cache.clear();
        }

        self.update_all_endpoints();
    }

    fn update_all_endpoints(&mut self) {
        self.canvas.program.layers.2.update_endpoint_conflicts(
            Endpoint::Start,
            &self.canvas.program.layers.1.accessibility(),
        );
        self.canvas.program.layers.2.update_endpoint_conflicts(
            Endpoint::Goal,
            &self.canvas.program.layers.1.accessibility(),
        );
    }

    fn select_search_node(&mut self, value: usize) {
        if self.search_memory.is_empty() {
            self.debug_node_selected = Some(0);
        } else {
            self.debug_node_selected = Some(usize::min(value, self.search_memory.len() - 1));
        }

        self.canvas.program.layers.3.solutions.clear();
        if let Some(ticket) = self.search_memory.get(value) {
            if let Some((r, search)) = &self.search {
                let solution = search
                    .memory()
                    .0
                    .arena
                    .retrace(ticket.node_id)
                    .unwrap()
                    .make_trajectory()
                    .unwrap();

                self.canvas
                    .program
                    .layers
                    .3
                    .solutions
                    .extend(solution.map(|s| (*r, s.trajectory)).into_iter());
                self.canvas.cache.clear();

                self.canvas.program.layers.3.obstacles.clear();
                if let Some(node) = search.memory().0.arena.get(ticket.node_id) {
                    if let Some(n) = self.negotiation_node_selected {
                        if let Some(n) = self.negotiation_history.get(n) {
                            let agent_id = self
                                .name_map
                                .iter()
                                .find(|(_, name)| {
                                    **name
                                        == *self
                                            .canvas
                                            .program
                                            .layers
                                            .2
                                            .selected_agent
                                            .as_ref()
                                            .unwrap()
                                })
                                .map(|(i, _)| *i);

                            if let Some(agent_id) = agent_id {
                                for obs in n
                                    .environment
                                    .iter_obstacles_from(node.state().key.vertex, agent_id)
                                {
                                    if let Some(t) = obs.trajectory() {
                                        let r = obs.profile().footprint_radius();
                                        self.canvas.program.layers.3.obstacles.push((r, t.clone()));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fn step_progress(&mut self) {
        if let Some((radius, search)) = &mut self.search {
            self.debug_step_count += 1;

            self.search_memory = search
                .memory()
                .0
                .queue
                .clone()
                .into_iter_sorted()
                .map(|n| n.0.clone())
                .collect();

            // TODO(@mxgrey): Make the number to take configurable
            for ticket in self.search_memory.iter().take(10) {
                if let Some(mt) = search
                    .memory()
                    .0
                    .arena
                    .retrace(ticket.node_id)
                    .unwrap()
                    .make_trajectory()
                    .unwrap()
                {
                    self.canvas
                        .program
                        .layers
                        .3
                        .searches
                        .push((*radius, mt.trajectory));
                }
            }

            if let SearchStatus::Solved(solution) = search.step().unwrap() {
                println!("Queue length: {}", search.memory().queue_length());
                println!("Solution: {:#?}", solution);
                self.canvas.program.layers.3.solutions.clear();
                self.canvas.program.layers.3.solutions.extend(
                    solution
                        .make_trajectory()
                        .unwrap()
                        .map(|t| (*radius, t.trajectory))
                        .into_iter(),
                );
                self.debug_node_selected = None;
                self.search = None;
            } else {
                println!("Queue length: {}", search.memory().queue_length());
                if let Some(selection) = self.debug_node_selected {
                    self.select_search_node(selection);
                }
            }

            self.canvas.cache.clear();
        }
    }

    fn add_agent(&mut self) {
        let name = {
            let mut name = generate_robot_name(self.next_robot_name_index);
            while self.canvas.program.layers.2.agents.contains_key(&name) {
                self.next_robot_name_index += 1;
                name = generate_robot_name(self.next_robot_name_index);
            }

            self.next_robot_name_index += 1;
            name
        };

        let start = [-30, 0];
        let goal = [-25, 0];
        // let start = [-5, 0];
        // let goal = [5, 0];
        let yaw = 0.0;
        let agent = match self.canvas.program.layers.2.selected_agent() {
            Some(ctx) => Agent {
                start,
                goal,
                yaw,
                ..ctx.agent
            },
            None => Agent {
                start,
                goal,
                yaw,
                radius: default_radius(),
                speed: default_speed(),
                spin: default_spin(),
            },
        };

        self.canvas
            .program
            .layers
            .2
            .agents
            .insert(name.clone(), AgentContext::new(agent));
        self.canvas.program.layers.2.selected_agent = Some(name);
        self.canvas
            .program
            .layers
            .1
            .set_robot_radius(agent.radius as f32);
        self.update_all_endpoints();
        self.canvas.cache.clear();
        self.generate_plan();
    }

    fn remove_agent(&mut self) {
        if let Some(selected) = &self.canvas.program.layers.2.selected_agent {
            self.canvas.program.layers.2.agents.remove(selected);
            self.canvas.program.layers.2.selected_agent = self
                .canvas
                .program
                .layers
                .2
                .agents
                .iter()
                .next()
                .map(|r| r.0.clone());
        }
    }

    fn make_scenario(&self) -> Scenario {
        let cell_size = self.canvas.program.layers.2.cell_size;
        let camera_bounds = match self.canvas.camera_bounds() {
            InclusionZone::Empty => None,
            InclusionZone::Some { lower, upper } => Some([[lower.x, lower.y], [upper.x, upper.y]]),
        };
        Scenario {
            agents: self
                .canvas
                .program
                .layers
                .2
                .agents
                .iter()
                .map(|(n, a)| (n.clone(), a.agent.clone()))
                .collect(),
            obstacles: self
                .canvas
                .program
                .layers
                .3
                .obstacles
                .iter()
                .map(|obs| Obstacle::new(obs.0, &obs.1, cell_size))
                .collect(),
            occupancy: serialize_grid(self.canvas.program.layers.1.grid()),
            cell_size,
            camera_bounds,
        }
    }

    fn generate_plan(&mut self) {
        // Clear all history
        self.search_memory.clear();
        self.search = None;
        self.canvas.program.layers.3.solutions.clear();
        self.canvas.program.layers.3.searches.clear();
        self.canvas.cache.clear();

        if self.debug_planner_on {
            let endpoints = &self.canvas.program.layers.2;
            let accessibility = self.canvas.program.layers.1.accessibility();
            let Some(ctx) = endpoints.selected_agent() else {
                return;
            };

            if !ctx.endpoints_valid() {
                print!(
                    "Cannot plan for {}:",
                    endpoints.selected_agent.as_ref().unwrap()
                );
                if !ctx.start_open {
                    print!(" invalid start |");
                }
                if !ctx.start_available {
                    print!(" start conflict |");
                }
                if !ctx.goal_open {
                    print!(" invalid goal |");
                }
                if !ctx.goal_available {
                    print!(" goal conflict |");
                }
                println!("");
                return;
            }

            let r = ctx.agent.radius;
            let extrapolator = DifferentialDriveLineFollow::new(ctx.agent.speed, ctx.agent.spin)
                .expect("Bad speeds");
            let profile = CircularProfile::new(r, 0.0, 0.0).expect("Bad profile sizes");

            let shared_accessibility = Arc::new(accessibility.clone());
            let activity_graph = SharedGraph::new(AccessibilityGraph::new(shared_accessibility));
            let heuristic_graph = activity_graph.clone();

            let (environment, minimum_time) = 'environment: {
                // if let Some(n) = self.negotiation_node_selected {
                if let Some(n) = self.negotiation_node_selected {
                    if let Some(node) = self.negotiation_history.get(n) {
                        let mut env = node.environment.clone();
                        let agent_id = self
                            .name_map
                            .iter()
                            .find(|(_, name)| **name == *endpoints.selected_agent.as_ref().unwrap())
                            .map(|(i, _)| *i);

                        // if let Some(agent_id) = agent_id {
                        if let Some(agent_id) = agent_id {
                            env.set_mask(Some(agent_id));
                            env.overlay_trajectory(agent_id, None).ok();
                            self.canvas.program.layers.3.obstacles.clear();
                            for obs in env.iter_all_obstacles() {
                                if let Some(t) = obs.trajectory() {
                                    let r = obs.profile().footprint_radius();
                                    self.canvas.program.layers.3.obstacles.push((r, t.clone()));
                                }
                            }

                            let finish_time = node
                                .proposals
                                .values()
                                .max_by_key(|t| t.meta.trajectory.finish_motion_time())
                                .unwrap()
                                .meta
                                .trajectory
                                .finish_motion_time();

                            break 'environment (Arc::new(env), Some(finish_time));
                        }
                    }
                }

                let environment = Arc::new(CcbsEnvironment::new(Arc::new({
                    let mut env = DynamicEnvironment::new(profile);
                    for (obs_size, obs_traj) in self.canvas.program.layers.3.obstacles.iter() {
                        env.obstacles.push(
                            DynamicCircularObstacle::new(
                                CircularProfile::new(*obs_size, 0.0, 0.0).unwrap(),
                            )
                            .with_trajectory(Some(obs_traj.clone())),
                        );
                    }
                    env
                })));

                let mut minimum_time: Option<TimePoint> = None;
                for obs in environment.iter_all_obstacles() {
                    if let Some(t) = obs.trajectory() {
                        let tf = t.finish_motion_time();
                        minimum_time = Some(minimum_time.map(|t| t.min(tf)).unwrap_or(tf));
                    }
                }

                (environment, minimum_time)
            };

            let domain = InformedSearch::new_sipp_se2(
                activity_graph,
                heuristic_graph,
                extrapolator,
                environment,
                TravelEffortCost::save_one_second_with_detour_up_to(5.0, 360_f64.to_radians()),
            )
            .unwrap();

            let start = ctx.agent.make_start();
            let goal = ctx.agent.make_goal().with_minimum_time(minimum_time);

            println!(
                "About to plan for {}:\nStart: {start:#?}\nGoal: {goal:#?}",
                endpoints.selected_agent.as_ref().unwrap(),
            );

            let planner =
                Planner::new(AStarConnect(domain)).with_halting(QueueLengthLimit(Some(1_000_000)));
            // .with_halting(MeasureLimit(None));
            let search = planner.plan(start, goal).unwrap();

            self.canvas.program.layers.3.searches.clear();
            self.debug_step_count = 0;
            if self.debug_planner_on {
                self.search_memory = search
                    .memory()
                    .0
                    .queue
                    .clone()
                    .into_iter_sorted()
                    .map(|n| n.0.clone())
                    .collect();
                self.search = Some((r, search));
            }
            self.debug_node_selected = None;
            return;
        }

        self.negotiation_history.clear();
        self.canvas.program.layers.3.obstacles.clear();
        let scenario = self.make_scenario();

        if !self.file_text_input_value.is_empty() {
            let mut backup = self.file_text_input_value.clone();
            backup += ".backup";
            println!("Saving backup file {backup}");
            self.save_file(&backup);
        }

        let start_time = std::time::Instant::now();
        // let propsoals = match negotiate(&scenario, Some(1_000_000)) {
        let (solution_node, node_history, name_map) = match negotiate(
            &scenario,
            // Some(1_000),
            Some(1_000_000),
            // None
        ) {
            Ok(solutions) => solutions,
            Err(err) => {
                match err {
                    NegotiationError::PlanningFailed((nodes, name_map)) => {
                        println!("Unable to find a solution");
                        self.negotiation_history = nodes;
                        self.negotiation_history.sort_unstable_by_key(|n| n.id);
                        self.name_map = name_map;
                    }
                    err => println!("Error while planning: {err:?}"),
                };

                let elapsed = start_time.elapsed();
                println!("Elapsed time: {} seconds", elapsed.as_secs_f64());
                return;
            }
        };
        let elapsed = start_time.elapsed();
        println!("Successful planning took {} seconds", elapsed.as_secs_f64());
        dbg!(node_history.len());

        assert!(self.canvas.program.layers.3.solutions.is_empty());
        for (i, proposal) in &solution_node.proposals {
            let name = name_map.get(i).unwrap();
            let r = scenario.agents.get(name).unwrap().radius;
            self.canvas
                .program
                .layers
                .3
                .solutions
                .push((r, proposal.meta.trajectory.clone()));
        }

        if self.debug_negotiation_on {
            self.negotiation_history = node_history;
            self.negotiation_history.sort_unstable_by_key(|n| n.id);
        }
        self.name_map = name_map;
    }
}

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Scenario to load on startup
    filename: Option<String>,
}

fn load_file(filename: &String) -> Option<Scenario> {
    let f = match std::fs::File::open(&filename) {
        Ok(f) => f,
        Err(err) => {
            println!("Unable to open file {}: {err:?}", filename);
            return None;
        }
    };
    let scenario: Scenario = match serde_yaml::from_reader(f) {
        Ok(scenario) => scenario,
        Err(err) => {
            println!("Unable to parse scenario in file {}: {err:?}", filename);
            return None;
        }
    };
    Some(scenario)
}

fn load_scenario(
    filename: Option<&String>,
) -> (
    BTreeMap<String, Agent>,
    Vec<(f64, Trajectory<WaypointSE2>)>,
    SparseGrid,
    InclusionZone,
    bool,
) {
    let mut agents = BTreeMap::new();
    let mut obstacles = Vec::new();
    let mut grid = SparseGrid::new(1.0);
    let mut zone = InclusionZone::Empty;
    let mut success = false;
    if let Some(filename) = filename {
        if let Some(scenario) = load_file(filename) {
            let cell_size = scenario.cell_size;
            agents = scenario.agents;
            obstacles = scenario
                .obstacles
                .into_iter()
                .filter_map(|obs| {
                    LinearTrajectorySE2::from_iter(obs.trajectory.into_iter().map(|(t, x, y)| {
                        let p = Cell::new(x, y).center_point(cell_size);
                        WaypointSE2::new_f64(t, p.x, p.y, 0.0)
                    }))
                    .ok()
                    .map(|t| {
                        t.with_indefinite_initial_time(obs.indefinite_start)
                            .with_indefinite_finish_time(obs.indefinite_finish)
                    })
                    .map(|t| (obs.radius, t))
                })
                .collect();
            for (y, row) in scenario.occupancy {
                for x in row {
                    grid.change_cells(&[(Cell::new(x, y), true)].into_iter().collect());
                }
            }

            if let Some(bounds) = scenario.camera_bounds {
                for p in bounds {
                    zone.include(iced::Point::new(p[0], p[1]));
                }
            }
            success = true;
        }
    }
    (agents, obstacles, grid, zone, success)
}

impl Application for App {
    type Message = Message;
    type Executor = executor::Default;
    type Flags = Args;

    fn new(flags: Self::Flags) -> (Self, Command<Self::Message>) {
        let cell_size = 1.0_f32;
        let (agents, obstacles, grid, mut zone, _) = load_scenario(flags.filename.as_ref());

        let mut canvas = SpatialCanvas::new(GridLayers {
            layers: (
                InfiniteGrid::new(cell_size),
                SparseGridAccessibilityVisual::new(
                    grid,
                    default_radius() as f32,
                    Some(Box::new(|| Message::OccupancyChanged)),
                )
                .showing_accessibility(true),
                EndpointSelector::new(
                    agents,
                    cell_size as f64,
                    Self::default_invalid_color(),
                    Self::default_selected_color(),
                    Self::default_endpoint_color(Endpoint::Start),
                    Self::default_endpoint_color(Endpoint::Goal),
                ),
                SolutionVisual::new(
                    cell_size as f64,
                    Self::default_solution_color(),
                    Self::default_search_color(),
                    obstacles,
                ),
            ),
        });
        canvas.zoom = 20.0;

        let mut app = Self {
            save_button: button::State::new(),
            load_button: button::State::new(),
            select_file_button: button::State::new(),
            file_text_input: text_input::State::new(),
            file_text_input_value: flags.filename.unwrap_or(String::new()),
            agent_yaw_slider: slider::State::new(),
            agent_radius_slider: slider::State::new(),
            agent_speed_slider: slider::State::new(),
            agent_spin_slider: slider::State::new(),
            add_agent_button: button::State::new(),
            remove_agent_button: button::State::new(),
            pick_agent_state: pick_list::State::new(),
            reset_view_button: button::State::new(),
            reset_time_button: button::State::new(),
            canvas,
            node_list_scroll: scrollable::State::new(),
            debug_text_scroll: scrollable::State::new(),
            show_details: KeyToggler::for_key(keyboard::KeyCode::LAlt),
            search: None,
            step_progress: button::State::new(),
            debug_planner_on: false,
            debug_negotiation_on: false,
            search_memory: Default::default(),
            negotiation_history: Default::default(),
            name_map: Default::default(),
            debug_step_count: 0,
            debug_node_selected: None,
            negotiation_node_selected: None,
            next_robot_name_index: 0,
        };

        if app.canvas.program.layers.2.agents.is_empty() {
            app.add_agent();
            zone.include(iced::Point::new(-10.0, -10.0));
            zone.include(iced::Point::new(10.0, 10.0));
        }

        app.update_all_endpoints();
        app.generate_plan();

        let set_view = Command::perform(async move {}, move |_| Message::SetView(zone));
        let top_agent = app.canvas.program.layers.2.agents.iter().next();
        let set_agent = if let Some((top_agent, _)) = top_agent {
            let top_agent = top_agent.clone();
            Some(Command::perform(async move {}, move |_| {
                Message::SelectAgent(top_agent.clone())
            }))
        } else {
            None
        };

        (
            app,
            Command::batch([set_view].into_iter().chain(set_agent.into_iter())),
        )
    }

    fn title(&self) -> String {
        "Grid Planner".to_owned()
    }

    fn update(&mut self, message: Self::Message) -> Command<Self::Message> {
        match message {
            Message::AddAgent => {
                self.add_agent();
                self.generate_plan();
            }
            Message::RemoveAgent => {
                self.remove_agent();
                self.generate_plan();
            }
            Message::SaveFile | Message::SaveFileAs => {
                if matches!(message, Message::SaveFileAs) {
                    match FileDialog::new().show_save_single_file() {
                        Ok(f) => match f {
                            Some(f) => {
                                self.file_text_input_value =
                                    f.as_path().as_os_str().to_str().unwrap().to_owned()
                            }
                            None => return Command::none(),
                        },
                        Err(err) => {
                            println!("Unable to select file: {err:?}");
                            return Command::none();
                        }
                    }
                }

                self.save_file(&self.file_text_input_value);
            }
            Message::LoadFile => {
                match FileDialog::new().show_open_single_file() {
                    Ok(f) => match f {
                        Some(value) => {
                            self.file_text_input_value =
                                value.as_path().as_os_str().to_str().unwrap().to_owned()
                        }
                        None => return Command::none(),
                    },
                    Err(err) => {
                        println!("Unable to load selected file: {err:?}");
                        return Command::none();
                    }
                }

                let (agents, obstacles, grid, zone, success) =
                    load_scenario(Some(&self.file_text_input_value));
                if !success {
                    return Command::none();
                }

                self.canvas.program.layers.1.set_grid(grid);
                self.canvas.program.layers.2.agents = agents
                    .into_iter()
                    .map(|(n, a)| (n, AgentContext::new(a)))
                    .collect();
                self.canvas.program.layers.3.obstacles = obstacles;
                self.update_all_endpoints();
                self.canvas.cache.clear();
                self.generate_plan();
                return Command::perform(async move {}, move |_| Message::SetView(zone));
            }
            Message::ScenarioFileInput(value) => {
                self.file_text_input_value = value;
            }
            Message::SelectAgent(new_agent) => {
                match self.canvas.program.layers.2.agents.get(&new_agent) {
                    Some(ctx) => {
                        self.canvas.program.layers.2.selected_agent = Some(new_agent);
                        self.canvas
                            .program
                            .layers
                            .1
                            .set_robot_radius(ctx.agent.radius as f32);
                        self.canvas.cache.clear();
                    }
                    None => {}
                }
            }
            Message::AgentYawSlide(value) => {
                if let Some(ctx) = self.canvas.program.layers.2.selected_agent_mut() {
                    ctx.agent.yaw = Self::from_tick(value, MIN_AGENT_YAW, MAX_AGENT_YAW);
                    self.generate_plan();
                }
            }
            Message::AgentRadiusSlide(value) => {
                self.set_robot_radius(Self::from_tick(value, MIN_AGENT_RADIUS, MAX_AGENT_RADIUS));
                self.generate_plan();
            }
            Message::AgentSpeedSlide(value) => {
                if let Some(ctx) = self.canvas.program.layers.2.selected_agent_mut() {
                    ctx.agent.speed = Self::from_tick(value, MIN_AGENT_SPEED, MAX_AGENT_SPEED);
                    self.generate_plan();
                }
            }
            Message::AgentSpinSlide(value) => {
                if let Some(ctx) = self.canvas.program.layers.2.selected_agent_mut() {
                    ctx.agent.spin = Self::from_tick(value, MIN_AGENT_SPIN, MAX_AGENT_SPIN);
                    self.generate_plan();
                }
            }
            Message::ResetView => {
                if !self.canvas.fit_to_bounds() {
                    // The canvas was not ready to fit the view, so we need to
                    // issue this message again
                    return Command::perform(async move {}, move |_| Message::ResetView);
                }
            }
            Message::SetView(zone) => {
                if !self.canvas.fit_to_zone(zone) {
                    // The canvas was not ready to fit the view, so we need to
                    // issue this message again
                    return Command::perform(async move {}, move |_| Message::SetView(zone));
                }
            }
            Message::ResetTime => {
                self.canvas.program.layers.3.reset_time();
            }
            Message::EventOccurred(event) => {
                if let iced_native::Event::Keyboard(event) = event {
                    match self.show_details.key_toggle(event) {
                        Toggle::On => {
                            // self.canvas.program.layers.1.show_accessibility = true;
                            self.canvas.program.layers.1.show_accessibility = false;
                            self.canvas.program.layers.2.show_details = true;
                            self.canvas.cache.clear();
                        }
                        Toggle::Off => {
                            // self.canvas.program.layers.1.show_accessibility = false;
                            self.canvas.program.layers.1.show_accessibility = true;
                            self.canvas.program.layers.2.show_details = false;
                            self.canvas.cache.clear();
                        }
                        Toggle::NoChange => {
                            // Do nothing
                        }
                    }

                    if let keyboard::Event::KeyPressed {
                        key_code: keyboard::KeyCode::D,
                        modifiers,
                    } = event
                    {
                        if modifiers.shift() {
                            self.debug_planner_on = false;
                            self.generate_plan();
                        } else {
                            self.debug_planner_on = true;
                            self.generate_plan();
                        }
                    }

                    if let keyboard::Event::KeyPressed {
                        key_code: keyboard::KeyCode::N,
                        modifiers,
                    } = event
                    {
                        if modifiers.shift() {
                            self.debug_negotiation_on = false;
                            self.negotiation_history.clear();
                        } else {
                            self.debug_negotiation_on = true;
                            self.generate_plan();
                        }
                    }

                    if let keyboard::Event::KeyPressed {
                        key_code: keyboard::KeyCode::S,
                        ..
                    } = event
                    {
                        self.step_progress();
                    }

                    if let keyboard::Event::KeyPressed {
                        key_code: keyboard::KeyCode::Down,
                        ..
                    } = event
                    {
                        if self.debug_planner_on {
                            let next_debug_node =
                                self.debug_node_selected.map(|n| n + 1).unwrap_or(0);
                            return Command::perform(async move {}, move |_| {
                                Message::SelectDebugNode(next_debug_node)
                            });
                        } else {
                            let next_negotiation_node =
                                self.negotiation_node_selected.map(|n| n + 1).unwrap_or(0);
                            return Command::perform(async move {}, move |_| {
                                Message::SelectNegotiationNode(next_negotiation_node)
                            });
                        }
                    }

                    if let keyboard::Event::KeyPressed {
                        key_code: keyboard::KeyCode::Up,
                        ..
                    } = event
                    {
                        if self.debug_planner_on {
                            let next_debug_node = self
                                .debug_node_selected
                                .map(|n| if n > 0 { n - 1 } else { 0 })
                                .unwrap_or(0);
                            return Command::perform(async move {}, move |_| {
                                Message::SelectDebugNode(next_debug_node)
                            });
                        } else {
                            let next_negotiation_node = self
                                .negotiation_node_selected
                                .map(|n| if n > 0 { n - 1 } else { 0 })
                                .unwrap_or(0);
                            return Command::perform(async move {}, move |_| {
                                Message::SelectNegotiationNode(next_negotiation_node)
                            });
                        }
                    }
                }
            }
            Message::EndpointSelected(selection) => {
                match selection {
                    EndpointSelection::Cell(endpoint) => {
                        self.canvas.program.layers.2.update_endpoint_conflicts(
                            endpoint,
                            self.canvas.program.layers.1.accessibility(),
                        );
                    }
                }
                self.generate_plan();
            }
            Message::OccupancyChanged => {
                self.generate_plan();
            }
            Message::SelectDebugNode(value) => {
                self.select_search_node(value);
            }
            Message::SelectNegotiationNode(value) => {
                if self.negotiation_history.is_empty() {
                    self.negotiation_node_selected = None;
                } else {
                    self.negotiation_node_selected =
                        Some(usize::min(value, self.negotiation_history.len() - 1));
                }

                if let Some(node) = self.negotiation_history.get(value) {
                    self.canvas.program.layers.3.obstacles.clear();
                    for obs in node.environment.iter_all_obstacles() {
                        if let Some(t) = obs.trajectory() {
                            let r = obs.profile().footprint_radius();
                            self.canvas.program.layers.3.obstacles.push((r, t.clone()));
                        }
                    }

                    self.canvas.program.layers.3.solutions.clear();
                    for (i, proposal) in &node.proposals {
                        let name = self.name_map.get(i).unwrap();
                        let r = self
                            .canvas
                            .program
                            .layers
                            .2
                            .agents
                            .get(name)
                            .unwrap()
                            .agent
                            .radius;
                        self.canvas
                            .program
                            .layers
                            .3
                            .solutions
                            .push((r, proposal.meta.trajectory.clone()));
                    }

                    if let Some(conceded) = node.conceded {
                        if let Some(name) = self.name_map.get(&conceded) {
                            let name = name.clone();
                            return Command::perform(async {}, move |_| {
                                Message::SelectAgent(name.clone())
                            });
                        }
                    }
                }
            }
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
            iced::time::every(std::time::Duration::from_millis(50)).map(|_| Message::Tick),
        ])
    }

    fn view(&mut self) -> Element<Self::Message> {
        let mut content = Column::new()
            .spacing(20)
            .align_items(Alignment::Start)
            .width(Length::Fill)
            .height(Length::Fill);

        let (agent_yaw, agent_radius, agent_speed, agent_spin) =
            if let Some(ctx) = self.canvas.program.layers.2.selected_agent() {
                (
                    ctx.agent.yaw,
                    ctx.agent.radius,
                    ctx.agent.speed,
                    ctx.agent.spin,
                )
            } else {
                (0.0, default_radius(), default_speed(), default_spin())
            };
        let yaw_tick = Self::get_tick(agent_yaw, MIN_AGENT_YAW, MAX_AGENT_YAW);
        let radius_tick = Self::get_tick(agent_radius, MIN_AGENT_RADIUS, MAX_AGENT_RADIUS);
        let speed_tick = Self::get_tick(agent_speed, MIN_AGENT_SPEED, MAX_AGENT_SPEED);
        let spin_tick = Self::get_tick(agent_spin, MIN_AGENT_SPIN, MAX_AGENT_SPIN);

        let file_row = Row::new()
            .spacing(20)
            .align_items(Alignment::Center)
            .width(Length::Fill)
            .push(
                Button::new(&mut self.select_file_button, iced::Text::new("Save As..."))
                    .on_press(Message::SaveFileAs),
            )
            .push({
                Button::new(&mut self.load_button, iced::Text::new("Load"))
                    .on_press(Message::LoadFile)
            })
            .push({
                let button = Button::new(&mut self.save_button, iced::Text::new("Save"));
                if !self.file_text_input_value.is_empty() {
                    button.on_press(Message::SaveFile)
                } else {
                    button
                }
            })
            .push(
                TextInput::new(
                    &mut self.file_text_input,
                    "Scenario File",
                    &mut self.file_text_input_value,
                    Message::ScenarioFileInput,
                )
                .padding(10),
            );

        let mut agent_names: Vec<String> = self
            .canvas
            .program
            .layers
            .2
            .agents
            .iter()
            .map(|(name, _)| name.clone())
            .collect();
        agent_names.sort_unstable();
        let robot_row = Row::new()
            .spacing(20)
            .align_items(Alignment::Center)
            .push(
                Button::new(&mut self.add_agent_button, iced::Text::new("Add Agent"))
                    .on_press(Message::AddAgent),
            )
            .push(
                Button::new(
                    &mut self.remove_agent_button,
                    iced::Text::new("Remove Agent"),
                )
                .on_press(Message::RemoveAgent),
            )
            .push(PickList::new(
                &mut self.pick_agent_state,
                agent_names,
                self.canvas.program.layers.2.selected_agent.clone(),
                Message::SelectAgent,
            ))
            .push(
                Column::new()
                    .width(Length::FillPortion(1))
                    .push(Text::new(format!("Yaw: {:.2} deg", agent_yaw)))
                    .push(Slider::new(
                        &mut self.agent_yaw_slider,
                        0..=Self::TICK_COUNT,
                        yaw_tick,
                        Message::AgentYawSlide,
                    )),
            )
            .push(
                Column::new()
                    .width(Length::FillPortion(1))
                    .push(Text::new(format!("Radius: {:.2} m", agent_radius)))
                    .push(Slider::new(
                        &mut self.agent_radius_slider,
                        0..=Self::TICK_COUNT,
                        radius_tick,
                        Message::AgentRadiusSlide,
                    )),
            )
            .push(
                Column::new()
                    .width(Length::FillPortion(1))
                    .push(Text::new(format!("Speed: {:.2} m/s", agent_speed)))
                    .push(Slider::new(
                        &mut self.agent_speed_slider,
                        0..=Self::TICK_COUNT,
                        speed_tick,
                        Message::AgentSpeedSlide,
                    )),
            )
            .push(
                Column::new()
                    .width(Length::FillPortion(1))
                    .push(Text::new(format!(
                        "Spin: {:.2} deg/sec",
                        agent_spin.to_degrees()
                    )))
                    .push(Slider::new(
                        &mut self.agent_spin_slider,
                        0..=Self::TICK_COUNT,
                        spin_tick,
                        Message::AgentSpinSlide,
                    )),
            );

        let instruction_row = Row::<Message>::new()
            .spacing(40)
            .align_items(Alignment::Center)
            .width(Length::Shrink)
            .push(
                Column::new()
                    .push(Text::new("Left click: Set start"))
                    .push(Text::new("Right click: Set goal")),
            )
            .push(
                Column::new()
                    .push(Text::new("Shift + Left click: Add occupancy"))
                    .push(Text::new("Shift + Right click: Remove occupancy")),
            )
            .push(
                Column::new()
                    .push(Text::new("Middle click: Pan view"))
                    .push(Text::new("Scroll: Zoom")),
            )
            .push(
                Button::new(&mut self.reset_view_button, iced::Text::new("Reset View"))
                    .on_press(Message::ResetView)
                    .height(Length::Shrink),
            )
            .push({
                if let Some(t) = self.canvas.program.layers.3.now {
                    Text::new(format!("t: {:.2}", t.as_secs_f64()))
                } else {
                    Text::new("t: off")
                }
            })
            .push(
                Button::new(&mut self.reset_time_button, iced::Text::new("Reset Time"))
                    .on_press(Message::ResetTime)
                    .height(Length::Shrink),
            );

        content = content.push(file_row).push(robot_row).push(instruction_row);

        const DEBUG_TEXT_SIZE: u16 = 15;
        if self.debug_planner_on {
            content = content.push(
                Row::new().push(self.canvas.view()).push(
                    Column::new()
                        .push(
                            Row::new()
                                .push(
                                    Button::new(&mut self.step_progress, Text::new("Step"))
                                        .on_press(Message::StepProgress),
                                )
                                .push(iced::Space::with_width(Length::Units(16)))
                                .push(Text::new(format!("Steps: {}", self.debug_step_count)))
                                .push(iced::Space::with_width(Length::Units(16)))
                                .push(Text::new(format!(
                                    "Queue size: {}",
                                    self.search_memory.len()
                                )))
                                .align_items(Alignment::Center),
                        )
                        .push(
                            {
                                let mut scroll =
                                    Scrollable::<Message>::new(&mut self.node_list_scroll);
                                if let Some((_, search)) = &self.search {
                                    for (i, ticket) in self.search_memory.iter().enumerate() {
                                        let node =
                                            search.memory().0.arena.get(ticket.node_id).unwrap();
                                        scroll = scroll.push(Radio::new(
                                            i,
                                            format!(
                                                "{i}: {:?} + {:?} = {:?}",
                                                node.cost().0,
                                                node.remaining_cost_estimate().0,
                                                ticket.evaluation.0,
                                            ),
                                            self.debug_node_selected,
                                            Message::SelectDebugNode,
                                        ));
                                    }
                                }
                                scroll
                            }
                            .height(Length::Fill),
                        )
                        .push(
                            {
                                Scrollable::<Message>::new(&mut self.debug_text_scroll).push(
                                    Text::new({
                                        if let Some(selection) = self.debug_node_selected {
                                            if let Some(node) = self.search_memory.get(selection) {
                                                if let Some((_, search)) = &self.search {
                                                    let node = search
                                                        .memory()
                                                        .0
                                                        .arena
                                                        .get(node.node_id)
                                                        .unwrap();
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
                                    })
                                    .size(DEBUG_TEXT_SIZE),
                                )
                            }
                            .height(Length::Fill),
                        ),
                ),
            );
        } else if !self.negotiation_history.is_empty() {
            content = content.push(
                Row::new()
                .push(self.canvas.view())
                .push(
                    Column::new()
                    .push(Text::new(format!("History size: {}", self.negotiation_history.len())))
                    .push({
                        let mut scroll = Scrollable::<Message>::new(&mut self.node_list_scroll);
                        for (i, node) in self.negotiation_history.iter().enumerate() {
                            scroll = scroll.push(Radio::new(
                                i,
                                format!("{}. Cost {:.2} | Keys: {} | Parent: {} | Conflicts: {}", node.id, node.cost.0, node.keys.len(), node.parent.unwrap_or(node.id), node.negotiation.conflicts.len()),
                                self.negotiation_node_selected,
                                Message::SelectNegotiationNode,
                            ));
                        }
                        scroll
                    }.height(Length::Fill))
                    .push(
                        Scrollable::<Message>::new(&mut self.debug_text_scroll)
                        .push(
                            Text::new({
                                if let Some(selection) = self.negotiation_node_selected {
                                    if let Some(node) = self.negotiation_history.get(selection) {
                                        let mut text = String::new();
                                        text += &format!("Keys: {}\n", node.keys.len());
                                        for key in &node.keys {
                                            text += &format!("{key:?}\n");
                                        }
                                        text += &format!("------\n{node:#?}");
                                        text
                                    } else {
                                        String::new()
                                    }
                                } else {
                                    String::new()
                                }
                            })
                            .size(DEBUG_TEXT_SIZE)
                        )
                        .height(Length::Fill)
                    )
                )
            )
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
    AgentYawSlide(u32),
    AgentRadiusSlide(u32),
    AgentSpeedSlide(u32),
    AgentSpinSlide(u32),
    AddAgent,
    RemoveAgent,
    SelectAgent(String),
    SaveFileAs,
    SaveFile,
    LoadFile,
    ScenarioFileInput(String),
    ResetView,
    ResetTime,
    SetView(InclusionZone),
    EventOccurred(iced_native::Event),
    EndpointSelected(EndpointSelection),
    OccupancyChanged,
    SelectDebugNode(usize),
    SelectNegotiationNode(usize),
    StepProgress,
    Tick,
}

fn main() -> iced::Result {
    let flags = Args::parse();
    App::run(iced::Settings::with_flags(flags))
}

mod style {
    use iced::container;
    pub struct Pane;
    impl container::StyleSheet for Pane {
        fn style(&self) -> container::Style {
            container::Style {
                text_color: None,
                background: None,
                border_radius: 0.0,
                border_width: 3.0,
                border_color: iced::Color {
                    r: 0.9,
                    g: 0.9,
                    b: 0.9,
                    a: 0.9,
                },
            }
        }
    }
}
