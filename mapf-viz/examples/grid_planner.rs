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
    pick_list::{self, PickList},
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
    planner::halt::MeasureLimit,
};
use mapf_viz::{
    SparseGridOccupancyVisual, InfiniteGrid,
    spatial_canvas::{SpatialCanvas, SpatialCanvasProgram, SpatialCache, InclusionZone},
    toggle::{Toggle, Toggler, KeyToggler},
};
use mapf_viz::spatial_layers;
use std::{
    collections::{HashMap, BTreeMap},
    sync::Arc,
};
use native_dialog::FileDialog;
use serde::{Serialize, Deserialize};
use clap::Parser;

const ASCII_UPPER: [char; 26] = [
    'A', 'B', 'C', 'D', 'E',
    'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y',
    'Z',
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

#[derive(Serialize, Deserialize, Clone, Debug, Copy)]
struct Agent {
    /// Start cell
    start: [i64; 2],
    /// Initial yaw of the robot
    yaw: f64,
    /// Goal cell
    goal: [i64; 2],
    /// Radius of the robot's footprint (meters)
    #[serde(default = " default_radius")]
    radius: f64,
    /// Translational speed of the robot (meters/sec)
    #[serde(default = "default_speed")]
    speed: f64,
    /// How fast the robot can spin (radians/sec)
    #[serde(default = "default_spin")]
    spin: f64,
}

#[derive(Serialize, Deserialize)]
struct Obstacle {
    /// Trajectory of the obstacle in terms of (time (s), x cell, y cell)
    trajectory: Vec<(f64, i64, i64)>,
    /// Radius of the obstacle
    #[serde(default = "default_radius")]
    radius: f64,
    #[serde(default = "bool_false", skip_serializing_if="is_false")]
    indefinite_start: bool,
    #[serde(default = "bool_false", skip_serializing_if="is_false")]
    indefinite_finish: bool,
}

impl Obstacle {
    fn new(radius: f64, trajectory: &LinearTrajectorySE2, cell_size: f64) -> Obstacle {
        Obstacle {
            trajectory: trajectory.iter()
                .map(|wp| {
                    let cell = Cell::from_point(wp.position.translation.vector.into(), cell_size);
                    (wp.time.as_secs_f64(), cell.x, cell.y)
                })
                .collect(),
            radius: radius,
            indefinite_start: trajectory.has_indefinite_initial_time(),
            indefinite_finish: trajectory.has_indefinite_finish_time(),
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Scenario {
    agents: BTreeMap<String, Agent>,
    obstacles: Vec<Obstacle>,
    // y -> [..x..]
    occupancy: HashMap<i64, Vec<i64>>,
    #[serde(default = "default_cell_size")]
    cell_size: f64,
    #[serde(skip_serializing_if="Option::is_none", default)]
    camera_bounds: Option<[[f32; 2]; 2]>,
}

pub fn default_radius() -> f64 {
    0.45
}

pub fn default_speed() -> f64 {
    0.75
}

pub fn default_spin() -> f64 {
    60_f64.to_radians()
}

pub fn default_cell_size() -> f64 {
    1.0
}

pub fn bool_false() -> bool {
    false
}

pub fn is_false(b: &bool) -> bool {
    !b
}

type SparseVisibility = Visibility<SparseGrid>;

fn serialize_grid(grid: &SparseGrid) -> HashMap<i64, Vec<i64>> {
    let mut ser: HashMap<i64, Vec<i64>> = HashMap::new();
    for cell in grid.occupied_cells() {
        ser.entry(cell.y).or_default().push(cell.x);
    }

    for (_, column) in &mut ser {
        column.sort();
    }

    ser
}

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
    pub agents: BTreeMap<String, AgentContext>,
    pub cell_size: f64,
    pub invalid_color: iced::Color,
    pub selected_color: iced::Color,
    pub start_color: iced::Color,
    pub goal_color: iced::Color,
    pub show_details: bool,
    pub selected_agent: Option<String>,
    shift: KeyToggler,
    _msg: std::marker::PhantomData<Message>,
}

#[derive(Debug, Clone)]
struct AgentContext {
    agent: Agent,
    start_visibility: Vec<Cell>,
    goal_visibility: Vec<Cell>,
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
            start_visibility: Vec::new(),
            goal_visibility: Vec::new(),
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
        Self{
            agents: agents.into_iter().map(|(name, a)| (name, AgentContext::new(a))).collect(),
            selected_agent,
            cell_size,
            invalid_color,
            selected_color,
            start_color,
            goal_color,
            show_details: false,
            shift: KeyToggler::for_key(keyboard::KeyCode::LShift),
            _msg: Default::default(),
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
        &mut self, endpoint: Endpoint,
        visibility: &SparseVisibility,
    ) {
        println!(" vvvvvvvvvvvvvvvvvvvvvvvvvv ");
        let Some(selected) = &self.selected_agent else { return };
        let cell: Cell = if let Some(ctx) = self.agents.get(selected) {
            endpoint.of(&ctx.agent)
        } else {
            return
        };

        // Start by assuming all are valid
        for (_, ctx) in &mut self.agents {
            endpoint.set_availability(ctx, true);
        }

        let cs = visibility.grid().cell_size();
        let p = cell.to_center_point(cs);
        if visibility.grid().is_square_occupied(p, 2.0*visibility.agent_radius()).is_some() {
            endpoint.set_availability(self.agents.get_mut(selected).unwrap(), false);
        }

        // Test O(N^2) for invalidity. We could make this way more efficient.
        let mut invalidated = Vec::new();
        triangular_for(self.agents.iter(), |(name_i, ctx_i), (name_j, ctx_j)| {
            let p_i = endpoint.of(&ctx_i.agent).to_center_point(cs);
            let p_j = endpoint.of(&ctx_j.agent).to_center_point(cs);
            let dist = (p_i - p_j).norm();
            let min_dist = ctx_i.agent.radius + ctx_j.agent.radius;
            dbg!(dist, min_dist);
            if dist < min_dist {
                invalidated.push((*name_i).clone());
                invalidated.push(name_j.clone());
            }
        });

        dbg!(&invalidated);
        for name in invalidated {
            let Some(ctx) = self.agents.get_mut(&name) else { continue };
            endpoint.set_availability(ctx, false);
            dbg!((name, ctx.start_open, ctx.goal_open));
        }
        println!(" ^^^^^^^^^^^^^^^^^^^^^^^^^^^ ");
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
                Endpoint::Start => { ctx.start_open = valid; },
                Endpoint::Goal => { ctx.goal_open = valid; },
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

    pub fn calculate_visibility(&mut self, visibility: &SparseVisibility) {
        for (_, ctx) in &mut self.agents {
            for (cell, validity, visible) in [
                (ctx.agent.start, &mut ctx.start_open, &mut ctx.start_visibility),
                (ctx.agent.goal, &mut ctx.goal_open, &mut ctx.goal_visibility),
            ] {
                let cell: Cell = cell.into();
                let p = cell.to_center_point(visibility.grid().cell_size());
                *validity = visibility.grid().is_square_occupied(
                    p, 2.0*ctx.agent.radius,
                ).is_none();
                *visible = visibility.calculate_visibility(cell).collect();
            }

            ctx.start_sees_goal = false;
            if ctx.start_open && ctx.goal_open {
                let cell_start: Cell = ctx.agent.start.into();
                let cell_goal: Cell = ctx.agent.goal.into();
                let p_start = cell_start.to_center_point(self.cell_size);
                let p_goal = cell_goal.to_center_point(self.cell_size);
                ctx.start_sees_goal = visibility.grid().is_sweep_occupied(
                    p_start, p_goal, 2.0*ctx.agent.radius
                ).is_none();
            }
        }
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
                    if self.shift.state() == Toggle::Off {
                        let endpoint = match event {
                            mouse::Event::ButtonPressed(mouse::Button::Left) => Endpoint::Start,
                            mouse::Event::ButtonPressed(mouse::Button::Right) => Endpoint::Goal,
                            _ => return (SpatialCache::Unchanged, canvas::event::Status::Ignored, None),
                        };

                        let cell_size = self.cell_size;
                        if let Some(cell) = self.endpoint_cell_mut(endpoint) {
                            *cell = Cell::from_point([p.x as f64, p.y as f64].into(), cell_size).into();
                        }

                        return (
                            SpatialCache::Refresh,
                            canvas::event::Status::Captured,
                            Some(Message::EndpointSelected(
                                EndpointSelection::Cell(endpoint)
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
        for (name, ctx) in &self.agents {
            if self.show_details {
                if ctx.start_sees_goal {
                    let cell_s: Cell = ctx.agent.start.into();
                    let cell_g: Cell = ctx.agent.goal.into();
                    let p_start = cell_s.to_center_point(self.cell_size);
                    let p_goal = cell_g.to_center_point(self.cell_size);
                    frame.stroke(
                        &Path::line(
                            [p_start.x as f32, p_start.y as f32].into(),
                            [p_goal.x as f32, p_goal.y as f32].into()
                        ),
                        Stroke{
                            color: self.selected_color,
                            width: 5_f32,
                            ..Default::default()
                        }
                    );
                }
            }

            for (cell, angle, color, valid, visibility) in [
                (ctx.agent.start, Some(ctx.agent.yaw), self.start_color, ctx.start_valid(), &ctx.start_visibility),
                (ctx.agent.goal, None, self.goal_color, ctx.goal_valid(), &ctx.goal_visibility)
            ] {
                let radius = ctx.agent.radius as f32;
                let cell: Cell = cell.into();
                let p = cell.to_center_point(self.cell_size);
                if self.show_details {
                    for v_cell in visibility {
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

                draw_agent(frame, p, angle, radius, color);
                let is_selected = self.selected_agent.as_ref().filter(|n| **n == *name).is_some();
                if !valid || is_selected {
                    let color = if !valid {
                        self.invalid_color
                    } else {
                        self.selected_color
                    };

                    frame.stroke(
                        &Path::circle([p.x as f32, p.y as f32].into(), radius),
                        Stroke{
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
        _cursor: canvas::Cursor
    ) {
        for (name, ctx) in &self.agents {
            for (kind, cell) in [("Start", ctx.agent.start), ("Goal", ctx.agent.goal)] {
                let cell: Cell = cell.into();
                let p = cell.to_center_point(self.cell_size);
                let r = ctx.agent.radius as f32;
                let delta = iced::Vector::new(r, -r);
                let p = iced::Point::new(p.x as f32, p.y as f32) + delta;

                if bounds.contains(p) {
                    hud.at(
                        p,
                        |frame| {
                            frame.fill_text(format!("{} {}: ({}, {})", name, kind, cell.x, cell.y).to_owned());
                        }
                    )
                }
            }
        }
    }

    fn estimate_bounds(&self) -> mapf_viz::spatial_canvas::InclusionZone {
        let mut zone = InclusionZone::Empty;
        for (_, ctx) in &self.agents {
            for endpoint in [ctx.agent.start, ctx.agent.goal] {
                let cell: Cell = endpoint.into();
                let p = cell.to_center_point(self.cell_size);
                for v in [[1.0, 1.0], [-1.0, -1.0]] {
                    let v: Vector = v.into();
                    let r = p + ctx.agent.radius*v;
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

        let t_obs = Trajectory::from_iter([
            WaypointSE2::new(TimePoint::from_secs_f64(0.0), 10.0, 0.0, 180_f64.to_radians()),
            // WaypointSE2::new_f64(10.0, -5.0, 0.0, 180_f64.to_radians()),
            WaypointSE2::new_f64(5.0, 0.0, 0.0, 180_f64.to_radians()),
            WaypointSE2::new_f64(8.0, 0.0, 0.0, -90_f64.to_radians()),
            WaypointSE2::new_f64(18.0, 0.0, -10.0, -90_f64.to_radians()),
        ]).unwrap();

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
        _spatial_cursor: canvas::Cursor
    ) {
        if let Some((t0, _)) = self.time_range() {

            for (_, trajectory) in &self.searches {
                draw_trajectory(frame, trajectory, self.search_color);
            }

            for (_, trajectory) in &self.solutions {
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
        _spatial_cursor: canvas::Cursor
    ) {
        for (radius, trajectory) in &self.solutions {
            let mut sequence: HashMap<Cell, Vec<String>> = HashMap::new();
            for (i, wp) in trajectory.iter().enumerate() {
                sequence.entry(
                    Cell::from_point(
                        Point::from(wp.position.translation.vector), self.cell_size
                    )
                ).or_default().push(i.to_string());
            }

            let r = *radius as f32 / 2_f32.sqrt();
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
    pick_agent_state: pick_list::State<String>,
    reset_view_button: button::State,
    canvas: SpatialCanvas<Message, GridLayers>,
    node_list_scroll: scrollable::State,
    debug_text_scroll: scrollable::State,
    show_details: KeyToggler,
    search: Option<(f64, Search<MyAlgo, GoalSE2<Cell>, MeasureLimit>)>,
    step_progress: button::State,
    debug_on: bool,
    memory: Vec<TreeTicket>,
    debug_step_count: u64,
    debug_node_selected: Option<usize>,
}

impl App {
    const TICK_COUNT: u32 = 1000;

    fn get_tick(x: f64, min: f64, max: f64) -> u32 {
        ((x - min)/(max - min) * Self::TICK_COUNT as f64) as u32
    }

    fn from_tick(x: u32, min: f64, max: f64) -> f64 {
        x as f64 * (max - min)/(Self::TICK_COUNT as f64) + min
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

    fn set_robot_radius(&mut self, radius: f64) {
        self.canvas.program.layers.1.set_robot_radius(radius as f32);
        let endpoint_selector = &mut self.canvas.program.layers.2;
        if endpoint_selector.set_agent_radius(radius) {
            for endpoint in [Endpoint::Start, Endpoint::Goal] {
                if let Some(cell) = endpoint_selector.endpoint_cell(endpoint) {
                    let cell: Cell = (*cell).into();
                    let p = cell.to_center_point(endpoint_selector.cell_size);
                    let valid = self.canvas.program.layers.1.grid().is_square_occupied(p, 2.0*radius).is_none();
                    endpoint_selector.set_endpoint_valid(endpoint, valid);
                }
            }
            self.recalculate_visibility();
            self.canvas.cache.clear();
        }

        self.update_all_endpoints();
    }

    fn update_all_endpoints(&mut self) {
        self.canvas.program.layers.2.update_endpoint_conflicts(
            Endpoint::Start,
            &self.canvas.program.layers.1.visibility(),
        );
        self.canvas.program.layers.2.update_endpoint_conflicts(
            Endpoint::Goal,
            &self.canvas.program.layers.1.visibility(),
        );
    }

    fn recalculate_visibility(&mut self) {
        self.canvas.program.layers.2.calculate_visibility(self.canvas.program.layers.1.visibility());
    }

    fn step_progress(&mut self) {
        if let Some((radius, search)) = &mut self.search {
            self.debug_step_count += 1;

            self.memory = search.memory().0
                .queue.clone().into_iter_sorted()
                .map(|n| n.0.clone()).collect();

            for ticket in &self.memory {
                if let Some(traj) = search.memory().0
                    .arena.retrace(ticket.node_id).unwrap()
                    .make_trajectory().unwrap()
                {
                    self.canvas.program.layers.3.searches.push((*radius, traj));
                }
            }

            if let SearchStatus::Solved(solution) = search.step().unwrap() {
                println!("Solution: {:#?}", solution);
                self.canvas.program.layers.3.solutions.clear();
                self.canvas.program.layers.3.solutions.extend(solution.make_trajectory().unwrap().map(|t| (*radius, t)).into_iter());
                self.debug_node_selected = None;
                self.search = None;
            } else {
                if let Some(selection) = self.debug_node_selected {
                    if let Some(node) = self.memory.get(selection) {
                        let solution = search
                            .memory().0.arena
                            .retrace(node.node_id)
                            .unwrap()
                            .make_trajectory()
                            .unwrap();

                        self.canvas.program.layers.3.solutions.clear();
                        self.canvas.program.layers.3.solutions.extend(solution.map(|s| (*radius, s)).into_iter());
                    }
                }
            }

            self.canvas.cache.clear();
        }
    }

    fn add_agent(&mut self) {
        let name = generate_robot_name(self.canvas.program.layers.2.agents.len());
        let start = [-5, 0];
        let yaw = 0.0;
        let goal = [5, 0];
        let agent = match self.canvas.program.layers.2.selected_agent() {
            Some(ctx) => {
                Agent {
                    start,
                    goal,
                    yaw,
                    ..ctx.agent
                }
            }
            None => {
                Agent {
                    start,
                    goal,
                    yaw,
                    radius: default_radius(),
                    speed: default_speed(),
                    spin: default_spin(),
                }
            }
        };

        self.canvas.program.layers.2.agents.insert(name.clone(), AgentContext::new(agent));
        self.canvas.program.layers.2.selected_agent = Some(name);
        self.canvas.program.layers.1.set_robot_radius(agent.radius as f32);
        self.recalculate_visibility();
        self.canvas.cache.clear();
    }

    fn generate_plan(&mut self) {
        // Clear all history
        self.memory.clear();
        self.search = None;
        self.canvas.program.layers.3.solutions.clear();
        self.canvas.program.layers.3.searches.clear();
        self.canvas.cache.clear();

        let endpoints = &self.canvas.program.layers.2;
        let visibility = self.canvas.program.layers.1.visibility();
        let mut other_agents: Vec<(f64, LinearTrajectorySE2)> = Vec::new();
        for (name, ctx) in &endpoints.agents {
            let start_cell: Cell = ctx.agent.start.into();
            let goal_cell: Cell = ctx.agent.goal.into();

            dbg!((name, ctx.start_open, ctx.goal_open));
            if !ctx.endpoints_valid() {
                print!("Cannot plan for {name}:");
                if !ctx.start_open {
                    print!(" invalid start");
                }
                if !ctx.start_available {
                    print!(" start conflict");
                }
                if !ctx.goal_open {
                    print!(" invalid goal");
                }
                if !ctx.goal_available {
                    print!(" goal conflict");
                }
                println!("");
                continue;
            }

            let r = ctx.agent.radius;
            self.canvas.program.layers.3.vertex_lookup.clear();

            let shared_visibility = Arc::new(visibility.clone());
            let heuristic_graph = SharedGraph::new(VisibilityGraph::new(
                shared_visibility.clone(), [],
            ));

            let activity_graph = SharedGraph::new(NeighborhoodGraph::new(
                shared_visibility, [],
            ));

            let extrapolator = DifferentialDriveLineFollow::new(ctx.agent.speed, ctx.agent.spin).expect("Bad speeds");
            let agent_radius = ctx.agent.radius;
            let profile = CircularProfile::new(
                agent_radius, agent_radius, agent_radius,
            ).expect("Bad profile sizes");

            let environment = Arc::new(
                OverlayedDynamicEnvironment::new(
                    Arc::new({
                        let mut env = DynamicEnvironment::new(profile);
                        for (obs_size, obs_traj) in self.canvas.program.layers.3.obstacles.iter().chain(other_agents.iter()) {
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
                orientation: Orientation::new(ctx.agent.yaw),
            };

            let goal = GoalSE2 {
                key: goal_cell,
                orientation: None,
            };

            println!("About to plan for {name}:\nStart: {start:#?}\nGoal: {goal:#?}");

            let start_time = std::time::Instant::now();
            let planner = Planner::new(AStarConnect(domain))
                .with_halting(MeasureLimit(Some(10000000)));
                // .with_halting(MeasureLimit(None));
            let mut search = planner.plan(start, goal).unwrap();

            self.canvas.program.layers.3.searches.clear();
            self.debug_step_count = 0;
            if self.debug_on {
                self.memory = search.memory().0
                    .queue.clone().into_iter_sorted()
                    .map(|n| n.0.clone()).collect();
                self.search = Some((r, search));
            } else {
                let result = search.solve().unwrap();
                let elapsed_time = start_time.elapsed();
                match result {
                    SearchStatus::Solved(solution) => {
                        // println!("Solution: {:#?}", solution);
                        // println!(" ======================= ");
                        let trajectory = solution.make_trajectory().unwrap().map(|t|
                            t
                            .with_indefinite_initial_time(true)
                            .with_indefinite_finish_time(true)
                        );
                        let arrival_time = trajectory.as_ref().map(|t| t.motion_duration().as_secs_f64()).unwrap_or(0.0);
                        let presence = trajectory.map(|t| (r, t));
                        self.canvas.program.layers.3.solutions.extend(presence.as_ref().into_iter().cloned());
                        other_agents.extend(presence.into_iter());
                        // println!(" ======================= ");
                        // println!("Trajectory: {:#?}", self.canvas.program.layers.3.solutions);
                        println!(
                            "{name} Arrival time: {arrival_time:?}, cost: {:?}",
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
        }
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

fn load_scenario(filename: Option<&String>) -> (
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
            obstacles = scenario.obstacles.into_iter().filter_map(|obs| {
                LinearTrajectorySE2::from_iter(
                    obs.trajectory.into_iter()
                    .map(|(t, x, y)| {
                        let p = Cell::new(x, y).to_center_point(cell_size);
                        WaypointSE2::new_f64(t, p.x, p.y, 0.0)
                    })
                ).ok()
                .map(|t|
                    t
                    .with_indefinite_initial_time(obs.indefinite_start)
                    .with_indefinite_finish_time(obs.indefinite_finish)
                )
                .map(|t| (obs.radius, t))
            }).collect();
            for (y, row) in scenario.occupancy {
                for x in row {
                    grid.change_cells(
                        &[(Cell::new(x, y), true)].into_iter().collect()
                    );
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

        let mut canvas = SpatialCanvas::new(
            GridLayers{
                layers: (
                    InfiniteGrid::new(cell_size),
                    SparseGridOccupancyVisual::new(
                        grid,
                        default_radius() as f32,
                        None,
                        Some(Box::new(|| { Message::OccupancyChanged })),
                    ).showing_visibility_graph(false),
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
                )
            }
        );
        canvas.zoom = 20.0;

        let mut app = Self {
            save_button: button::State::new(),
            load_button: button::State::new(),
            select_file_button: button::State::new(),
            file_text_input: text_input::State::new(),
            file_text_input_value: String::new(),
            agent_yaw_slider: slider::State::new(),
            agent_radius_slider: slider::State::new(),
            agent_speed_slider: slider::State::new(),
            agent_spin_slider: slider::State::new(),
            add_agent_button: button::State::new(),
            pick_agent_state: pick_list::State::new(),
            reset_view_button: button::State::new(),
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
        };

        if app.canvas.program.layers.2.agents.is_empty() {
            // Calling add_agent also recalculates visibility
            app.add_agent();
            zone.include(iced::Point::new(-10.0, -10.0));
            zone.include(iced::Point::new(10.0, 10.0));
        }

        app.update_all_endpoints();
        app.recalculate_visibility();

        let set_view = Command::perform(async move {}, move |_| Message::SetView(zone));
        let top_agent = app.canvas.program.layers.2.agents.iter().next();
        let set_agent = if let Some((top_agent, _)) = top_agent {
            let top_agent = top_agent.clone();
            Some(Command::perform(async move {}, move |_| Message::SelectAgent(top_agent.clone())))
        } else {
            None
        };

        (app, Command::batch([set_view].into_iter().chain(set_agent.into_iter())))
    }

    fn title(&self) -> String {
        "Grid Planner".to_owned()
    }

    fn update(
        &mut self,
        message: Self::Message
    ) -> Command<Self::Message> {
        match message {
            Message::AddAgent => {
                self.add_agent();
                self.generate_plan();
            }
            Message::SaveFile | Message::SaveFileAs => {
                if matches!(message, Message::SaveFileAs) {
                    match FileDialog::new().show_save_single_file() {
                        Ok(f) => match f {
                            Some(f) => self.file_text_input_value = f.as_path().as_os_str().to_str().unwrap().to_owned(),
                            None => return Command::none(),
                        }
                        Err(err) => {
                            println!("Unable to select file: {err:?}");
                            return Command::none();
                        }
                    }
                }

                let out_file = match std::fs::File::create(&self.file_text_input_value) {
                    Ok(r) => r,
                    Err(err) => {
                        println!("Unable to save to file {}: {:?}", self.file_text_input_value, err);
                        return Command::none();
                    }
                };

                let camera_bounds = match self.canvas.camera_bounds() {
                    InclusionZone::Empty => None,
                    InclusionZone::Some { lower, upper } => Some([[lower.x, lower.y], [upper.x, upper.y]])
                };

                let cell_size = self.canvas.program.layers.2.cell_size;
                let scenario = Scenario {
                    agents: self.canvas.program.layers.2.agents.iter().map(|(n, a)| (n.clone(), a.agent.clone())).collect(),
                    obstacles: self.canvas.program.layers.3.obstacles.iter().map(|obs| Obstacle::new(obs.0, &obs.1, cell_size)).collect(),
                    occupancy: serialize_grid(self.canvas.program.layers.1.grid()),
                    cell_size,
                    camera_bounds,
                };

                match serde_yaml::to_writer(out_file, &scenario) {
                    Ok(()) => {}
                    Err(err) => {
                        println!("Unable to save to file {}: {:?}", self.file_text_input_value, err);
                        return Command::none();
                    }
                }
            }
            Message::LoadFile => {
                match FileDialog::new().show_open_single_file() {
                    Ok(f) => match f {
                        Some(value) => self.file_text_input_value = value.as_path().as_os_str().to_str().unwrap().to_owned(),
                        None => return Command::none(),
                    }
                    Err(err) => {
                        println!("Unable to load selected file: {err:?}");
                        return Command::none();
                    }
                }

                let (agents, obstacles, grid, zone, success) = load_scenario(Some(&self.file_text_input_value));
                if !success {
                    return Command::none();
                }

                self.canvas.program.layers.1.set_grid(grid);
                self.canvas.program.layers.2.agents = agents.into_iter().map(|(n, a)| (n, AgentContext::new(a))).collect();
                self.canvas.program.layers.3.obstacles = obstacles;
                self.update_all_endpoints();
                self.recalculate_visibility();
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
                        self.canvas.program.layers.1.set_robot_radius(ctx.agent.radius as f32);
                        self.recalculate_visibility();
                        self.canvas.cache.clear();
                        self.generate_plan();
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
            Message::EventOccurred(event) => {
                if let iced_native::Event::Keyboard(event) = event {
                    match self.show_details.key_toggle(event) {
                        Toggle::On => {
                            self.canvas.program.layers.1.show_visibility_graph = true;
                            self.canvas.program.layers.1.show_details(true);
                            self.canvas.program.layers.2.show_details = true;
                            self.canvas.cache.clear();
                        },
                        Toggle::Off => {
                            self.canvas.program.layers.1.show_visibility_graph = false;
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
                        self.canvas.program.layers.2.update_endpoint_conflicts(
                            endpoint,
                            self.canvas.program.layers.1.visibility(),
                        );
                        self.recalculate_visibility();
                    }
                }
                self.generate_plan();
            },
            Message::OccupancyChanged => {
                self.recalculate_visibility();
                self.generate_plan();
            },
            Message::DebugNodeSelected(value) => {
                if self.memory.is_empty() {
                    self.debug_node_selected = Some(0);
                } else {
                    self.debug_node_selected = Some(usize::min(value, self.memory.len() - 1));
                }

                if let Some(node) = self.memory.get(value) {
                    if let Some((r, search)) = &self.search {
                        let solution = search
                            .memory().0.arena
                            .retrace(node.node_id)
                            .unwrap()
                            .make_trajectory()
                            .unwrap();
                        self.canvas.program.layers.3.solutions.extend(solution.map(|s| (*r, s)).into_iter());
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

        // let agent_radius = self.canvas.program.layers.1.visibility().agent_radius() as f32;
        let (agent_yaw, agent_radius, agent_speed, agent_spin) = if let Some(ctx) = self.canvas.program.layers.2.selected_agent() {
            (ctx.agent.yaw, ctx.agent.radius, ctx.agent.speed, ctx.agent.spin)
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
                Button::new(
                    &mut self.select_file_button,
                    iced::Text::new("Save As...")
                ).on_press(Message::SaveFileAs)
            )
            .push({
                Button::new(
                    &mut self.load_button,
                    iced::Text::new("Load")
                ).on_press(Message::LoadFile)
            })
            .push({
                let button = Button::new(
                    &mut self.save_button,
                    iced::Text::new("Save")
                );
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
                    Message::ScenarioFileInput
                )
                .padding(10)
            );

        let mut agent_names: Vec<String> = self.canvas.program.layers.2.agents.iter().map(|(name, _)| name.clone()).collect();
        agent_names.sort();
        let robot_row = Row::new()
            .spacing(20)
            .align_items(Alignment::Center)
            .push(
                Button::new(
                    &mut self.add_agent_button,
                    iced::Text::new("Add Agent"),
                ).on_press(Message::AddAgent)
            )
            .push(
                PickList::new(
                    &mut self.pick_agent_state,
                    agent_names,
                    self.canvas.program.layers.2.selected_agent.clone(),
                    Message::SelectAgent,
                )
            )
            .push(
                Column::new()
                .width(Length::FillPortion(1))
                .push(Text::new(format!("Yaw: {:.2} deg", agent_yaw)))
                .push(
                    Slider::new(
                        &mut self.agent_yaw_slider,
                        0..=Self::TICK_COUNT,
                        yaw_tick,
                        Message::AgentYawSlide
                    )
                )
            )
            .push(
                Column::new()
                .width(Length::FillPortion(1))
                .push(Text::new(format!("Radius: {:.2} m", agent_radius)))
                .push(
                    Slider::new(
                        &mut self.agent_radius_slider,
                        0..=Self::TICK_COUNT,
                        radius_tick,
                        Message::AgentRadiusSlide,
                    )
                )
            )
            .push(
                Column::new()
                .width(Length::FillPortion(1))
                .push(Text::new(format!("Speed: {:.2} m/s", agent_speed)))
                .push(
                    Slider::new(
                        &mut self.agent_speed_slider,
                        0..=Self::TICK_COUNT,
                        speed_tick,
                        Message::AgentSpeedSlide,
                    )
                )
            )
            .push(
                Column::new()
                .width(Length::FillPortion(1))
                .push(Text::new(format!("Spin: {:.2} deg/sec", agent_spin.to_degrees())))
                .push(
                    Slider::new(
                        &mut self.agent_spin_slider,
                        0..=Self::TICK_COUNT,
                        spin_tick,
                        Message::AgentSpinSlide,
                    )
                )
            );

        let instruction_row = Row::<Message>::new()
            .spacing(40)
            .align_items(Alignment::Center)
            .width(Length::Shrink)
            .push(
                Column::new()
                .push(Text::new("Left click: Set start"))
                .push(Text::new("Right click: Set goal"))
            )
            .push(
                Column::new()
                .push(Text::new("Shift + Left click: Add occupancy"))
                .push(Text::new("Shift + Right click: Remove occupancy"))
            )
            .push(
                Column::new()
                .push(Text::new("Middle click: Pan view"))
                .push(Text::new("Scroll: Zoom"))
            )
            .push(
                Button::new(
                    &mut self.reset_view_button,
                    iced::Text::new("Reset View")
                )
                .on_press(Message::ResetView)
                .height(Length::Shrink)
            );

        content = content
            .push(file_row)
            .push(robot_row)
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
                        if let Some((_, search)) = &self.search {
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
                                        if let Some((_, search)) = &self.search {
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
    AgentYawSlide(u32),
    AgentRadiusSlide(u32),
    AgentSpeedSlide(u32),
    AgentSpinSlide(u32),
    AddAgent,
    SelectAgent(String),
    SaveFileAs,
    SaveFile,
    LoadFile,
    ScenarioFileInput(String),
    ResetView,
    SetView(InclusionZone),
    EventOccurred(iced_native::Event),
    EndpointSelected(EndpointSelection),
    OccupancyChanged,
    DebugNodeSelected(usize),
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
