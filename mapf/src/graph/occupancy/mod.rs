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

use crate::util::triangular_for;
use bitfield::{bitfield, Bit, BitMut};
use std::collections::{hash_map, HashMap, HashSet};
use std::ops::Sub;
use util::LineSegment;

pub type Point = nalgebra::geometry::Point2<f64>;
pub type Vector = nalgebra::Vector2<f64>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cell {
    pub x: i64,
    pub y: i64,
}

impl Cell {
    /// Make a new cell from a pair of indices.
    pub fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }

    /// Get the cell that this point is inside of. Points that are perfectly on
    /// the edge between two cells will be biased towards the cell with the
    /// higher index value.
    pub fn from_point(p: Point, cell_size: f64) -> Self {
        Self {
            x: (p.x / cell_size).floor() as i64,
            y: (p.y / cell_size).floor() as i64,
        }
    }

    /// Get the point on the "bottom left" (lowest coordinate values) corner of
    /// the cell.
    pub fn to_bottom_left_point(&self, cell_size: f64) -> Point {
        Point::new(cell_size * self.x as f64, cell_size * self.y as f64)
    }

    /// Get the point in the center of the cell.
    pub fn to_center_point(&self, cell_size: f64) -> Point {
        Point::new(
            cell_size * (self.x as f64 + 0.5),
            cell_size * (self.y as f64 + 0.5),
        )
    }

    /// Get a new cell that is the same as this one, but shifted in x and y by
    /// the given values.
    pub fn shifted(&self, x: i64, y: i64) -> Self {
        Self {
            x: self.x + x,
            y: self.y + y,
        }
    }

    pub fn in_visible_quadrant_of(&self, other_cell: &Cell, other_status: CornerStatus) -> bool {
        if let Some(corner) = Corner::from_direction(other_cell.x - self.x, other_cell.y - self.y) {
            // If self is in a quadrant that other_cell is a corner relative to,
            // then we should not try to make any connection between the two.
            return !other_status.get(corner);
        }

        return true;
    }
}

impl Sub for Cell {
    type Output = (i64, i64);
    fn sub(self, other: Self) -> Self::Output {
        (self.x - other.x, self.y - other.y)
    }
}

/// Indicates what type of type of corner a cell is. Each value in the tuple can
/// be +1 or -1. The pair of values gives an (x, y) direction indicating where
/// the corner is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Corner(i8, i8);

impl Corner {
    fn from_direction(x: i64, y: i64) -> Option<Self> {
        if x == 0 || y == 0 {
            return None;
        }

        return Some(Corner(x.signum() as i8, y.signum() as i8));
    }
}

impl Into<u8> for Corner {
    fn into(self) -> u8 {
        assert!((self.0 == 1 || self.0 == -1) && (self.1 == 1 || self.1 == -1));
        return ((self.0 + 1) / 2 + (self.1 + 1)) as u8;
    }
}

impl Into<usize> for Corner {
    fn into(self) -> usize {
        let as_u8: u8 = self.into();
        return as_u8.into();
    }
}

impl From<u8> for Corner {
    fn from(bit: u8) -> Self {
        assert!(bit < 4);
        let q = (bit / 2) as i8;
        let r = (bit % 2) as i8;
        return Corner(2 * r - 1, 2 * q - 1);
    }
}

bitfield! {
    /// Indicates what type of type of corner a cell is. This is implemented as
    /// a bitfield, so you can call northwest(), southeast(), etc to get a
    /// boolean which indicates whether this is a corner in that direction.
    #[derive(Clone, Copy, PartialEq)]
    pub struct CornerStatus(u8);
    impl Debug;
    u8;
    pub southwest, set_southwest: 0;
    pub southeast, set_southeast: 1;
    pub northwest, set_northwest: 2;
    pub northeast, set_northeast: 3;
}

impl Default for CornerStatus {
    fn default() -> Self {
        Self(0)
    }
}

impl CornerStatus {
    pub fn is_corner(&self) -> bool {
        return self.0 != 0;
    }

    pub fn set(&mut self, corner: Corner, status: bool) {
        self.set_bit(corner.into(), status);
    }

    pub fn get(&self, corner: Corner) -> bool {
        return self.bit(corner.into());
    }

    pub fn iter(&self) -> CornerStatusIter {
        return CornerStatusIter {
            status: *self,
            next_bit: 0,
        };
    }
}

pub struct CornerStatusIter {
    status: CornerStatus,
    next_bit: u8,
}

impl Iterator for CornerStatusIter {
    type Item = (Corner, bool);

    fn next(&mut self) -> Option<Self::Item> {
        if self.next_bit > 3 {
            return None;
        }

        let current_bit = self.next_bit;
        self.next_bit += 1;
        return Some((current_bit.into(), self.status.bit(current_bit as usize)));
    }
}

impl IntoIterator for CornerStatus {
    type Item = (Corner, bool);
    type IntoIter = CornerStatusIter;

    fn into_iter(self) -> Self::IntoIter {
        return self.iter();
    }
}

impl IntoIterator for &CornerStatus {
    type Item = (Corner, bool);
    type IntoIter = CornerStatusIter;

    fn into_iter(self) -> Self::IntoIter {
        return self.iter();
    }
}

type ConfirmedChanges = Vec<(Cell, bool)>;
type ChangedCorners = Vec<(Cell, CornerStatus)>;

pub trait Grid: std::fmt::Debug {
    type OccupiedIterator<'a>: IntoIterator<Item = &'a Cell>
    where
        Cell: 'a,
        Self: 'a;

    type CornerIterator<'a>: IntoIterator<Item = (&'a Cell, &'a CornerStatus)>
    where
        Cell: 'a,
        CornerStatus: 'a,
        Self: 'a;

    /// Change the occupancy value of a set of cells. Get back any changes that
    /// have occurred to the corners of the occupancy.
    fn change_cells(&mut self, changes: &HashMap<Cell, bool>)
        -> (ConfirmedChanges, ChangedCorners);

    /// Get the size (width and height) of a cell.
    fn cell_size(&self) -> f64;

    /// Check if a single cell is occupied.
    fn is_occupied(&self, cell: &Cell) -> bool;

    /// Get an iterator over all occupied cells.
    fn occupied_cells<'b>(&'b self) -> Self::OccupiedIterator<'b>;

    /// Get an iterator over all corners.
    fn corners<'b>(&'b self) -> Self::CornerIterator<'b>;

    /// Check if a point is occupied. If it is, return the cell that occupies it.
    fn is_point_occupied(&self, p: Point) -> Option<Cell>;

    /// Check if a square has any occupancy. The first cell found that occupies
    /// the space will be returned.
    fn is_square_occupied(&self, p: Point, width: f64) -> Option<Cell>;

    /// Check if a rectangular sweep from p0 to p1 with the specified width has
    /// any occupancy. The first cell found that occupies the space will be
    /// returned.
    ///
    /// If p0 is almost equal to p1 then this will simply return the result of
    /// is_point_occupied(p0), because there is no way to infer what direction
    /// is meant to span the width of the sweep.
    fn is_sweep_occupied(&self, p0: Point, p1: Point, width: f64) -> Option<Cell>;
}

type BlockedBy = Option<Cell>;

#[derive(Debug, Clone)]
pub struct Visibility<G: Grid> {
    grid: G,
    agent_radius: f64,
    cell_shift: i64,
    points: HashMap<Cell, (BlockedBy, CornerStatus)>,
    edges: HashMap<Cell, HashMap<Cell, BlockedBy>>,
}

impl<G: Grid> Visibility<G> {
    /// Create a new visibility graph for the given grid and radius. The radius
    /// cannot be changed without recalculating the entire visibility graph.
    pub fn new(grid: G, agent_radius: f64) -> Self {
        let cell_size = grid.cell_size();
        let mut output = Self {
            grid,
            agent_radius,
            cell_shift: Self::calculate_cell_shift(agent_radius, cell_size),
            points: HashMap::new(),
            edges: HashMap::new(),
        };

        Self::update_corners(
            &output.grid,
            &Vec::new(),
            output.grid.corners(),
            output.agent_radius,
            output.cell_shift,
            &mut output.points,
            &mut output.edges,
        );

        return output;
    }

    /// Change the values for a set of cells.
    // TODO(MXG): Have this return more detailed information about what changed
    // instead of only returning a boolean.
    pub fn change_cells(&mut self, changes: &HashMap<Cell, bool>) -> bool {
        let (confirmed_changes, corner_changes) = self.grid.change_cells(changes);
        if confirmed_changes.is_empty() {
            // If none of the cells actually changed, then no corners should
            // have changed either.
            assert!(corner_changes.is_empty());

            // If no changes actually happened, then don't bother with the rest
            // of this function.
            return false;
        }

        return Self::update_corners(
            &self.grid,
            &confirmed_changes,
            corner_changes.iter().map(|(c, s)| (c, s)),
            self.agent_radius,
            self.cell_shift,
            &mut self.points,
            &mut self.edges,
        );
    }

    pub fn iter_points(&self) -> impl Iterator<Item = (&Cell, &CornerStatus)> {
        self.points
            .iter()
            .filter(|(_, (blocked_by, _))| blocked_by.is_none())
            .map(|(cell, (_, corner_status))| (cell, corner_status))
    }

    pub fn debug_points(&self) -> &HashMap<Cell, (BlockedBy, CornerStatus)> {
        return &self.points;
    }

    pub fn iter_edges(&self) -> VisibilityEdgeIter<'_, G> {
        VisibilityEdgeIter {
            visibility: self,
            point_iter: self.edges.iter(),
            edge_iter: None,
            pair_tracker: UniqueCellPairSet::new(),
        }
    }

    pub fn debug_edges(&self) -> &HashMap<Cell, HashMap<Cell, BlockedBy>> {
        return &self.edges;
    }

    pub fn calculate_visibility(&self, cell: Cell) -> impl Iterator<Item = Cell> + '_ {
        let visibility_edges = self.edges.get(&cell);
        [visibility_edges]
            .into_iter()
            .filter_map(|x| x)
            .flat_map(|edges| {
                edges
                    .into_iter()
                    .filter(|(_, blocked_by)| blocked_by.is_none())
                    .map(|(cell, _)| *cell)
            })
            .chain(
                // This chain kicks in when visibility_edges returned None, which
                // means we need to calculate the visibility for this cell.
                [visibility_edges]
                    .into_iter()
                    .filter(|x| x.is_none())
                    .flat_map(move |_| {
                        self.iter_points()
                            .filter(move |(v_cell, corner_status)| {
                                if !cell.in_visible_quadrant_of(*v_cell, **corner_status) {
                                    return false;
                                }

                                let cell_size = self.grid.cell_size();
                                let p0 = cell.to_center_point(cell_size);
                                let p1 = v_cell.to_center_point(cell_size);
                                return self
                                    .grid
                                    .is_sweep_occupied(p0, p1, 2.0 * self.agent_radius)
                                    .is_none();
                            })
                            .map(|(v_cell, _)| *v_cell)
                    }),
            )
    }

    pub fn neighbors(&self, of_cell: Cell) -> impl Iterator<Item = Cell> + '_ {
        // dbg!("neighbors");
        [of_cell]
            .into_iter()
            .filter(|of_cell| {
                self.grid()
                    .is_square_occupied(
                        of_cell.to_center_point(self.grid().cell_size()),
                        2.0 * self.agent_radius,
                    )
                    .is_none()
            })
            .flat_map(move |of_cell| {
                // dbg!(of_cell);
                [-1, 0, 1].into_iter().flat_map(move |i| {
                    // dbg!(i);
                    [-1, 0, 1]
                        .into_iter()
                        .filter(move |j| !(i == 0 && *j == 0))
                        .filter_map(move |j| {
                            // dbg!(j);
                            let cell_size = self.grid().cell_size();
                            let neighbor = of_cell.shifted(i, j);
                            if self
                                .grid()
                                .is_sweep_occupied(
                                    of_cell.to_center_point(cell_size),
                                    neighbor.to_center_point(cell_size),
                                    2.0 * self.agent_radius(),
                                )
                                .is_none()
                            {
                                Some(neighbor)
                            } else {
                                None
                            }
                        })
                })
            })
    }

    /// Get a reference to the underlying occupancy grid.
    pub fn grid(&self) -> &G {
        return &self.grid;
    }

    /// Take the grid from this visibility graph. This visibility graph cannot
    /// be used after this.
    pub fn take_grid(self) -> G {
        return self.grid;
    }

    pub fn agent_radius(&self) -> f64 {
        return self.agent_radius;
    }

    /// The whole visibility graph needs to be recalculated after this.
    pub fn change_agent_radius(&mut self, value: f64) {
        self.agent_radius = value;
        self.cell_shift = Self::calculate_cell_shift(self.agent_radius, self.grid().cell_size());
        self.points.clear();
        self.edges.clear();

        Self::update_corners(
            &self.grid,
            &Vec::new(),
            self.grid.corners(),
            self.agent_radius,
            self.cell_shift,
            &mut self.points,
            &mut self.edges,
        );
    }

    pub fn unstable<'a>(&'a self) -> UnstableVisibilityAPI<'a, G> {
        return UnstableVisibilityAPI { parent: self };
    }

    fn update_corners<'b>(
        grid: &G,
        confirmed_changes: &ConfirmedChanges,
        corners: impl IntoIterator<Item = (&'b Cell, &'b CornerStatus)>,
        agent_radius: f64,
        cell_shift: i64,
        points: &mut HashMap<Cell, (BlockedBy, CornerStatus)>,
        edges: &mut HashMap<Cell, HashMap<Cell, BlockedBy>>,
    ) -> bool {
        let mut changed = !confirmed_changes.is_empty();
        let mut new_points = Vec::new();
        for (base_cell, status) in corners {
            for (corner, valid) in status {
                let cell =
                    base_cell.shifted(cell_shift * corner.0 as i64, cell_shift * corner.1 as i64);

                if valid {
                    match points.entry(cell) {
                        hash_map::Entry::Vacant(entry) => {
                            // If this corner point is currently vacant, then we
                            // need to check whether it has any blockers.
                            let blocked_by = grid.is_square_occupied(
                                cell.to_center_point(grid.cell_size()),
                                2.0 * agent_radius,
                            );

                            entry
                                .insert((blocked_by, Default::default()))
                                .1
                                .set(corner, true);
                            new_points.push(cell);
                        }
                        hash_map::Entry::Occupied(mut entry) => {
                            entry.get_mut().1.set(corner, true);
                            let mut remove_connections = Vec::new();
                            for (other, _) in edges.entry(cell).or_default() {
                                if !other.in_visible_quadrant_of(&cell, entry.get().1) {
                                    remove_connections.push(*other);
                                }
                            }

                            for other in remove_connections {
                                edges.entry(cell).or_default().remove(&other);
                                edges.entry(other).or_default().remove(&cell);
                            }
                        }
                    }
                } else {
                    match points.entry(cell) {
                        hash_map::Entry::Vacant(_) => {
                            // Nothing needs to be done
                        }
                        hash_map::Entry::Occupied(mut entry) => {
                            entry.get_mut().1.set(corner, false);
                            if !entry.get().1.is_corner() {
                                // This entry is no longer a corner, so we need
                                // to remove it.
                                entry.remove();
                                changed = true;
                                if let Some(remove_from) = edges.remove(&cell) {
                                    for other in remove_from {
                                        edges
                                            .get_mut(&other.0)
                                            .expect(
                                                "Missing complementary edge in visibility graph",
                                            )
                                            .remove(&cell);
                                    }
                                }
                            } else {
                                // This opens up the possibility of new
                                // connections for this corner, so we will add
                                // this to the new_points collection to be
                                // re-examined.
                                new_points.push(cell);
                            }
                        }
                    }
                }
            }
        }

        let visibility_point_reach = cell_shift - 1;
        for (changed_cell, changed_cell_occupied) in confirmed_changes {
            // Check if any of these changed cells will cause a change in
            // whether a visibility point is occupied.
            for (point_cell, (point_blocked_by, _)) in &mut *points {
                if *changed_cell_occupied {
                    // TODO(MXG): Consider using a bounding volume heiarchy
                    // instead of iterating through every point.
                    if point_blocked_by.is_none() {
                        let dist = *changed_cell - *point_cell;
                        if dist.0.abs() <= visibility_point_reach
                            && dist.1.abs() <= visibility_point_reach
                        {
                            *point_blocked_by = Some(*changed_cell);
                            changed = true;
                        }
                    }
                } else {
                    // TODO(MXG): Consider using a hashmap of blockers -> points
                    // instead of iterating through every point.
                    if let Some(blocking_cell) = point_blocked_by {
                        if blocking_cell == changed_cell {
                            // If the changed cell was known to be blocking this
                            // visibility point, then the visibility point might
                            // be unoccupied now, but we need to test that.
                            *point_blocked_by = grid.is_square_occupied(
                                point_cell.to_center_point(grid.cell_size()),
                                2.0 * agent_radius,
                            );

                            changed |= point_blocked_by.is_none();
                        }
                    }
                }
            }
        }

        for cell in new_points {
            // Note: It is possible for an entry in new_points to no longer be
            // in the points map because this may happen:
            // 1. One cell change causes a point to have new corner possibilities,
            //    therefore the point gets added to new_points.
            // 2. A later cell change in confirmed_changes causes the corner to
            //    be removed entirely.
            if let Some((_, status)) = points.get(&cell) {
                let mut new_connections = Vec::new();
                let cell_connections = edges.entry(cell).or_default();
                for (other, (_, other_status)) in &*points {
                    if cell == *other {
                        continue;
                    }

                    let connection_entry = cell_connections.entry(*other);
                    if let hash_map::Entry::Occupied(_) = connection_entry {
                        // If this connection is already active then we don't need
                        // to do anything here.
                        continue;
                    }

                    if !cell.in_visible_quadrant_of(other, *other_status) {
                        continue;
                    }

                    if !other.in_visible_quadrant_of(&cell, *status) {
                        continue;
                    }

                    let blocked_by = grid.is_sweep_occupied(
                        cell.to_center_point(grid.cell_size()),
                        other.to_center_point(grid.cell_size()),
                        2.0 * agent_radius,
                    );

                    // When .insert_entry becomes stable we can change this match block to
                    // connection_entry.insert_entry(blocked_by);
                    match connection_entry {
                        hash_map::Entry::Occupied(mut entry) => {
                            entry.insert(blocked_by);
                        }
                        hash_map::Entry::Vacant(entry) => {
                            entry.insert(blocked_by);
                        }
                    }
                    new_connections.push((*other, blocked_by));
                    changed = true;
                }

                for (other_cell, blocked_by) in new_connections {
                    edges
                        .entry(other_cell)
                        .or_default()
                        .insert(cell, blocked_by);
                }
            }
        }

        if confirmed_changes.is_empty() {
            // Skip the triangular-for-loop below if there are no changes to
            // consider because the inner-most loop will be empty anyway.
            return changed;
        }

        triangular_for(points.iter(), |(cell_i, _), (cell_j, _)| {
            for (changed_cell, occupied) in confirmed_changes {
                let mut changed_blocker: Option<Option<Cell>> = None;
                if let hash_map::Entry::Occupied(entry) =
                    &mut edges.entry(**cell_i).or_default().entry(*cell_j)
                {
                    if *occupied {
                        let line = LineSegment::new(
                            cell_i.to_center_point(grid.cell_size()),
                            cell_j.to_center_point(grid.cell_size()),
                        );

                        if line.passes_near_cell(changed_cell, grid.cell_size(), agent_radius) {
                            entry.insert(Some(*changed_cell));
                            changed_blocker = Some(Some(*changed_cell));
                            changed = true;
                        }
                    } else {
                        // Check if this entry was blocked by this newly opened cell
                        if let Some(blocked_by) = *entry.get() {
                            if blocked_by == *changed_cell {
                                let new_blocker = grid.is_sweep_occupied(
                                    cell_i.to_center_point(grid.cell_size()),
                                    cell_j.to_center_point(grid.cell_size()),
                                    2.0 * agent_radius,
                                );

                                entry.insert(new_blocker);
                                changed_blocker = Some(new_blocker);
                                changed = true;
                            }
                        }
                    }
                }

                if let Some(new_blocker) = changed_blocker {
                    edges
                        .entry(*cell_j)
                        .or_default()
                        .insert(**cell_i, new_blocker);
                }
            }
        });

        return changed;
    }

    fn calculate_cell_shift(agent_radius: f64, cell_size: f64) -> i64 {
        (agent_radius / cell_size + 0.5).ceil() as i64
    }
}

pub struct UnstableVisibilityAPI<'a, G: Grid> {
    parent: &'a Visibility<G>,
}

impl<'a, G: Grid> UnstableVisibilityAPI<'a, G> {
    pub fn points(&self) -> &'a HashMap<Cell, (BlockedBy, CornerStatus)> {
        return &self.parent.points;
    }

    pub fn edges(&self) -> &'a HashMap<Cell, HashMap<Cell, BlockedBy>> {
        return &self.parent.edges;
    }
}

struct UniqueCellPairSet {
    set: HashSet<(Cell, Cell)>,
}

impl UniqueCellPairSet {
    fn new() -> Self {
        Self {
            set: Default::default(),
        }
    }

    fn insert(&mut self, cell_i: &Cell, cell_j: &Cell) -> bool {
        if cell_i.x < cell_j.x {
            return self.set.insert((*cell_i, *cell_j));
        } else if cell_i.x == cell_j.x {
            if cell_i.y < cell_j.y {
                return self.set.insert((*cell_i, *cell_j));
            }
        }

        // cell_j should actually go first, so let's switch them
        return self.set.insert((*cell_j, *cell_i));
    }
}

pub struct VisibilityEdgeIter<'a, G: Grid> {
    visibility: &'a Visibility<G>,
    point_iter: hash_map::Iter<'a, Cell, HashMap<Cell, BlockedBy>>,
    edge_iter: Option<(&'a Cell, hash_map::Iter<'a, Cell, BlockedBy>)>,
    pair_tracker: UniqueCellPairSet,
}

impl<'a, G: Grid> Iterator for VisibilityEdgeIter<'a, G> {
    type Item = (&'a Cell, &'a Cell);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some((cell_i, edges)) = &mut self.edge_iter {
                while let Some((cell_j, blocked_by)) = edges.next() {
                    if blocked_by.is_none() {
                        if self
                            .visibility
                            .points
                            .get(cell_j)
                            .expect("Missing visibility point information")
                            .0
                            .is_none()
                        {
                            if self.pair_tracker.insert(cell_i, cell_j) {
                                return Some((cell_i, cell_j));
                            }
                        }
                    }
                }
            }

            // We have exhausted the previous iterator, so now we should find
            // the next one.
            self.edge_iter = None;
            while let Some((cell, edges)) = self.point_iter.next() {
                if self
                    .visibility
                    .points
                    .get(cell)
                    .expect("Missing visibility point information")
                    .0
                    .is_none()
                {
                    self.edge_iter = Some((cell, edges.iter()));
                    break;
                }
            }

            if self.edge_iter.is_none() {
                break;
            }
        }

        return None;
    }
}

pub mod sparse_grid;
pub use sparse_grid::SparseGrid;
pub mod visibility_graph;
pub use visibility_graph::{VisibilityGraph, NeighborhoodGraph};
mod util;
