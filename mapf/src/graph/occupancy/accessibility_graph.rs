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

use crate::{
    domain::Reversible,
    error::NoError,
    graph::{
        occupancy::{Cell, Grid},
        Graph,
    },
    motion::r2::Point,
    util::ForkIter,
};
use bitfield::{bitfield, Bit};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

/// From any unoccupied cell, expand towards any adjacent cells for whom the
/// expansion is valid.
pub struct AccessibilityGraph<G: Grid> {
    accessibility: Arc<Accessibility<G>>,
}

impl<G: Grid> AccessibilityGraph<G> {
    pub fn new(accessibility: Arc<Accessibility<G>>) -> Self {
        Self { accessibility }
    }
}

impl<G: Grid> Clone for AccessibilityGraph<G> {
    fn clone(&self) -> Self {
        Self {
            accessibility: self.accessibility.clone(),
        }
    }
}

impl<G: Grid> Graph for AccessibilityGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type EdgeAttributes = ();

    type VertexRef<'a>
        = Point
    where
        G: 'a;
    type Edge<'a>
        = (Cell, Cell)
    where
        G: 'a;
    type EdgeIter<'a>
        = impl Iterator<Item = (Cell, Cell)> + 'a
    where
        Self: 'a;

    fn vertex<'a>(&'a self, key: &Cell) -> Option<Point> {
        if self.accessibility.is_inaccessible(key) {
            return None;
        }

        Some(key.center_point(self.accessibility.grid.cell_size()))
    }

    fn edges_from_vertex<'a>(&'a self, key: &Self::Key) -> Self::EdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
    {
        if self.accessibility.grid.is_occupied(key) {
            return ForkIter::Left(None.into_iter());
        }

        let directions = match self.accessibility.constraints.get(key) {
            Some(constraints) => match constraints {
                CellAccessibility::Accessible(constraints) => *constraints,
                CellAccessibility::Inaccessible => return ForkIter::Left(None.into_iter()),
            },
            None => CellDirections::all(),
        };

        let from_cell = *key;
        ForkIter::Right(
            directions
                .iter_from(from_cell)
                .map(move |to_cell: Cell| (from_cell, to_cell)),
        )
    }

    type LazyEdgeIter<'a>
        = [(Cell, Cell); 0]
    where
        G: 'a;

    fn lazy_edges_between<'a>(&'a self, _: &Self::Key, _: &Self::Key) -> Self::LazyEdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
    {
        []
    }
}

impl<G: Grid> Reversible for AccessibilityGraph<G> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        // Accessibility is always symmetric/bidirectional, so we can just clone
        // the graph in order to reverse it.
        Ok(self.clone())
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct CellDirections(u8);
    impl Debug;
    u8;
    pub north, set_north: 0;
    pub northeast, set_northeast: 1;
    pub east, set_east: 2;
    pub southeast, set_southeast: 3;
    pub south, set_south: 4;
    pub southwest, set_southwest: 5;
    pub west, set_west: 6;
    pub northwest, set_northwest: 7;
}

impl CellDirections {
    pub fn iter_from(self, cell: Cell) -> impl Iterator<Item = Cell> {
        self.iter().map(move |[i, j]| cell.shifted(i, j))
    }

    /// Iterate over the directions that are accessible
    pub fn iter(self) -> CellDirectionsIter {
        CellDirectionsIter {
            next_dir: 0,
            directions: self,
            accessibility: true,
        }
    }

    /// Iterate over the directions that are inaccessible
    pub fn iter_inaccessible(self) -> CellDirectionsIter {
        CellDirectionsIter {
            next_dir: 0,
            directions: self,
            accessibility: false,
        }
    }
}

impl IntoIterator for CellDirections {
    type Item = [i64; 2];
    type IntoIter = CellDirectionsIter;
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

pub struct CellDirectionsIter {
    // NOTE: We need to allow next_dir to increment beyond 255 so we don't get
    // an integer overflow while iterating
    next_dir: u16,
    directions: CellDirections,
    /// Do we want to iterate on directions that are accessible (true) or
    /// inaccessible (false)?
    accessibility: bool,
}

impl Iterator for CellDirectionsIter {
    type Item = [i64; 2];
    fn next(&mut self) -> Option<Self::Item> {
        if self.next_dir > 7 {
            // We have already iterated over all the directions
            return None;
        }

        while self.directions.bit(self.next_dir as usize) != self.accessibility {
            self.next_dir += 1;
            if self.next_dir > 7 {
                return None;
            }
        }

        let shift = match self.next_dir {
            0 => [0, 1],
            1 => [1, 1],
            2 => [1, 0],
            3 => [1, -1],
            4 => [0, -1],
            5 => [-1, -1],
            6 => [-1, 0],
            7 => [-1, 1],
            _ => return None,
        };

        self.next_dir += 1;
        Some(shift)
    }
}

impl CellDirections {
    pub fn all() -> Self {
        Self(u8::MAX)
    }

    pub fn is_all(&self) -> bool {
        self.0 == u8::MAX
    }

    pub fn set_direction(&mut self, i: i8, j: i8, value: bool) -> Result<(), [i8; 2]> {
        match [i, j] {
            [0, 1] => self.set_north(value),
            [1, 1] => self.set_northeast(value),
            [1, 0] => self.set_east(value),
            [1, -1] => self.set_southeast(value),
            [0, -1] => self.set_south(value),
            [-1, -1] => self.set_southwest(value),
            [-1, 0] => self.set_west(value),
            [-1, 1] => self.set_northwest(value),
            _ => return Err([i, j]),
        }

        Ok(())
    }
}

#[derive(Clone, Debug)]
pub enum CellAccessibility {
    /// The cell is accessible but it has some constraints on where the agent
    /// can move from it.
    Accessible(CellDirections),
    /// The cell is entirely inaccessible. The agent cannot be centered at this
    /// cell.
    Inaccessible,
}

impl CellAccessibility {
    pub fn is_inaccessible(&self) -> bool {
        matches!(self, CellAccessibility::Inaccessible)
    }
}

#[derive(Clone, Debug)]
pub struct Accessibility<G: Grid> {
    grid: G,
    agent_radius: f64,
    cell_shift: i64,
    constraints: HashMap<Cell, CellAccessibility>,
}

impl<G: Grid> Accessibility<G> {
    pub fn new(grid: G, agent_radius: f64) -> Self {
        let cell_size = grid.cell_size();
        let mut output = Self {
            grid,
            agent_radius,
            cell_shift: Self::calculate_cell_shift(agent_radius, cell_size),
            constraints: HashMap::new(),
        };

        Self::update_constraints(
            output.grid.occupied_cells(),
            &output.grid,
            output.agent_radius,
            output.cell_shift,
            &mut output.constraints,
        );

        output
    }

    pub fn iter_accessibility<'a>(
        &'a self,
    ) -> impl Iterator<Item = (&'a Cell, &'a CellAccessibility)> {
        self.constraints.iter()
    }

    pub fn grid(&self) -> &G {
        &self.grid
    }

    pub fn agent_radius(&self) -> f64 {
        self.agent_radius
    }

    pub fn is_inaccessible(&self, cell: &Cell) -> bool {
        self.grid.is_occupied(cell)
            || self
                .constraints
                .get(cell)
                .filter(|c| c.is_inaccessible())
                .is_some()
    }

    pub fn change_cells(&mut self, mut changes: HashMap<Cell, bool>) -> bool {
        changes.retain(|cell, value| self.grid.is_occupied(cell) != *value);
        if changes.is_empty() {
            return false;
        }

        self.grid.change_cells(&changes);
        Self::update_constraints(
            changes.keys(),
            &self.grid,
            self.agent_radius,
            self.cell_shift,
            &mut self.constraints,
        );

        return true;
    }

    pub fn change_agent_radius(&mut self, value: f64) {
        self.agent_radius = value;
        self.cell_shift = Self::calculate_cell_shift(self.agent_radius, self.grid.cell_size());
        self.constraints.clear();

        Self::update_constraints(
            self.grid.occupied_cells(),
            &self.grid,
            self.agent_radius,
            self.cell_shift,
            &mut self.constraints,
        );
    }

    fn update_constraints<'a>(
        changed_cells: impl IntoIterator<Item = &'a Cell>,
        grid: &G,
        agent_radius: f64,
        cell_shift: i64,
        constraints: &mut HashMap<Cell, CellAccessibility>,
    ) {
        let mut inspect_cells: HashSet<Cell> = HashSet::new();
        for cell in changed_cells {
            let cell: Cell = *cell;
            for x in -cell_shift..=cell_shift {
                for y in -cell_shift..=cell_shift {
                    inspect_cells.insert(cell.shifted(x, y));
                }
            }
        }

        // For each cell, determine if it is occupied or unavailable
        for cell in &inspect_cells {
            if grid.is_occupied(cell) {
                // If the cell is occupied there's no need to refer to store it
                // in the constraints because we know that it has no adjacency
                // by virtue of being occupied.
                constraints.remove(cell);
                continue;
            }

            let p = cell.center_point(grid.cell_size());
            if grid.is_circle_occupied(p, agent_radius).is_some() {
                constraints.insert(*cell, CellAccessibility::Inaccessible);
                continue;
            }

            // Remove any constraint that might exist on this cell for now. We
            // will put it back if needed in the next loop.
            constraints.remove(cell);
        }

        // For each cell, determine which of its neighbors it can travel to
        for from_cell in &inspect_cells {
            if grid.is_occupied(from_cell) {
                // If the cell is occupied, there's no need to check for
                // expansions out of it
                continue;
            }

            if constraints.contains_key(from_cell) {
                // If it was set as Unavailable in the previous loop, there's
                // no need to check for expansions out of it.
                continue;
            }

            let from_p = from_cell.center_point(grid.cell_size());
            let mut cell_directions = CellDirections::all();
            for i in [-1, 0, 1] {
                for j in [-1, 0, 1] {
                    if i == 0 && j == 0 {
                        continue;
                    }

                    let to_cell: Cell = from_cell.shifted(i, j);
                    if grid.is_occupied(&to_cell)
                        || constraints
                            .get(&to_cell)
                            .filter(|c| c.is_inaccessible())
                            .is_some()
                    {
                        cell_directions.set_direction(i as i8, j as i8, false).ok();
                        continue;
                    }

                    let to_p = to_cell.center_point(grid.cell_size());
                    if grid
                        .is_sweep_occupied(from_p, to_p, 2.0 * agent_radius)
                        .is_some()
                    {
                        cell_directions.set_direction(i as i8, j as i8, false).ok();
                    }
                }
            }

            if !cell_directions.is_all() {
                constraints.insert(*from_cell, CellAccessibility::Accessible(cell_directions));
            }
        }
    }

    fn calculate_cell_shift(agent_radius: f64, cell_size: f64) -> i64 {
        (agent_radius / cell_size + 0.5).ceil() as i64
    }
}
