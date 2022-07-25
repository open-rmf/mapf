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

use crate::{
    occupancy::{Visibility, Grid, Cell, Point},
    graph::{Graph, Edge},
};
use std::sync::Arc;

#[derive(Debug)]
pub struct VisibilityGraph<G: Grid> {
    visibility: Arc<Visibility<G>>,
}

impl<G: Grid> VisibilityGraph<G> {
    pub fn new(visibility: Arc<Visibility<G>>) -> Self {
        Self{visibility}
    }
}

/// From any unoccupied cell, latch onto the visibility graph but do not visit
/// any other adjacent cells.
impl<G: Grid> Graph for VisibilityGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type Edge = (Cell, Cell);
    type EdgeIter<'a> where Self: 'a = impl Iterator<Item=(Cell, Cell)> + 'a;

    fn vertex(&self, cell: Self::Key) -> Option<Self::Vertex> {
        // We don't bother to filter out occupied cells because those cells will
        // not generate any valid edges anyway. If we filtered them out here we
        // would be frequently doing redundant occupancy checking.
        Some(cell.to_center_point(self.visibility.grid().cell_size()))
    }

    fn edges_from_vertex<'a>(&'a self, from_cell: Self::Key) -> Self::EdgeIter<'a> {
        [from_cell].into_iter()
        .filter(|from_cell| {
            self.visibility.grid().is_square_occupied(
                from_cell.to_center_point(self.visibility.grid().cell_size()),
                2.0*self.visibility.agent_radius(),
            ).is_none()
        })
        .flat_map(|from_cell| {
            self.visibility.calculate_visibility(from_cell)
            .map(move |to_cell| (from_cell, *to_cell))
        })
    }
}

#[derive(Debug)]
pub struct NeighborhoodGraph<G: Grid> {
    visibility: Arc<Visibility<G>>,
}

impl<G: Grid> NeighborhoodGraph<G> {
    pub fn new(visibility: Arc<Visibility<G>>) -> Self {
        Self{visibility}
    }
}

/// From any unoccupied cell, expand towards both its adjacent cells and the
/// visibility graph.
impl<G: Grid> Graph for NeighborhoodGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type Edge = (Cell, Cell);
    type EdgeIter<'a> where Self: 'a = impl Iterator<Item=(Cell, Cell)> + 'a;

    fn vertex(&self, cell: Self::Key) -> Option<Self::Vertex> {
        if self.visibility.grid().is_occupied(&cell) {
            None
        } else {
            Some(cell.to_center_point(self.visibility.grid().cell_size()))
        }
    }

    fn edges_from_vertex<'a>(&'a self, from_cell: Self::Key) -> Self::EdgeIter<'a> {
        [from_cell].into_iter()
        .filter(|from_cell| {
            self.visibility.grid().is_square_occupied(
                from_cell.to_center_point(self.visibility.grid().cell_size()),
                2.0*self.visibility.agent_radius(),
            ).is_none()
        })
        .flat_map(|from_cell| {
            self.visibility.calculate_visibility(from_cell)
                .filter(move |to_cell| {
                    // Ignore adjacent cells because those will be given by the
                    // neighbors iterator below. If we repeat the same cell twice,
                    // we force the search queue to do unnecessary work.
                    (to_cell.x - from_cell.x).abs() <= 1 || (to_cell.y - from_cell.y).abs() <= 1
                })
                .map(move |to_cell| (from_cell, *to_cell))
            .chain(
                self.visibility.neighbors(from_cell)
                .map(move |to_cell| (from_cell, to_cell))
            )
        })
    }
}

impl Edge<Cell> for (Cell, Cell) {
    fn from_vertex(&self) -> &Cell {
        &self.0
    }

    fn to_vertex(&self) -> &Cell {
        &self.1
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn build() {

    }
}
