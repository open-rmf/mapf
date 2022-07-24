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
    fn new(visibility: Arc<Visibility<G>>) -> Self {
        Self{visibility}
    }
}

impl<G: Grid> Graph for VisibilityGraph<G> {
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
        self.visibility.calculate_visibility(from_cell)
            .map(move |to_cell| (from_cell, *to_cell))
        .chain(
            self.visibility.neighbors(from_cell)
            .map(move |to_cell| (from_cell, to_cell))
        )
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
