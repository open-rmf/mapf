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

use std::collections::{hash_set, HashSet, hash_map, HashMap, BTreeSet, btree_map, BTreeMap};
use super::{Point, Vector, Cell, Corner, CornerStatus, ConfirmedChanges, ChangedCorners, Grid};
use super::util::{SearchF64, LineSegment};

#[derive(Clone)]
pub struct SparseGrid {
    cell_size: f64,
    occupied: HashSet<Cell>,
    corners: HashMap<Cell, CornerStatus>,
    occupancy_map: BTreeMap<i64, BTreeSet<i64>>,
}

impl SparseGrid {
    /// Create a new empty grid where nothing is occupied
    pub fn new(cell_size: f64) -> SparseGrid {
        Self{
            cell_size,
            occupied: HashSet::default(),
            corners: HashMap::default(),
            occupancy_map: BTreeMap::default(),
        }
    }

    fn update_corner_status(
        &mut self,
        delta: &mut ChangedCorners,
        cell: &Cell,
    ) {
        if !self.is_occupied(cell) {
            if self.corners.remove(cell).is_some() {
                delta.push((*cell, CornerStatus::default()));
            }
            return;
        }

        let mut status = CornerStatus::default();
        let vertical_edges: [(i8, bool); 2] = [
            (-1, !self.occupied.contains(&cell.shifted(-1, 0))),
            (1, !self.occupied.contains(&cell.shifted(1, 0))),
        ];

        if vertical_edges[0].1 || vertical_edges[1].1 {
            let horizontal_edges: [(i8, bool); 2] = [
                (-1, !self.occupied.contains(&cell.shifted(0, -1))),
                (1, !self.occupied.contains(&cell.shifted(0, 1))),
            ];

            for (i, vertical_edge) in vertical_edges {
                for (j, horizontal_edge) in horizontal_edges {
                    if vertical_edge && horizontal_edge {
                        if !self.occupied.contains(&cell.shifted(i as i64, j as i64)) {
                            status.set(Corner(i, j), true);
                        }
                    }
                }
            }
        }

        if status.is_corner() {
            match self.corners.entry(*cell) {
                hash_map::Entry::Vacant(entry) => {
                    entry.insert(status);
                    delta.push((*cell, status));
                },
                hash_map::Entry::Occupied(mut entry) => {
                    let existing_corner = entry.get_mut();
                    let changed = status != *existing_corner;
                    *existing_corner = status;
                    if changed {
                        delta.push((*cell, status));
                    }
                }
            }
        } else {
            if self.corners.remove(cell).is_some() {
                delta.push((*cell, status));
            }
        }
    }

    fn update_cell(
        &mut self,
        cell: &Cell,
        occupied: bool,
    ) -> bool {
        if occupied {
            if self.occupied.insert(*cell) {
                self.occupancy_map.entry(cell.x)
                    .or_default().insert(cell.y);

                return true;
            }

            return false;
        } else {
            if self.occupied.remove(cell) {
                match self.occupancy_map.entry(cell.x) {
                    btree_map::Entry::Vacant(_) => {
                        // This should never be vacant if the cell value was in
                        // the set of occupied cells
                        assert!(false, "Vacant column for cell {:?} that was in the occupied set", cell);
                    },
                    btree_map::Entry::Occupied(mut entry) => {
                        let empty_column = {
                            let column = entry.get_mut();
                            column.remove(&cell.y);
                            column.is_empty()
                        };

                        if empty_column {
                            // If the column is empty now, we should remove it
                            // from the map entirely to reduce the need to
                            // iterate.
                            entry.remove();
                        }
                    }
                }

                return true;
            }

            return false;
        }
    }
}

impl Grid for SparseGrid {

    type OccupiedIterator<'a> = hash_set::Iter<'a, Cell>;
    type CornerIterator<'a> = hash_map::Iter<'a, Cell, CornerStatus>;

    /// Change whether a cell is occupied or not. Returns true if a change
    /// happened, otherwise returns false if the cell already had this value.
    fn change_cells(&mut self, changes: &HashMap<Cell, bool>) -> (
        ConfirmedChanges,
        ChangedCorners,
     ) {
        let mut confirmed_changes = Vec::new();
        confirmed_changes.reserve(changes.len());

        for change in changes {
            if self.update_cell(change.0, *change.1) {
                confirmed_changes.push((*change.0, *change.1));
            }
        }

        let mut delta = ChangedCorners::default();
        let mut checked = HashSet::new();
        for (check, _) in &confirmed_changes {
            for i in -1..=1 {
                for j in -1..=1 {
                    let cell = check.shifted(i, j);
                    if checked.insert(cell) {
                        self.update_corner_status(&mut delta, &cell);
                    }
                }
            }
        }

        return (confirmed_changes, delta);
    }

    fn cell_size(&self) -> f64 {
        return self.cell_size;
    }

    fn is_occupied(&self, cell: &Cell) -> bool {
        return self.occupied.contains(cell);
    }

    fn occupied_cells<'b>(&'b self) -> Self::OccupiedIterator<'b> {
        return self.occupied.iter();
    }

    fn corners<'b>(&'b self) -> Self::CornerIterator<'b> {
        return self.corners.iter();
    }

    fn is_point_occupied(&self, p: Point) -> Option<Cell> {
        let cell = Cell::from_point(p, self.cell_size);
        if self.occupied.contains(&cell) {
            return Some(cell);
        }

        return None;
    }

    fn is_square_occupied(&self, p: Point, width: f64) -> Option<Cell> {
        let d = width/2.0;
        let delta = Vector::new(d, d);
        let min_p = p - delta;
        let max_p = p + delta;
        let min_cell = Cell::from_point(min_p, self.cell_size);
        let max_cell = Cell{
            x: (max_p.x/self.cell_size).ceil() as i64,
            y: (max_p.y/self.cell_size).ceil() as i64,
        };

        for (i, column) in self.occupancy_map.range(min_cell.x .. max_cell.x) {
            for j in column.range(min_cell.y .. max_cell.y) {
                return Some(Cell::new(*i, *j));
            }
        }

        return None;
    }

    fn is_sweep_occupied(&self, p0: Point, p1: Point, width: f64) -> Option<Cell> {
        let d = width/2.0;
        let dist = (p1 - p0).norm();
        if dist < 1e-8 {
            return self.is_point_occupied(p0);
        }

        let v = (p1 - p0)/dist;
        let n = Vector::new(-v.y, v.x);

        let points = [p0 + n*d, p0 - n*d, p1 + n*d, p1 - n*d];

        let lines = [
            LineSegment::new(points[0], points[1]),
            LineSegment::new(points[0], points[2]),
            LineSegment::new(points[1], points[3]),
            LineSegment::new(points[2], points[3]),
        ];

        let cell_x_min = (
            points.iter().min_by(
                |p_l, p_r| p_l.x.partial_cmp(&p_r.x).unwrap()
            ).unwrap().x / self.cell_size
        ).floor() as i64;

        let cell_x_max = (
            points.iter().max_by(
                |p_l, p_r| p_l.x.partial_cmp(&p_r.x).unwrap()
            ).unwrap().x / self.cell_size
        ).ceil() as i64;

        for (cell_x, column) in self.occupancy_map.range(cell_x_min .. cell_x_max) {
            let x_low = *cell_x as f64 * self.cell_size;
            let x_high = (cell_x + 1) as f64 * self.cell_size;
            let mut y_low = SearchF64::new();
            let mut y_high = SearchF64::new();

            for p in points {
                if x_low <= p.x && p.x <= x_high {
                    y_low.check_min(p.y);
                    y_high.check_max(p.y);
                }
            }

            for x in [x_low, x_high] {
                for line in &lines {
                    for y in line.vertical_intersect(x) {
                        y_low.check_min(y);
                        y_high.check_max(y);
                    }
                }
            }

            if let (Some(y_low), Some(y_high)) = (y_low.value, y_high.value) {
                let cell_y_min = (y_low / self.cell_size).floor() as i64;
                let cell_y_max = (y_high / self.cell_size).ceil() as i64;
                for cell_y in column.range(cell_y_min .. cell_y_max) {
                    return Some(Cell::new(*cell_x, *cell_y));
                }
            }
        }

        return None;
    }
}
