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
    graph::{Edge, Graph},
    occupancy::{Cell, Grid, Point, Visibility},
    util::triangular_for,
};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

#[derive(Debug)]
pub struct VisibilityGraph<G: Grid> {
    visibility: Arc<Visibility<G>>,
    visibility_of_interest: HashMap<Cell, HashSet<Cell>>,
    points_of_interest: HashSet<Cell>,
}

fn gather_points_of_interest<G: Grid>(
    visibility: &Visibility<G>,
    points_of_interest: impl Iterator<Item = Cell>,
) -> (HashSet<Cell>, HashMap<Cell, HashSet<Cell>>) {
    let mut connections: HashMap<Cell, HashSet<Cell>> = Default::default();
    let mut valid = HashSet::new();
    for interest in points_of_interest {
        if visibility
            .grid()
            .is_square_occupied(
                interest.to_center_point(visibility.grid().cell_size()),
                visibility.agent_radius(),
            )
            .is_some()
        {
            continue;
        }

        for v in visibility.calculate_visibility(interest) {
            // connections.insert(v, interest);
            connections.entry(v).or_default().insert(interest);
        }

        valid.insert(interest);
    }

    triangular_for(valid.iter(), |interest_i, interest_j| {
        if **interest_i == *interest_j {
            return;
        }

        let cell_size = visibility.grid().cell_size();
        if visibility
            .grid()
            .is_sweep_occupied(
                interest_i.to_center_point(cell_size),
                interest_j.to_center_point(cell_size),
                2.0 * visibility.agent_radius(),
            )
            .is_none()
        {
            connections
                .entry(**interest_i)
                .or_default()
                .insert(*interest_j);
            connections
                .entry(*interest_j)
                .or_default()
                .insert(**interest_i);
        }
    });

    return (valid, connections);
}

impl<G: Grid> VisibilityGraph<G> {
    pub fn new(
        visibility: Arc<Visibility<G>>,
        points_of_interest: impl Iterator<Item = Cell>,
    ) -> Self {
        let (points_of_interest, visibility_of_interest) =
            gather_points_of_interest(visibility.as_ref(), points_of_interest);
        Self {
            visibility,
            visibility_of_interest,
            points_of_interest,
        }
    }
}

/// From any unoccupied cell, latch onto the visibility graph but do not visit
/// any other adjacent cells.
impl<G: Grid> Graph for VisibilityGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type EdgeAttributes = ();

    type VertexRef<'a> = Self::Vertex where G: 'a;
    type Edge<'a> = (Cell, Cell) where G: 'a;
    type EdgeIter<'a> = impl Iterator<Item=(Cell, Cell)> + 'a where Self: 'a;

    fn vertex(&self, cell: &Self::Key) -> Option<Self::Vertex> {
        // We don't bother to filter out occupied cells because those cells will
        // not generate any valid edges anyway. If we filtered them out here we
        // would be frequently doing redundant occupancy checking.
        Some(cell.to_center_point(self.visibility.grid().cell_size()))
    }

    fn edges_from_vertex<'a>(&'a self, from_cell: Self::Key) -> Self::EdgeIter<'a>
    where
        Cell: 'a,
    {
        // dbg!("visibility graph");
        [from_cell]
            .into_iter()
            .filter(|from_cell| {
                // dbg!(from_cell);
                self.visibility
                    .grid()
                    .is_square_occupied(
                        from_cell.to_center_point(self.visibility.grid().cell_size()),
                        2.0 * self.visibility.agent_radius(),
                    )
                    .is_none()
            })
            .flat_map(|from_cell| {
                // dbg!(from_cell);
                self.visibility
                    .calculate_visibility(from_cell)
                    .map(move |to_cell| {
                        // dbg!((from_cell, to_cell))
                        (from_cell, to_cell)
                    })
                    .filter(|(from_cell, to_cell)| from_cell != to_cell)
            })
            .chain({
                let interest = self.visibility_of_interest.get(&from_cell);
                [interest]
                    .into_iter()
                    .filter_map(|x| x)
                    .flat_map(|x| x)
                    .map(move |point_of_interest| (from_cell, *point_of_interest))
                    .chain(
                        [interest]
                            .into_iter()
                            .filter(|x| x.is_none())
                            .flat_map(move |_| {
                                self.points_of_interest
                                    .iter()
                                    .filter(move |poi| {
                                        let to_p =
                                            poi.to_center_point(self.visibility.grid().cell_size());
                                        let from_p = from_cell
                                            .to_center_point(self.visibility.grid().cell_size());
                                        self.visibility
                                            .grid()
                                            .is_sweep_occupied(
                                                from_p,
                                                to_p,
                                                2.0 * self.visibility.agent_radius(),
                                            )
                                            .is_none()
                                    })
                                    .map(move |poi| (from_cell.clone(), *poi))
                            }),
                    )
            })
    }
}

#[derive(Debug)]
pub struct NeighborhoodGraph<G: Grid> {
    visibility: Arc<Visibility<G>>,
    visibility_of_interest: HashMap<Cell, HashSet<Cell>>,
    points_of_interest: HashSet<Cell>,
}

impl<G: Grid> NeighborhoodGraph<G> {
    pub fn new(
        visibility: Arc<Visibility<G>>,
        points_of_interest: impl Iterator<Item = Cell>,
    ) -> Self {
        let (points_of_interest, visibility_of_interest) =
            gather_points_of_interest(visibility.as_ref(), points_of_interest);
        Self {
            visibility,
            visibility_of_interest,
            points_of_interest,
        }
    }
}

/// From any unoccupied cell, expand towards both its adjacent cells and the
/// visibility graph.
impl<G: Grid> Graph for NeighborhoodGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type EdgeAttributes = ();

    type VertexRef<'a> = Self::Vertex where G: 'a;
    type Edge<'a> = (Cell, Cell) where G: 'a;
    type EdgeIter<'a> = impl Iterator<Item=(Cell, Cell)> + 'a where Self: 'a;

    fn vertex(&self, cell: &Self::Key) -> Option<Self::Vertex> {
        if self.visibility.grid().is_occupied(&cell) {
            None
        } else {
            Some(cell.to_center_point(self.visibility.grid().cell_size()))
        }
    }

    fn edges_from_vertex<'a>(&'a self, from_cell: Self::Key) -> Self::EdgeIter<'a>
    where
        Cell: 'a,
    {
        // dbg!("neighborhood graph");
        let from_p = from_cell.to_center_point(self.visibility.grid().cell_size());
        [from_cell]
            .into_iter()
            .filter(move |_| {
                // dbg!(from_cell);
                self.visibility
                    .grid()
                    .is_square_occupied(from_p, 2.0 * self.visibility.agent_radius())
                    .is_none()
            })
            .flat_map(move |_| {
                // dbg!(from_cell);
                self.visibility
                    .calculate_visibility(from_cell)
                    .filter(move |to_cell| {
                        // dbg!(to_cell);
                        // Ignore adjacent cells because those will be given by the
                        // neighbors iterator below. If we repeat the same cell twice,
                        // we force the search queue to do unnecessary work.
                        (to_cell.x - from_cell.x).abs() > 1 || (to_cell.y - from_cell.y).abs() > 1
                    })
                    .map(move |to_cell| (from_cell, to_cell))
                    .chain(
                        self.visibility
                            .neighbors(from_cell)
                            .map(move |to_cell| (from_cell, to_cell))
                            .filter(|(from_cell, to_cell)| {
                                // dbg!((from_cell, to_cell));
                                // dbg!(from_cell != to_cell)
                                from_cell != to_cell
                            }),
                    )
                    .chain({
                        let interest = self.visibility_of_interest.get(&from_cell);
                        [interest]
                            .into_iter()
                            .filter_map(|x| x)
                            .flat_map(|x| x)
                            .map(move |point_of_interest| (from_cell, *point_of_interest))
                            .chain([interest].into_iter().filter(|x| x.is_none()).flat_map(
                                move |_| {
                                    self.points_of_interest
                                        .iter()
                                        .filter(move |poi| {
                                            let to_p = poi.to_center_point(
                                                self.visibility.grid().cell_size(),
                                            );
                                            self.visibility
                                                .grid()
                                                .is_sweep_occupied(
                                                    from_p,
                                                    to_p,
                                                    2.0 * self.visibility.agent_radius(),
                                                )
                                                .is_none()
                                        })
                                        .map(move |poi| (from_cell.clone(), *poi))
                                },
                            ))
                    })
            })
    }
}

impl Edge<Cell, ()> for (Cell, Cell) {
    fn from_vertex(&self) -> &Cell {
        &self.0
    }

    fn to_vertex(&self) -> &Cell {
        &self.1
    }

    fn attributes(&self) -> &() {
        &()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn build() {}
}
