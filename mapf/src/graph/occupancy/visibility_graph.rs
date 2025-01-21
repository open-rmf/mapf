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
    domain::Reversible,
    error::NoError,
    graph::{
        occupancy::{Cell, Grid, Point, Visibility},
        Edge, Graph,
    },
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

impl<G: Grid> Clone for VisibilityGraph<G> {
    fn clone(&self) -> Self {
        Self {
            visibility: self.visibility.clone(),
            visibility_of_interest: self.visibility_of_interest.clone(),
            points_of_interest: self.points_of_interest.clone(),
        }
    }
}

fn gather_points_of_interest<G: Grid>(
    visibility: &Visibility<G>,
    points_of_interest: impl IntoIterator<Item = Cell>,
) -> (HashSet<Cell>, HashMap<Cell, HashSet<Cell>>) {
    let mut connections: HashMap<Cell, HashSet<Cell>> = Default::default();
    let mut valid = HashSet::new();
    for interest in points_of_interest {
        if visibility
            .grid()
            .is_square_occupied(
                interest.center_point(visibility.grid().cell_size()),
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
                interest_i.center_point(cell_size),
                interest_j.center_point(cell_size),
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
        points_of_interest: impl IntoIterator<Item = Cell>,
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

    type VertexRef<'a>
        = Self::Vertex
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

    fn vertex(&self, cell: &Self::Key) -> Option<Self::Vertex> {
        // We don't bother to filter out occupied cells because those cells will
        // not generate any valid edges anyway. If we filtered them out here we
        // would be frequently doing redundant occupancy checking.
        Some(cell.center_point(self.visibility.grid().cell_size()))
    }

    fn edges_from_vertex<'a, 'b>(&'a self, from_cell: &'b Self::Key) -> Self::EdgeIter<'a>
    where
        Cell: 'a,
    {
        let from_cell = *from_cell;
        [from_cell]
            .into_iter()
            .filter(|from_cell| {
                self.visibility
                    .grid()
                    .is_square_occupied(
                        from_cell.center_point(self.visibility.grid().cell_size()),
                        2.0 * self.visibility.agent_radius(),
                    )
                    .is_none()
            })
            .flat_map(|from_cell| {
                self.visibility
                    .calculate_visibility(from_cell)
                    .map(move |to_cell| (from_cell, to_cell))
                    .filter(|(from_cell, to_cell)| from_cell != to_cell)
            })
            .chain({
                let interest = self.visibility_of_interest.get(&from_cell);
                interest
                    .into_iter()
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
                                            poi.center_point(self.visibility.grid().cell_size());
                                        let from_p = from_cell
                                            .center_point(self.visibility.grid().cell_size());
                                        self.visibility
                                            .grid()
                                            .is_sweep_occupied(
                                                from_p,
                                                to_p,
                                                2.0 * self.visibility.agent_radius(),
                                            )
                                            .is_none()
                                    })
                                    .map(move |poi| (from_cell, *poi))
                            }),
                    )
            })
    }

    type LazyEdgeIter<'a>
        = Option<(Cell, Cell)>
    where
        G: 'a;
    fn lazy_edges_between<'a>(
        &'a self,
        from_key: &Self::Key,
        to_key: &Self::Key,
    ) -> Self::LazyEdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
    {
        if self.visibility.points.contains_key(to_key) || self.points_of_interest.contains(to_key) {
            // No need to return anything because the target is in the set of
            // points that will be returned by edges_from_vertex(from_key)
            return None;
        }

        let from_p = from_key.center_point(self.visibility.grid().cell_size());
        let to_p = to_key.center_point(self.visibility.grid().cell_size());
        for p in [from_p, to_p] {
            if self
                .visibility
                .grid()
                .is_square_occupied(p, 2.0 * self.visibility.agent_radius)
                .is_some()
            {
                // An endpoint cell is occupied so we cannot generate an edge
                // between the two cells.
                return None;
            }
        }

        if self
            .visibility
            .grid()
            .is_sweep_occupied(from_p, to_p, 2.0 * self.visibility.agent_radius())
            .is_some()
        {
            // A point along the sweep between the cells is occupied so we
            // cannot generate an edge between teh two cells.
            return None;
        }

        return Some((*from_key, *to_key));
    }
}

impl<G: Grid> Reversible for VisibilityGraph<G> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        // Visibility graphs are always bidirectional, so the reverse is the
        // same as the forward.
        Ok(self.clone())
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
        points_of_interest: impl IntoIterator<Item = Cell>,
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

impl<G: Grid> Clone for NeighborhoodGraph<G> {
    fn clone(&self) -> Self {
        Self {
            visibility: self.visibility.clone(),
            visibility_of_interest: self.visibility_of_interest.clone(),
            points_of_interest: self.points_of_interest.clone(),
        }
    }
}

/// From any unoccupied cell, expand towards both its adjacent cells and the
/// visibility graph.
///
/// Similar to [`VisibilityGraph`] except it also expands to adjacent cells like
/// [`super::AdjacencyGraph`].
impl<G: Grid> Graph for NeighborhoodGraph<G> {
    type Key = Cell;
    type Vertex = Point;
    type EdgeAttributes = ();

    type VertexRef<'a>
        = Self::Vertex
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

    fn vertex(&self, cell: &Self::Key) -> Option<Self::Vertex> {
        if self.visibility.grid().is_occupied(&cell) {
            None
        } else {
            Some(cell.center_point(self.visibility.grid().cell_size()))
        }
    }

    fn edges_from_vertex<'a>(&'a self, from_cell: &Self::Key) -> Self::EdgeIter<'a>
    where
        Cell: 'a,
    {
        // dbg!("neighborhood graph");
        let from_cell = *from_cell;
        let from_p = from_cell.center_point(self.visibility.grid().cell_size());
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
                        interest
                            .into_iter()
                            .flat_map(|x| x)
                            .map(move |point_of_interest| (from_cell, *point_of_interest))
                            .chain([interest].into_iter().filter(|x| x.is_none()).flat_map(
                                move |_| {
                                    self.points_of_interest
                                        .iter()
                                        .filter(move |poi| {
                                            let to_p = poi
                                                .center_point(self.visibility.grid().cell_size());
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

    type LazyEdgeIter<'a>
        = Option<(Cell, Cell)>
    where
        G: 'a;

    fn lazy_edges_between<'a>(
        &'a self,
        from_key: &Self::Key,
        to_key: &Self::Key,
    ) -> Self::LazyEdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
    {
        if self.visibility.points.contains_key(to_key) || self.points_of_interest.contains(to_key) {
            // No need to return anything because the target is in the set of
            // points that will be returned by edges_from_vertex(from_key)
            return None;
        }

        let from_p = from_key.center_point(self.visibility.grid().cell_size());
        let to_p = to_key.center_point(self.visibility.grid().cell_size());
        for p in [from_p, to_p] {
            if self
                .visibility
                .grid()
                .is_square_occupied(p, 2.0 * self.visibility.agent_radius)
                .is_some()
            {
                // An endpoint cell is occupied so we cannot generate an edge
                // between the two cells.
                return None;
            }
        }

        if self
            .visibility
            .grid()
            .is_sweep_occupied(from_p, to_p, 2.0 * self.visibility.agent_radius())
            .is_some()
        {
            // A point along the sweep between the cells is occupied so we
            // cannot generate an edge between teh two cells.
            return None;
        }

        return Some((*from_key, *to_key));
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

impl<G: Grid> Reversible for NeighborhoodGraph<G> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        // Visibility graphs are always bidirectional, so the reverse is the
        // same as the forward.
        Ok(self.clone())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn build() {}
}
