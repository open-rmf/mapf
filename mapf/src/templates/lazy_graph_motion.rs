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
    domain::{ArrivalKeyring, Connectable, Extrapolator, KeyedSpace, Reversible},
    error::ThisError,
    graph::{Edge, Graph},
    templates::GraphMotion,
    util::FlatResultMapTrait,
};
use std::borrow::Borrow;

/// [`LazyGraphMotion`] implements the [`Connectable`] trait for graphs that
/// perform lazy evaluation. This allows the domain to find connections to goal
/// states that would otherwise not show up in normal graph traversal.
#[derive(Debug, Clone)]
pub struct LazyGraphMotion<S, G, E, R, C> {
    /// Describe how the agent through the graph. The graph should have a useful
    /// implementation of [`Graph::lazy_edges_between`] in order for this struct
    /// to be useful. Otherwise it will be a no-opt.
    pub motion: GraphMotion<S, G, E>,
    /// The keyring that converts goals into graph keys.
    pub keyring: R,
    /// An optional field to chain another [`Connectable`] to this graph motion.
    pub chain: C,
}

impl<S, G, E, R, C, State, Action, Goal> Connectable<State, Action, Goal>
    for LazyGraphMotion<S, G, E, R, C>
where
    S: KeyedSpace<G::Key>,
    S::Key: Borrow<G::Key>,
    S::State: Into<State> + Clone,
    State: Borrow<S::State> + Clone,
    G: Graph,
    G::Key: Clone,
    G::EdgeAttributes: Clone,
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes>,
    E::Extrapolation: Into<Action>,
    Action: Into<E::Extrapolation>,
    R: ArrivalKeyring<G::Key, G::Key, Goal>,
    C: Connectable<State, Action, Goal>,
{
    type Connections<'a> = impl Iterator<Item=Result<(Action, State), Self::ConnectionError>> + 'a
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Goal: 'a;

    type ConnectionError =
        LazyGraphMotionError<G::Key, R::ArrivalKeyError, E::ExtrapolationError, C::ConnectionError>;
    fn connect<'a>(&'a self, from_state: State, to_target: &'a Goal) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        State: 'a,
        Action: 'a,
        Goal: 'a,
    {
        let from_spatial_state: S::State = from_state.clone().borrow().clone();
        let from_key: G::Key = self
            .motion
            .space
            .key_for(&from_spatial_state)
            .borrow()
            .borrow()
            .clone();
        self.keyring
            .get_arrival_keys(&from_key, to_target)
            .into_iter()
            .flat_map(move |r: Result<G::Key, R::ArrivalKeyError>| {
                let from_spatial_state = from_spatial_state.clone();
                let from_key = from_key.clone();
                r.map_err(LazyGraphMotionError::Keyring)
                    .flat_result_map(move |to_vertex: G::Key| {
                        let from_spatial_state = from_spatial_state.clone();
                        self.motion
                            .graph
                            .lazy_edges_between(&from_key, &to_vertex)
                            .into_iter()
                            .flat_map(move |edge| {
                                let from_state = from_spatial_state.clone();
                                let to_vertex = to_vertex.clone();
                                let edge: G::EdgeAttributes = edge.attributes().clone();

                                self.motion
                                    .graph
                                    .vertex(&to_vertex)
                                    .ok_or_else(|| {
                                        LazyGraphMotionError::MissingVertex(to_vertex.clone())
                                    })
                                    .flat_result_map(move |v| {
                                        let from_state = from_state.clone();
                                        let to_vertex = to_vertex.clone();
                                        let extrapolations = self.motion.extrapolator.extrapolate(
                                            self.motion.space.waypoint(&from_state).borrow(),
                                            v.borrow(),
                                            &edge,
                                        );

                                        extrapolations.into_iter().map(move |r| {
                                            let to_vertex = to_vertex.clone();
                                            r.map_err(LazyGraphMotionError::Extrapolator).map(
                                                move |(action, waypoint)| {
                                                    let to_vertex = to_vertex.clone();
                                                    let state = self.motion.space.make_keyed_state(
                                                        to_vertex.clone(),
                                                        waypoint,
                                                    );
                                                    (action.into(), state.into())
                                                },
                                            )
                                        })
                                    })
                                    .map(|x| x.flatten())
                            })
                    })
                    .map(|x| x.flatten())
                    .map(|x: Result<(Action, State), Self::ConnectionError>| x)
            })
            .chain(
                self.chain
                    .connect(from_state, to_target)
                    .into_iter()
                    .map(|r| r.map_err(LazyGraphMotionError::Chain)),
            )
    }
}

impl<S, G, E, R, C> Reversible for LazyGraphMotion<S, G, E, R, C>
where
    S: Reversible,
    G: Reversible,
    E: Reversible,
    R: Reversible,
    C: Reversible,
{
    type ReversalError = LazyGraphMotionReversalError<
        <GraphMotion<S, G, E> as Reversible>::ReversalError,
        R::ReversalError,
        C::ReversalError,
    >;

    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        Ok(LazyGraphMotion {
            motion: self
                .motion
                .reversed()
                .map_err(LazyGraphMotionReversalError::GraphMotion)?,
            keyring: self
                .keyring
                .reversed()
                .map_err(LazyGraphMotionReversalError::Keyring)?,
            chain: self
                .chain
                .reversed()
                .map_err(LazyGraphMotionReversalError::Chain)?,
        })
    }
}

#[derive(Debug, ThisError)]
pub enum LazyGraphMotionError<K, R, E, C> {
    #[error("The graph is missing the requested vertex [{0:?}]")]
    MissingVertex(K),
    #[error("The keyring experienced an error:\n{0:?}")]
    Keyring(R),
    #[error("The extrapolator experienced an error:\n{0:?}")]
    Extrapolator(E),
    #[error("The chained connector experienced an error:\n{0:?}")]
    Chain(C),
}

#[derive(Debug, ThisError)]
pub enum LazyGraphMotionReversalError<M, R, C> {
    #[error("The graph motion had a reversal error:\n{0:?}")]
    GraphMotion(M),
    #[error("The keyring had a reversal error:\n{0:?}")]
    Keyring(R),
    #[error("The chained connector had a reversal error:\n{0:?}")]
    Chain(C),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{graph::occupancy::*, motion::se2::*};
    use std::sync::Arc;

    #[test]
    fn test_lazy_graph_motion_connect() {
        let cell_size = 0.1;
        let mut visibility = Visibility::new(SparseGrid::new(cell_size), 1.0);
        visibility.change_cells(&[(Cell::new(22, 3), true)].into_iter().collect());

        let graph = VisibilityGraph::new(Arc::new(visibility), []);
        let extrapolator = DifferentialDriveLineFollow::new(1.0, 1.0).unwrap();

        let lazy = LazyGraphMotion {
            motion: GraphMotion {
                space: DiscreteSpaceTimeSE2::<Cell, 100>::new(),
                graph: graph.clone(),
                extrapolator,
            },
            keyring: (),
            chain: (),
        };

        let from_cell = Cell::new(2, 3);
        let from_p = from_cell.to_center_point(cell_size);
        let from_state = StateSE2::new(
            from_cell,
            WaypointSE2::new_f64(0.0, from_p.x, from_p.y, 0.0),
        );

        for (to_cell, expected_connections) in [
            (Cell::new(46, 3), 0), // blocked
            (Cell::new(22, 3), 0), // blocked
            (Cell::new(13, 3), 0), // blocked
            (Cell::new(22, 70), 1),
            (Cell::new(-30, -30), 1),
            (Cell::new(-60, 20), 1),
        ] {
            let r: Result<Vec<(DifferentialDriveLineFollowMotion, _)>, _> =
                lazy.connect(from_state, &to_cell).into_iter().collect();

            let connections = r.unwrap();
            assert_eq!(expected_connections, connections.len());
        }
    }
}
