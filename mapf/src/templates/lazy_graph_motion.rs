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
    E: Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::Extrapolation: Into<Action>,
    Action: Into<E::Extrapolation>,
    R: ArrivalKeyring<G::Key, G::Key, Goal>,
    C: Connectable<State, Action, Goal>,
{
    type Connections<'a>
        = LazyGraphMotionConnections<'a, S, G, E, R, C, State, Action, Goal>
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
        let from_spatial_state = from_state.borrow().clone();
        let from_key_ref = self.motion.space.key_for(&from_spatial_state);
        let from_key = from_key_ref.borrow().borrow();
        let arrival_keys = self
            .keyring
            .get_arrival_keys(&from_key, to_target)
            .into_iter();

        LazyGraphMotionConnections::<'a, S, G, E, R, C, State, Action, Goal> {
            current_iter: None,
            lazy_edges: None,
            arrival_keys,
            chained_connections: self
                .chain
                .connect(from_state.clone(), to_target)
                .into_iter(),
            motion: &self.motion,
            from_state: from_spatial_state.clone(),
            _ignore: Default::default(),
        }
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

pub struct LazyGraphMotionConnections<'a, S, G, E, R, C, State, Action, Goal>
where
    S: 'a + KeyedSpace<G::Key>,
    S::Key: 'a + Borrow<G::Key>,
    S::State: 'a + Into<State> + Clone,
    State: 'a + Borrow<S::State> + Clone,
    G: 'a + Graph,
    G::Key: 'a + Clone,
    G::EdgeAttributes: 'a + Clone,
    E: 'a + Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::Extrapolation: 'a + Into<Action>,
    Action: 'a + Into<E::Extrapolation>,
    R: 'a + ArrivalKeyring<G::Key, G::Key, Goal>,
    R::ArrivalKeyError: 'a,
    C: 'a + Connectable<State, Action, Goal>,
    State: 'a,
    Action: 'a,
    Goal: 'a,
{
    current_iter:
        Option<Extrapolations<G::Key, <E::ExtrapolationIter<'a> as IntoIterator>::IntoIter>>,
    lazy_edges: Option<<G::LazyEdgeIter<'a> as IntoIterator>::IntoIter>,
    arrival_keys: <R::ArrivalKeys<'a> as IntoIterator>::IntoIter,
    chained_connections: <C::Connections<'a> as IntoIterator>::IntoIter,
    motion: &'a GraphMotion<S, G, E>,
    from_state: S::State,
    _ignore: std::marker::PhantomData<fn(State, Action)>,
}

impl<'a, S, G, E, R, C, State, Action, Goal> Iterator
    for LazyGraphMotionConnections<'a, S, G, E, R, C, State, Action, Goal>
where
    S: 'a + KeyedSpace<G::Key>,
    S::Key: 'a + Borrow<G::Key>,
    S::State: 'a + Into<State> + Clone,
    State: 'a + Borrow<S::State> + Clone,
    G: 'a + Graph,
    G::Key: 'a + Clone,
    G::EdgeAttributes: 'a + Clone,
    E: 'a + Extrapolator<S::Waypoint, G::Vertex, G::EdgeAttributes, G::Key>,
    E::Extrapolation: 'a + Into<Action>,
    Action: 'a + Into<E::Extrapolation>,
    R: 'a + ArrivalKeyring<G::Key, G::Key, Goal>,
    R::ArrivalKeyError: 'a,
    C: 'a + Connectable<State, Action, Goal>,
    State: 'a,
    Action: 'a,
    Goal: 'a,
{
    type Item = Result<
        (Action, State),
        LazyGraphMotionError<G::Key, R::ArrivalKeyError, E::ExtrapolationError, C::ConnectionError>,
    >;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(current_iter) = self.current_iter.as_mut() {
                if let Some(result) = current_iter.extrapolations.next() {
                    match result {
                        Ok((action, waypoint)) => {
                            let to_vertex = current_iter.to_vertex.clone();
                            let state = self.motion.space.make_keyed_state(to_vertex, waypoint);
                            return Some(Ok((action.into(), state.into())));
                        }
                        Err(err) => {
                            return Some(Err(LazyGraphMotionError::Extrapolator(err)));
                        }
                    }
                }
            }
            self.current_iter = None;

            if let Some(lazy_edges) = self.lazy_edges.as_mut() {
                if let Some(edge) = lazy_edges.next() {
                    let to_vertex = edge.to_vertex();
                    let Some(v) = self.motion.graph.vertex(to_vertex) else {
                        return Some(Err(LazyGraphMotionError::MissingVertex(to_vertex.clone())));
                    };

                    let extrapolations = self.motion.extrapolator.extrapolate(
                        self.motion.space.waypoint(&self.from_state).borrow(),
                        v.borrow(),
                        &edge.attributes(),
                        (
                            Some(
                                self.motion
                                    .space
                                    .key_for(&self.from_state)
                                    .borrow()
                                    .borrow(),
                            ),
                            Some(&to_vertex),
                        ),
                    );

                    self.current_iter = Some(Extrapolations {
                        to_vertex: to_vertex.clone(),
                        extrapolations: extrapolations.into_iter(),
                    });
                    // Restart the loop so we can immediately start iterating
                    // over these extrapolations.
                    continue;
                }
            }

            if let Some(result) = self.arrival_keys.next() {
                let to_vertex = match result {
                    Ok(to_vertex) => to_vertex,
                    Err(err) => {
                        return Some(Err(LazyGraphMotionError::Keyring(err)));
                    }
                };

                let from_key_ref = self.motion.space.key_for(&self.from_state);
                let from_key = from_key_ref.borrow().borrow();
                let lazy_edges = self.motion.graph.lazy_edges_between(from_key, &to_vertex);
                self.lazy_edges = Some(lazy_edges.into_iter());
                // Restart the loop so we can immediately start iterating over
                // these lazy edges.
                continue;
            }

            if let Some(next_in_chain) = self.chained_connections.next() {
                return Some(next_in_chain.map_err(LazyGraphMotionError::Chain));
            }

            return None;
        }
    }
}

struct Extrapolations<K, E> {
    to_vertex: K,
    extrapolations: E,
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
        let from_p = from_cell.center_point(cell_size);
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
