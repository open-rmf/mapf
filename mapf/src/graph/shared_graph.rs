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

use crate::{domain::Reversible, error::ThisError, graph::Graph};
use std::{
    ops::Deref,
    sync::{Arc, RwLock},
};

/// Wrapper around Graph types which allow the graph and its reverse to be
/// shared while still implementing the Graph and Reversible traits. It is
/// advisable to pass around graphs within this wrapper to minimize the memory
/// footprint of your planners.
#[derive(Debug)]
pub struct SharedGraph<G> {
    graph: Arc<G>,
    reverse: Arc<RwLock<Option<Arc<G>>>>,
}

impl<G> Clone for SharedGraph<G> {
    fn clone(&self) -> Self {
        Self {
            graph: self.graph.clone(),
            reverse: self.reverse.clone(),
        }
    }
}

impl<G> SharedGraph<G> {
    pub fn new(graph: G) -> Self {
        Self {
            graph: Arc::new(graph),
            reverse: Arc::new(RwLock::new(None)),
        }
    }

    pub fn from_refs(graph: Arc<G>, reverse: Arc<G>) -> Self {
        Self {
            graph,
            reverse: Arc::new(RwLock::new(Some(reverse))),
        }
    }
}

impl<G: Graph> Graph for SharedGraph<G> {
    type Vertex = G::Vertex;
    type Key = G::Key;
    type EdgeAttributes = G::EdgeAttributes;

    type VertexRef<'a> = G::VertexRef<'a>
    where
        G: 'a;

    type Edge<'a> = G::Edge<'a>
    where
        G: 'a;

    type EdgeIter<'a> = G::EdgeIter<'a>
    where
        G: 'a;

    fn vertex<'a>(&'a self, key: &Self::Key) -> Option<Self::VertexRef<'a>> {
        self.graph.vertex(key)
    }

    fn edges_from_vertex<'a>(&'a self, key: &Self::Key) -> Self::EdgeIter<'a>
    where
        Self: 'a,
        Self::Vertex: 'a,
        Self::Key: 'a,
        Self::EdgeAttributes: 'a,
    {
        self.graph.edges_from_vertex(key)
    }

    type LazyEdgeIter<'a> = G::LazyEdgeIter<'a>
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
        self.graph.lazy_edges_between(from_key, to_key)
    }
}

impl<G> Deref for SharedGraph<G> {
    type Target = G;
    fn deref(&self) -> &Self::Target {
        &*self.graph
    }
}

impl<G: Reversible> Reversible for SharedGraph<G> {
    type ReversalError = SharedGraphReversalError<G::ReversalError>;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        {
            let guard = self
                .reverse
                .read()
                .map_err(|_| SharedGraphReversalError::PoisonedMutex)?;

            if let Some(reverse) = &*guard {
                return Ok(SharedGraph::from_refs(reverse.clone(), self.graph.clone()));
            }
        }

        // We don't already have a reverse of the graph, so we need to get
        // write permissions and create one.
        let mut guard = self
            .reverse
            .write()
            .map_err(|_| SharedGraphReversalError::PoisonedMutex)?;

        if let Some(reverse) = &*guard {
            // Check one more time if the reverse is available because it might
            // have been created while we were waitin for write permissions.
            return Ok(SharedGraph::from_refs(reverse.clone(), self.graph.clone()));
        }

        // We definitely need to create the reverse now.
        let reverse = Arc::new(
            self.graph
                .reversed()
                .map_err(SharedGraphReversalError::Graph)?,
        );

        *guard = Some(reverse.clone());
        Ok(SharedGraph::from_refs(reverse, self.graph.clone()))
    }
}

#[derive(ThisError, Debug)]
pub enum SharedGraphReversalError<R> {
    #[error("The graph had an error while reversing:\n{0}")]
    Graph(R),
    #[error("A mutex was poisoned")]
    PoisonedMutex,
}
