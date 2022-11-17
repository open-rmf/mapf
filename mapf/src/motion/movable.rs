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
    motion::{Trajectory, Waypoint},
    node::{Agent, CostOf, KeyOf, PartialKeyed, Weighted},
};
use std::sync::Arc;

/// This trait should be implemented for Arc<N> where N is a movable node. It
/// should generally not be implemented on N directly because then the moved
/// node cannot safely keep track of its parent.
pub trait Movable<W: Waypoint>: Weighted + PartialKeyed + Agent<W, Trajectory<W>> {
    fn move_from(
        parent: Arc<Self>,
        key: Option<Self::Key>,
        cost_from_parent: Self::Cost,
        remaining_cost_estimate: Self::Cost,
        motion_from_parent: Option<Trajectory<W>>,
    ) -> Self;
}

pub trait ArcMovable<W: Waypoint, N: Weighted + PartialKeyed + Agent<W, Trajectory<W>>> {
    fn moved_with(
        self,
        key: Option<N::Key>,
        cost_from_parent: N::Cost,
        remaining_cost_estimate: N::Cost,
        motion_from_parent: Option<Trajectory<W>>,
    ) -> Self;
}

impl<W: Waypoint, N: Movable<W>> ArcMovable<W, N> for Arc<N> {
    fn moved_with(
        self,
        key: Option<KeyOf<N>>,
        cost_from_parent: CostOf<N>,
        remaining_cost_estimate: CostOf<N>,
        motion_from_parent: Option<Trajectory<W>>,
    ) -> Self {
        Arc::new(N::move_from(
            self,
            key,
            cost_from_parent,
            remaining_cost_estimate,
            motion_from_parent,
        ))
    }
}

/// Unlike Movable, this trait should be implemented directly for the Node type.
pub trait StartingPoint<Cost, Key, W: Waypoint> {
    fn start_from(
        state: W,
        key: Option<Key>,
        initial_cost: Cost,
        remaining_cost_estimate: Cost,
        initial_motion: Option<Trajectory<W>>,
    ) -> Self;
}
