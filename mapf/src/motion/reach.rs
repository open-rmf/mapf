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
    error::{Error, NoError},
};

/// Reachable is a trait to describe attempts to reach directly from a node
/// towards its goal if the goal can be reached from the node with a single
/// motion decision. This returns an iterator in case there are multiple options
/// for how to reach the goal with a single motion decision.
pub trait Reachable<Node, Goal, W: Waypoint> {
    type ReachError: Error;
    type Reaching<'a>: IntoIterator<Item=Result<Trajectory<W>, Self::ReachError>>
    where Self: 'a, W: 'a, Node: 'a, Goal: 'a;

    fn reach_for<'a>(&'a self, parent: &'a Node, goal: &'a Goal) -> Self::Reaching<'a>;
}

pub struct NoReach;
impl<N, G, W: Waypoint> Reachable<N, G, W> for NoReach {
    type ReachError = NoError;
    type Reaching<'a> where N: 'a, G: 'a, W: 'a = impl Iterator<Item=Result<Trajectory<W>, NoError>>;

    fn reach_for<'a>(&'a self, _: &'a N, _: &'a G) -> Self::Reaching<'a> {
        [].into_iter()
    }
}
