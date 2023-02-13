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

/// The `Space` trait describes the data structures of a state space
pub trait Space {
    /// The portion of the search state that describes the physical aspects of
    /// the state
    type Waypoint;

    /// The domain state that is searched over
    type State;

    /// Extract a waypoint for the given state
    fn waypoint(&self, state: &Self::State) -> Self::Waypoint;
}

/// [`PartialKeyedSpace`] is a trait for describing spaces whose states can
/// usually be assigned a unique key while there may be exceptions. If the
/// states can always be assigned a unique key then [`KeyedSpace`] should be
/// used instead.
pub trait PartialKeyedSpace<K>: Space {
    fn make_partial_keyed_state(
        &self,
        key: Option<K>,
        waypoint: Self::Waypoint,
    ) -> Self::State;

    fn partial_key(&self, state: &Self::State) -> Option<K>;
}

/// [`KeyedSpace`] is a trait for described spaces whose states can always be
/// assigned a unique key. If some states might not have a unique key,
/// [`PartialKeyedSpace`] should be used instead. Anything that implements.
pub trait KeyedSpace<K>: Space {
    fn make_keyed_state(
        &self,
        key: K,
        waypoint: Self::Waypoint,
    ) -> Self::State;

    fn key(&self, state: &Self::State) -> K;
}
