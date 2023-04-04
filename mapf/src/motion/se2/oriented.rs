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

use crate::motion::{
    r2,
    se2::{self, Orientation},
};

/// Implement this trait for structs that can express an SE(2) orientation or
/// the absence of an orientation.
pub trait MaybeOriented {
    fn maybe_oriented(&self) -> Option<Orientation>;
}

/// Implement this trait for structs that can always express an SE(2) orientation.
/// Structs with this trait must always implement [`MaybeOriented`].
pub trait Oriented: MaybeOriented {
    /// By default this is implemented by unwrapping the value of [`MaybeOriented`].
    /// There is typically no need to change this.
    fn oriented(&self) -> Orientation {
        self.maybe_oriented().unwrap()
    }
}

impl MaybeOriented for () {
    fn maybe_oriented(&self) -> Option<Orientation> {
        None
    }
}

impl MaybeOriented for r2::Position {
    fn maybe_oriented(&self) -> Option<Orientation> {
        None
    }
}

impl MaybeOriented for se2::Position {
    fn maybe_oriented(&self) -> Option<Orientation> {
        Some(self.rotation)
    }
}

impl Oriented for se2::Position {}

// NOTE: usize is commonly used as a graph key which means it could also be used
// as a goal for SE(2) planning, as long as we implement MaybeOriented for it.
impl MaybeOriented for usize {
    fn maybe_oriented(&self) -> Option<Orientation> {
        None
    }
}
