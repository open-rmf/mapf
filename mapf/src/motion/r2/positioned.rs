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

use crate::motion::{r2::Point, se2};

pub trait MaybePositioned {
    fn maybe_point(&self) -> Option<Point>;
}

pub trait Positioned: MaybePositioned {
    fn point(&self) -> Point {
        self.maybe_point().unwrap()
    }
}

impl MaybePositioned for Point {
    fn maybe_point(&self) -> Option<Point> {
        Some(*self)
    }
}

impl Positioned for Point {}

impl MaybePositioned for se2::Position {
    fn maybe_point(&self) -> Option<Point> {
        Some(self.translation.vector.into())
    }
}

impl Positioned for se2::Position {}

impl MaybePositioned for usize {
    fn maybe_point(&self) -> Option<Point> {
        None
    }
}
