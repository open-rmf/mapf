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

/// A trait for properties that can specify speed limits.]
pub trait SpeedLimiter {
    fn speed_limit(&self) -> Option<f64>;
}

/// Empty tuples `()` can be used for [`SpeedLimiter`]. They always return that
/// there is no speed limit.
impl SpeedLimiter for () {
    fn speed_limit(&self) -> Option<f64> {
        None
    }
}

/// A simple struct for implementing the SpeedLimiter trait.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedLimit(pub Option<f64>);
impl SpeedLimiter for SpeedLimit {
    fn speed_limit(&self) -> Option<f64> {
        self.0
    }
}
