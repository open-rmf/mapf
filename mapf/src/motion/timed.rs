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

use time_point::TimePoint;

pub trait Timed {
    fn time(&self) -> &TimePoint;
}

pub(crate) mod private {
    /// This is a private trait that should only be used internally by trajectory.
    /// We do not want users arbitrarily setting the times of waypoints because
    /// changing the time goes hand-in-hand with the ordering of waypoints inside
    /// the trajectory.
    pub trait SetTime {
        fn set_time(&mut self, new_time: super::TimePoint);
    }
}
