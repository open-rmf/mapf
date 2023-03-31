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

use super::{timed, Interpolation};

pub trait Waypoint:
    timed::Timed + Interpolation<Self::Position, Self::Velocity> + Clone + std::fmt::Debug
{
    /// What type of spatial position does the waypoint have
    type Position;

    /// How does the waypoint represent the time derivative of its position
    type Velocity;
}

pub trait IntegrateWaypoints<W> {
    type WaypointIntegrationError;

    type IntegratedWaypointIter<'a>: IntoIterator<
        Item=Result<W, Self::WaypointIntegrationError>
    > + 'a
    where
        Self: 'a,
        Self::WaypointIntegrationError: 'a,
        W: 'a;

    fn integrated_waypoints<'a>(
        &'a self,
        initial_waypoint: Option<W>,
    ) -> Self::IntegratedWaypointIter<'a>
    where
        Self: 'a,
        Self::WaypointIntegrationError: 'a,
        W: 'a;
}
