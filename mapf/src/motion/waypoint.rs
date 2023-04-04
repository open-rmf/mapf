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

use super::{r2::Positioned, se2::MaybeOriented, timed, Interpolation};
use arrayvec::ArrayVec;

pub trait Waypoint:
    timed::Timed + Interpolation<Self::Position, Self::Velocity> + Clone + std::fmt::Debug
{
    /// What type of spatial position does the waypoint have
    type Position;

    /// How does the waypoint represent the time derivative of its position
    type Velocity;
}

/// A trait for states and actions to yield their waypoints
pub trait IntegrateWaypoints<W> {
    type WaypointIntegrationError;

    type IntegratedWaypointIter<'a>: IntoIterator<Item = Result<W, Self::WaypointIntegrationError>>
        + 'a
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

/// A trait for actions to provide a measurement of the distance travelled
/// between its initial and final states.
pub trait Measurable<State> {
    fn arclength(&self, from_state: &State, to_state: &State) -> Arclength;
}

/// The total distance travelled during a motion, as measured by arclength of
/// the path. Translational and rotational distance covered are separated as
/// independent measures. If a space has no rotational component (e.g. R2), then
/// the rotational will always be zero.
pub struct Arclength {
    /// Translational arclength travelled in meters
    pub translational: f64,
    /// Rotational arclength travelled in radians
    pub rotational: f64,
}

impl<S, W, const N: usize> Measurable<S> for ArrayVec<W, N>
where
    S: Positioned + MaybeOriented,
    W: Positioned + MaybeOriented,
{
    fn arclength(&self, from_state: &S, to_state: &S) -> Arclength {
        let mut translational = 0.0;
        let mut rotational = 0.0;
        let mut last_p = from_state.point();
        let mut last_yaw = from_state.maybe_oriented();

        for wp in self {
            let p = wp.point();
            translational += (p - last_p).norm();
            last_p = p;

            let yaw = wp.maybe_oriented();
            if let (Some(yaw), Some(last_yaw)) = (yaw, last_yaw) {
                rotational += (yaw / last_yaw).angle().abs();
            }

            if let Some(yaw) = yaw {
                last_yaw = Some(yaw);
            }
        }

        // The final state should be almost exactly the same as the last move
        assert!((to_state.point() - last_p).norm() < 1e-3);
        Arclength {
            translational,
            rotational,
        }
    }
}
