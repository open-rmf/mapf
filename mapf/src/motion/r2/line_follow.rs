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

use super::{Position, Waypoint, Positioned};
use crate::{
    motion::{
        self, SpeedLimiter, se2, Duration, DynamicEnvironment,
        conflict::{compute_safe_linear_paths, SafeAction, WaitForObstacle},
    },
    domain::{
        Extrapolator, IncrementalExtrapolator, ExtrapolationProgress,
        Reversible, Backtrack, flip_endpoint_times, backtrack_times,
        ConflictAvoider,
    },
    error::{NoError, ThisError},
    util::FlatResultMapTrait,
};
use arrayvec::ArrayVec;
use smallvec::SmallVec;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineFollow {
    speed: f64,
    direction: f64,
    distance_threshold: f64,
}

impl LineFollow {
    pub fn new(speed: f64) -> Result<Self, ()> {
        if speed <= 0.0 {
            return Err(());
        }

        Ok(LineFollow {
            speed,
            direction: 1.0,
            distance_threshold: motion::DEFAULT_TRANSLATIONAL_THRESHOLD,
        })
    }

    pub fn set_speed(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.speed = value;
        Ok(())
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    fn extrapolate_impl(
        &self,
        from_waypoint: &Waypoint,
        to_target: &Position,
        speed_limit: Option<f64>,
    ) -> Result<Option<(ArrayVec<Waypoint, 1>, Waypoint)>, LineFollowError> {
        let speed = if let Some(limit) = speed_limit {
            if limit <= 0.0 {
                return Err(LineFollowError::InvalidSpeedLimit(limit));
            }
            self.speed.min(limit)
        } else {
            self.speed
        };

        let dx = (to_target - from_waypoint.position).norm();
        if dx <= self.distance_threshold {
            // The target is close enough to the start point that we treat it
            // as though the agent is already there.
            let wp = Waypoint::new(from_waypoint.time, to_target.x, to_target.y);
            return Ok(Some((ArrayVec::new(), wp)));
        }

        let t = Duration::from_secs_f64(self.direction * dx / speed) + from_waypoint.time;
        let wp = Waypoint::new(t, to_target.x, to_target.y);
        let extrap = ArrayVec::from_iter([wp]);
        Ok(Some((extrap, wp)))
    }
}

impl<Target, Guidance> Extrapolator<Waypoint, Target, Guidance> for LineFollow
where
    Target: Positioned,
    Guidance: SpeedLimiter,
{
    type Extrapolation = ArrayVec<Waypoint, 1>;
    type ExtrapolationError = LineFollowError;
    type ExtrapolationIter<'a> = Option<Result<(ArrayVec<Waypoint, 1>, Waypoint), LineFollowError>>
    where
        Target: 'a,
        Guidance: 'a;

    fn extrapolate<'a>(
        &'a self,
        from_state: &Waypoint,
        to_target: &Target,
        with_guidance: &Guidance,
    ) -> Self::ExtrapolationIter<'a>
    where
        Target: 'a,
        Guidance: 'a,
    {
        self.extrapolate_impl(from_state, &to_target.point(), with_guidance.speed_limit()).transpose()
    }
}

impl<Target, Guidance> IncrementalExtrapolator<Waypoint, Target, Guidance> for LineFollow
where
    Target: Positioned,
    Guidance: SpeedLimiter,
{
    type IncrementalExtrapolation = ArrayVec<Waypoint, 1>;
    type IncrementalExtrapolationError = LineFollowError;
    type IncrementalExtrapolationIter<'a> = Option<Result<(ArrayVec<Waypoint, 1>, Waypoint, ExtrapolationProgress), LineFollowError>>
    where
        Target: 'a,
        Guidance: 'a;

    fn incremental_extrapolate<'a>(
        &'a self,
        from_state: &Waypoint,
        to_target: &Target,
        with_guidance: &Guidance,
    ) -> Self::IncrementalExtrapolationIter<'a>
    where
        Target: 'a,
        Guidance: 'a,
    {
        self.extrapolate(from_state, to_target, with_guidance)
            .map(|r| r.map(|(action, state)| (action, state, ExtrapolationProgress::Arrived)))
    }
}

impl<const N: usize> Backtrack<Waypoint, ArrayVec<Waypoint, N>> for LineFollow {
    type BacktrackError = NoError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &Waypoint,
        final_reverse_state: &Waypoint,
    ) -> Result<(Waypoint, Waypoint), Self::BacktrackError> {
        flip_endpoint_times(initial_reverse_state, final_reverse_state)
    }

    fn backtrack(
        &self,
        parent_forward_state: &Waypoint,
        parent_reverse_state: &Waypoint,
        reverse_action: &ArrayVec<Waypoint, N>,
        child_reverse_state: &Waypoint,
    ) -> Result<(ArrayVec<Waypoint, N>, Waypoint), Self::BacktrackError> {
        backtrack_times(
            parent_forward_state,
            parent_reverse_state,
            reverse_action,
            child_reverse_state
        )
    }
}

impl<Target, Guidance, W> ConflictAvoider<Waypoint, Target, Guidance, DynamicEnvironment<W>> for LineFollow
where
    Target: Positioned,
    Guidance: SpeedLimiter,
    W: motion::Waypoint + Into<Waypoint>,
{
    type AvoidanceAction = SmallVec<[SafeAction<Waypoint, WaitForObstacle>; 5]>;
    type AvoidanceActionIter<'a> = impl IntoIterator<Item=Result<(Self::AvoidanceAction, Waypoint), LineFollowError>> + 'a
    where
        Target: 'a,
        Guidance: 'a,
        W: 'a;

    type AvoidanceError = LineFollowError;

    fn avoid_conflicts<'a>(
        &'a self,
        from_point: &Waypoint,
        to_target: &Target,
        with_guidance: &Guidance,
        in_environment: &'a DynamicEnvironment<W>,
    ) -> Self::AvoidanceActionIter<'a>
    where
        Self: 'a,
        Self::AvoidanceAction: 'a,
        Self::AvoidanceError: 'a,
        Target: 'a,
        Guidance: 'a,
        DynamicEnvironment<W>: 'a
    {
        let from_point = *from_point;
        self
        .extrapolate(&from_point, to_target, with_guidance)
        .into_iter()
        .flat_map(move |r|
            r.flat_result_map(move |(_, to_point)|
                compute_safe_linear_paths(from_point, to_point, in_environment)
                .into_iter()

                // TODO(@mxgrey): Should we pass an error if the last
                // element in the action is not a movement? That should
                // never happen, so being quiet about it might not be a
                // good thing.
                // .filter_map(move |action| {
                //     let wp = action
                //         .last()
                //         .map(|m| m.movement())
                //         .flatten()
                //         .copied();

                //     wp.map(move |wp| (action, wp))
                // })

                .map(move |action| {
                    // TODO(@mxgrey): Remove these unwraps before targeting
                    // production. Either use the map technique that's commented
                    // out above or return an error. I'm temporarily using
                    // unwrap to violently catch internal mistakes.
                    let wp = *action
                        .last().unwrap()
                        .movement().unwrap();

                    (action, wp)
                })
            )
        )
    }
}

#[derive(Debug, ThisError, Clone, Copy)]
pub enum LineFollowError {
    #[error("provided with an invalid speed limit (must be >0.0): {0}")]
    InvalidSpeedLimit(f64),
}

impl Reversible for LineFollow {
    type ReversalError = NoError;

    fn reversed(&self) -> Result<Self, NoError> {
        Ok(Self {
            speed: self.speed,
            direction: -1.0 * self.direction,
            distance_threshold: self.distance_threshold,
        })
    }
}

impl From<se2::DifferentialDriveLineFollow> for LineFollow {
    fn from(other: se2::DifferentialDriveLineFollow) -> Self {
        LineFollow::new(other.translational_speed())
            .expect("corrupt speed in DifferentialDriveLineFollow")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::TimePoint;
    use approx::assert_relative_eq;

    #[test]
    fn test_extrapolation() {
        let t0 = TimePoint::from_secs_f64(3.0);
        let wp0 = Waypoint::new(t0, 1.0, -3.0);
        let movement = LineFollow::new(2.0).expect("Failed to make LineFollow");
        let p_target = Position::new(1.0, 3.0);
        let (waypoints, _) = movement
            .extrapolate(&wp0, &p_target, &())
            .expect("Failed to extrapolate")
            .expect("Missing extrapolation result");
        assert_eq!(waypoints.len(), 1);
        assert_relative_eq!(
            waypoints.last().unwrap().time.as_secs_f64(),
            (t0 + Duration::from_secs_f64(6.0 / 2.0)).as_secs_f64()
        );

        assert_relative_eq!(waypoints.last().unwrap().position[0], p_target[0]);

        assert_relative_eq!(waypoints.last().unwrap().position[1], p_target[1]);

        let trajectory = motion::r2::LinearTrajectory::from_iter(
            [wp0].into_iter().chain(waypoints.iter().map(|wp| *wp)),
        )
        .expect("Failed to create trajectory");
        assert_eq!(trajectory.len(), 2);
    }
}
