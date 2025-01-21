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

use super::{Position, Positioned, WaypointR2};
use crate::{
    domain::{
        backtrack_times, flip_endpoint_times, Backtrack, ConflictAvoider, ExtrapolationProgress,
        Extrapolator, IncrementalExtrapolator, Key, Reversible,
    },
    error::{NoError, ThisError},
    graph::Graph,
    motion::{
        self,
        conflict::{
            compute_safe_arrival_path, compute_safe_linear_path_wait_hints, SafeAction,
            WaitForObstacle,
        },
        se2, Duration, SafeArrivalTimes, SafeIntervalCache, SafeIntervalMotionError, SpeedLimiter,
    },
    util::ForkIter,
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
        from_waypoint: &WaypointR2,
        to_target: &Position,
        speed_limit: Option<f64>,
    ) -> Result<(ArrayVec<WaypointR2, 1>, WaypointR2), LineFollowError> {
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
            let wp = WaypointR2::new(from_waypoint.time, to_target.x, to_target.y);
            return Ok((ArrayVec::new(), wp));
        }

        let t = Duration::from_secs_f64(self.direction * dx / speed) + from_waypoint.time;
        let wp = WaypointR2::new(t, to_target.x, to_target.y);
        let extrap = ArrayVec::from_iter([wp]);
        Ok((extrap, wp))
    }
}

impl<Target, Guidance, Key> Extrapolator<WaypointR2, Target, Guidance, Key> for LineFollow
where
    Target: Positioned,
    Guidance: SpeedLimiter,
{
    type Extrapolation = ArrayVec<WaypointR2, 1>;
    type ExtrapolationError = LineFollowError;
    type ExtrapolationIter<'a>
        = Option<Result<(ArrayVec<WaypointR2, 1>, WaypointR2), LineFollowError>>
    where
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    fn extrapolate<'a>(
        &'a self,
        from_state: &WaypointR2,
        to_target: &Target,
        with_guidance: &Guidance,
        _: (Option<&Key>, Option<&Key>),
    ) -> Self::ExtrapolationIter<'a>
    where
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
    {
        Some(self.extrapolate_impl(from_state, &to_target.point(), with_guidance.speed_limit()))
    }
}

impl<Target, Guidance, Key> IncrementalExtrapolator<WaypointR2, Target, Guidance, Key>
    for LineFollow
where
    Target: Positioned,
    Guidance: SpeedLimiter,
{
    type IncrementalExtrapolation = ArrayVec<WaypointR2, 1>;
    type IncrementalExtrapolationError = LineFollowError;
    type IncrementalExtrapolationIter<'a>
        = Option<
        Result<(ArrayVec<WaypointR2, 1>, WaypointR2, ExtrapolationProgress), LineFollowError>,
    >
    where
        Target: 'a,
        Guidance: 'a,
        Key: 'a;

    fn incremental_extrapolate<'a>(
        &'a self,
        from_state: &WaypointR2,
        to_target: &Target,
        with_guidance: &Guidance,
        for_keys: (Option<&Key>, Option<&Key>),
    ) -> Self::IncrementalExtrapolationIter<'a>
    where
        Target: 'a,
        Guidance: 'a,
        Key: 'a,
    {
        self.extrapolate(from_state, to_target, with_guidance, for_keys)
            .map(|r| r.map(|(action, state)| (action, state, ExtrapolationProgress::Arrived)))
    }
}

impl<const N: usize> Backtrack<WaypointR2, ArrayVec<WaypointR2, N>> for LineFollow {
    type BacktrackError = NoError;
    fn flip_endpoints(
        &self,
        initial_reverse_state: &WaypointR2,
        final_reverse_state: &WaypointR2,
    ) -> Result<(WaypointR2, WaypointR2), Self::BacktrackError> {
        flip_endpoint_times(initial_reverse_state, final_reverse_state)
    }

    fn backtrack(
        &self,
        parent_forward_state: &WaypointR2,
        parent_reverse_state: &WaypointR2,
        reverse_action: &ArrayVec<WaypointR2, N>,
        child_reverse_state: &WaypointR2,
    ) -> Result<(ArrayVec<WaypointR2, N>, WaypointR2), Self::BacktrackError> {
        backtrack_times(
            parent_forward_state,
            parent_reverse_state,
            reverse_action,
            child_reverse_state,
        )
    }
}

impl<Target, Guidance, K, G: Graph<Key = K>>
    ConflictAvoider<WaypointR2, Target, Guidance, K, SafeIntervalCache<G>> for LineFollow
where
    G::Vertex: Positioned,
    Target: Positioned,
    Guidance: SpeedLimiter,
    K: Key + Clone,
{
    type AvoidanceAction = SmallVec<[SafeAction<WaypointR2, WaitForObstacle>; 5]>;
    type AvoidanceActionIter<'a>
        = impl IntoIterator<Item = Result<(Self::AvoidanceAction, WaypointR2), Self::AvoidanceError>>
        + 'a
    where
        Target: 'a,
        Guidance: 'a,
        K: 'a,
        G: 'a;

    type AvoidanceError = SafeIntervalMotionError<G::Key, LineFollowError>;

    fn avoid_conflicts<'a>(
        &'a self,
        from_point: &WaypointR2,
        to_target: &Target,
        with_guidance: &Guidance,
        (from_key, target_key): (Option<&K>, Option<&K>),
        safe_intervals: &SafeIntervalCache<G>,
    ) -> Self::AvoidanceActionIter<'a>
    where
        Self: 'a,
        Self::AvoidanceAction: 'a,
        Self::AvoidanceError: 'a,
        Target: 'a,
        Guidance: 'a,
        K: 'a,
        G: 'a,
    {
        let from_point = *from_point;
        let to_point = match self.extrapolate_impl(
            &from_point,
            &to_target.point(),
            with_guidance.speed_limit(),
        ) {
            Ok(extrapolation) => extrapolation.1,
            Err(err) => {
                return ForkIter::Left(
                    Some(Err(SafeIntervalMotionError::Extrapolator(err))).into_iter(),
                )
            }
        };

        let mut safe_arrival_times = match target_key {
            Some(target_key) => match safe_intervals.safe_intervals_for(&target_key) {
                Ok(r) => r,
                Err(err) => {
                    return ForkIter::Left(
                        Some(Err(SafeIntervalMotionError::Cache(err))).into_iter(),
                    )
                }
            },
            None => SafeArrivalTimes::new(),
        };

        let motion_key = if let (Some(from_key), Some(target_key)) = (from_key, target_key) {
            Some((from_key.clone(), target_key.clone()))
        } else {
            None
        };
        let environment_view = safe_intervals.environment().view_for(motion_key.as_ref());

        safe_arrival_times.retain(|t| *t >= to_point.time);
        // Add the time when the agent would normally arrive at the vertex.
        safe_arrival_times.insert(0, to_point.time);

        let ranked_hints =
            compute_safe_linear_path_wait_hints((&from_point, &to_point), None, &environment_view);

        let paths: SmallVec<[_; 5]> = safe_arrival_times
            .into_iter()
            .filter_map(move |arrival_time| {
                compute_safe_arrival_path(
                    from_point,
                    to_point,
                    arrival_time,
                    &ranked_hints,
                    &environment_view,
                )
            })
            .map(move |action| {
                // TODO(@mxgrey): Remove these unwraps before targeting
                // production, possibly by returning an error. We're temporarily
                // using unwrap to violently catch internal mistakes.
                let wp = *action.last().unwrap().movement().unwrap();
                Ok((action, wp))
            })
            .collect();

        ForkIter::Right(paths.into_iter())
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
        let wp0 = WaypointR2::new(t0, 1.0, -3.0);
        let movement = LineFollow::new(2.0).expect("Failed to make LineFollow");
        let p_target = Position::new(1.0, 3.0);
        let (waypoints, _) = movement
            .extrapolate(&wp0, &p_target, &(), (Some(&0), Some(&1)))
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
