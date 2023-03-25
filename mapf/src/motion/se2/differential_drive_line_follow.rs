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

use super::{Point, Position, Waypoint, MaybeOriented, StateSE2};
use crate::{
    motion::{self, SpeedLimiter, Duration, r2::{self, MaybePositioned}},
    domain::{
        Extrapolator, IncrementalExtrapolator, SelfKey, Connectable, Reversible,
        ExtrapolationProgress, Backtrack, flip_endpoint_times, backtrack_times,
    },
    error::{ThisError, NoError},
};
use arrayvec::ArrayVec;
use time_point::TimePoint;
use std::borrow::Borrow;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveLineFollow {
    /// What is the nominal translational speed that the agent will move with
    translational_speed: f64,

    /// What is the nominal rotaional speed that the agent will move with
    rotational_speed: f64,

    /// Are we extrapolating forward (+1.0) or backward (-1.0) in time?
    direction: f64,

    /// If the initial waypoint is within this translational threshold of the
    /// target, no translation will be performed when extrapolating.
    translational_threshold: f64,

    /// If the initial waypoint is within this rotational threshold (in radians)
    /// then rotation may be skipped while extrapolating.
    rotational_threshold: f64,
}

impl DifferentialDriveLineFollow {
    /// Make a new movement description. If one of the requested values is
    /// invalid, then an error will be returned. Make sure both values are
    /// greater than zero.
    pub fn new(translational_speed: f64, rotational_speed: f64) -> Result<Self, ()> {
        if translational_speed <= 0.0 {
            return Err(());
        }

        if rotational_speed <= 0.0 {
            return Err(());
        }

        return Ok(DifferentialDriveLineFollow {
            translational_speed,
            rotational_speed,
            direction: 1.0,
            translational_threshold: motion::DEFAULT_TRANSLATIONAL_THRESHOLD,
            rotational_threshold: motion::DEFAULT_ROTATIONAL_THRESHOLD,
        });
    }

    pub fn set_translational_speed(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.translational_speed = value;
        return Ok(());
    }

    pub fn set_rotational_speed(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.rotational_speed = value;
        return Ok(());
    }

    pub fn set_translational_threshold(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.translational_threshold = value;
        return Ok(());
    }

    pub fn set_rotational_threshold(&mut self, value: f64) -> Result<(), ()> {
        if value <= 0.0 {
            return Err(());
        }

        self.rotational_threshold = value;
        return Ok(());
    }

    pub fn translational_speed(&self) -> f64 {
        return self.translational_speed;
    }

    pub fn rotational_speed(&self) -> f64 {
        return self.rotational_speed;
    }

    pub fn translational_threshold(&self) -> f64 {
        return self.translational_threshold;
    }

    pub fn rotational_threshold(&self) -> f64 {
        return self.rotational_threshold;
    }

    /// Helper function for the implementations of extrapolate(). Not meant for
    /// use by other functions
    fn move_towards_target(
        &self,
        from_waypoint: &Waypoint,
        to_target: &Point,
        speed_limiter: &impl SpeedLimiter,
    ) -> Result<ReachedTarget, DifferentialDriveLineFollowError> {
        // NOTE: We trust that all properties in self have values greater
        // than zero because we enforce that for all inputs.
        let mut output: ArrayVec<Waypoint, 3> = ArrayVec::new();
        let mut current_time = from_waypoint.time;
        let mut current_yaw = from_waypoint.position.rotation;

        let translational_speed = speed_limiter
            .speed_limit()
            .map(|s| s.min(self.translational_speed))
            .unwrap_or(self.translational_speed);

        let p0 = Point::from(from_waypoint.position.translation.vector);
        let p1 = to_target;
        let delta_p = self.direction * (*p1 - p0);
        let distance = delta_p.norm();
        if distance > self.translational_threshold {
            let approach_yaw = nalgebra::UnitComplex::from_angle(f64::atan2(delta_p[1], delta_p[0]));
            let delta_yaw_abs = (approach_yaw / from_waypoint.position.rotation)
                .angle()
                .abs();
            if delta_yaw_abs > self.rotational_threshold {
                current_time += time_point::Duration::from_secs_f64(
                    self.direction * delta_yaw_abs / self.rotational_speed
                );
                output.push(Waypoint {
                    time: current_time,
                    position: Position::from_parts(
                        from_waypoint.position.translation,
                        approach_yaw,
                    ),
                });
            }

            current_yaw = approach_yaw;
            current_time += time_point::Duration::from_secs_f64(
                self.direction * distance / translational_speed
            );
            output.push(Waypoint {
                time: current_time,
                position: Position::new(p1.coords, approach_yaw.angle()),
            });
        }

        return Ok(ReachedTarget {
            waypoints: output,
            time: current_time,
            yaw: current_yaw,
        });
    }
}

struct ReachedTarget {
    waypoints: ArrayVec<Waypoint, 3>,
    time: TimePoint,
    yaw: nalgebra::UnitComplex<f64>,
}

impl<Target, Guidance> Extrapolator<Waypoint, Target, Guidance> for DifferentialDriveLineFollow
where
    Target: r2::Positioned + MaybeOriented,
    Guidance: SpeedLimiter,
{
    type Extrapolation = ArrayVec<Waypoint, 3>;
    type ExtrapolationError = DifferentialDriveLineFollowError;
    type ExtrapolationIter<'a> = Option<Result<(Self::Extrapolation, Waypoint), Self::ExtrapolationError>>
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
        let target_point = to_target.point();
        let mut arrival = match self.move_towards_target(
            from_state,
            &target_point,
            with_guidance,
        ) {
            Ok(arrival) => arrival,
            Err(err) => return Some(Err(err)),
        };

        if let Some(target_yaw) = to_target.maybe_oriented() {
            let delta_yaw_abs = (target_yaw / arrival.yaw).angle().abs();
            if delta_yaw_abs > self.rotational_threshold {
                // Rotate towards the target orientation if we're not already facing
                // it.
                arrival.time += Duration::from_secs_f64(
                    self.direction * delta_yaw_abs / self.rotational_speed
                );
                arrival.waypoints.push(Waypoint {
                    time: arrival.time,
                    position: Position::new(
                        target_point.coords,
                        target_yaw.angle(),
                    ),
                });
            }
        }

        let wp = *arrival.waypoints.last().unwrap_or(from_state);
        return Some(Ok((arrival.waypoints, wp)));
    }
}

impl<Target, Guidance> IncrementalExtrapolator<Waypoint, Target, Guidance> for DifferentialDriveLineFollow
where
    Target: r2::Positioned + MaybeOriented,
    Guidance: SpeedLimiter,
{
    type IncrementalExtrapolation = ArrayVec<Waypoint, 1>;
    type IncrementalExtrapolationError = DifferentialDriveLineFollowError;
    type IncrementalExtrapolationIter<'a> = Option<Result<
        (Self::IncrementalExtrapolation, Waypoint, ExtrapolationProgress),
        Self::IncrementalExtrapolationError
    >>
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
        let target_point = to_target.point();
        let mut arrival = match self.move_towards_target(
            from_state,
            &target_point,
            with_guidance,
        ) {
            Ok(arrival) => arrival,
            Err(err) => return Some(Err(err)),
        };

        let mut action = ArrayVec::new();
        if let Some(next_increment) = arrival.waypoints.first() {
            action.push(*next_increment);
            if arrival.waypoints.len() > 1 {
                return Some(Ok((action, *next_increment, ExtrapolationProgress::Incomplete)));
            }
        }

        if let Some(target_yaw) = to_target.maybe_oriented() {
            let delta_yaw_abs = (target_yaw / arrival.yaw).angle().abs();
            if delta_yaw_abs > self.rotational_threshold {
                if let Some(next_increment) = action.first().map(|wp| *wp) {
                    return Some(Ok((action, next_increment, ExtrapolationProgress::Incomplete)));
                } else {
                    // Rotate towards the target orientation if we're not
                    // already facing it.
                    arrival.time += Duration::from_secs_f64(delta_yaw_abs / self.rotational_speed);
                    let wp = Waypoint {
                        time: arrival.time,
                        position: Position::new(
                            target_point.coords,
                            target_yaw.angle(),
                        ),
                    };
                    action.push(wp);
                    return Some(Ok((action, wp, ExtrapolationProgress::Arrived)));
                }
            }
        }

        let wp = *action.first().unwrap_or(from_state);
        Some(Ok((action, wp, ExtrapolationProgress::Arrived)))
    }
}

impl<const N: usize> Backtrack<Waypoint, ArrayVec<Waypoint, N>> for DifferentialDriveLineFollow {
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

#[derive(Debug, ThisError, Clone, Copy)]
pub enum DifferentialDriveLineFollowError {
    #[error("provided with an invalid speed limit (must be >0.0): {0}")]
    InvalidSpeedLimit(f64),
}

impl Reversible for DifferentialDriveLineFollow {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> {
        Ok(Self {
            direction: -self.direction,
            ..self.clone()
        })
    }
}

#[derive(Debug, Clone)]
pub struct MergeIntoGoal<const R: u32>(pub DifferentialDriveLineFollow);

impl<K, Target, const R: u32> Connectable<StateSE2<K, R>, ArrayVec<Waypoint, 3>, Target> for MergeIntoGoal<R>
where
    Target: MaybePositioned + MaybeOriented + SelfKey<Key=K>,
    K: PartialEq + std::fmt::Debug,
{
    type ConnectionError = DifferentialDriveLineFollowError;
    type Connections<'a> = Option<Result<(ArrayVec<Waypoint, 3>, StateSE2<K, R>), Self::ConnectionError>>
    where
        K: 'a,
        Target: 'a;

    fn connect<'a>(
        &'a self,
        from_state: StateSE2<K, R>,
        to_target: &'a Target,
    ) -> Self::Connections<'a>
    where
        Self: 'a,
        Self::ConnectionError: 'a,
        StateSE2<K, R>: 'a,
        ArrayVec<Waypoint, 3>: 'a,
        Target: 'a,
    {
        let goal_key = to_target.key();
        if from_state.key.vertex != *goal_key.borrow() {
            return None;
        }

        let target_pos = to_target.maybe_point();
        let target_orientation = to_target.maybe_oriented();
        if target_pos.is_none() && target_orientation.is_none() {
            // If there isn't a position or orientation specified for the goal,
            // then simply return. We don't need to do anything special to reach
            // the goal.
            return None;
        }

        let target_pos = Position::from_parts(
            target_pos.unwrap_or(from_state.waypoint.position.translation.vector.into()).into(),
            target_orientation.unwrap_or(from_state.waypoint.position.rotation),
        );

        self.0.extrapolate(&from_state.waypoint, &target_pos, &())
        .map(|r|
            r.map(|(action, wp)|
                (action, StateSE2::new(from_state.key.vertex, wp))
            )
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::se2::Vector;
    use approx::assert_relative_eq;

    #[test]
    fn test_extrapolation() {
        let t0 = time_point::TimePoint::from_secs_f64(3.0);
        let wp0 = Waypoint::new(t0, 1.0, -3.0, -40f64.to_radians());
        let movement = DifferentialDriveLineFollow::new(2.0, 3.0)
            .expect("Failed to make DifferentialLineFollow");
        let p_target = Position::new(Vector::new(1.0, 3.0), 60f64.to_radians());
        let (waypoints, end) = movement
            .extrapolate(&wp0, &p_target, &())
            .expect("Failed to extrapolate")
            .expect("The extrapolation should have produced a path");
        assert_eq!(waypoints.len(), 3);
        assert_relative_eq!(
            waypoints.last().unwrap().time.as_secs_f64(),
            (t0 + time_point::Duration::from_secs_f64(
                ((90f64 - (-40f64)).abs() + (60f64 - 90f64).abs()).to_radians()
                    / movement.rotational_speed()
                    + 6f64 / movement.translational_speed
            ))
            .as_secs_f64()
        );

        assert_relative_eq!(
            end.position.translation.vector[0],
            p_target.translation.vector[0]
        );
        assert_relative_eq!(
            end.position.translation.vector[1],
            p_target.translation.vector[1]
        );
        assert_relative_eq!(
            end.position.rotation.angle(),
            p_target.rotation.angle()
        );

        let mut trajectory = motion::se2::LinearTrajectory::from_iter(waypoints.into_iter())
            .expect("Failed to create trajectory");
        trajectory.insert(wp0).expect("Waypoint insertion failed");
        assert_eq!(trajectory.len(), 4);
    }
}