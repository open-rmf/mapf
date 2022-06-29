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
use crate::{
    motion::{self, timed, Interpolation, InterpError, Extrapolator, r2},
    error::NoError,
};
use super::{Position, Point, Vector, Velocity};
use arrayvec::ArrayVec;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub time: TimePoint,
    pub position: Position,
}

impl timed::Timed for Waypoint {
    fn time(&self) -> &TimePoint {
        return &self.time;
    }

    fn set_time(&mut self, new_time: TimePoint) {
        self.time = new_time;
    }
}

impl Waypoint {
    pub fn new(time: TimePoint, x: f64, y: f64, yaw: f64) -> Self {
        return Waypoint{
            time,
            position: Position::new(
                Vector::new(x, y),
                yaw
            )
        }
    }
}

impl motion::Waypoint for Waypoint {
    type Position = Position;
    type Velocity = Velocity;
}

impl From<Waypoint> for r2::timed_position::Waypoint {
    fn from(se2_wp: Waypoint) -> Self {
        r2::timed_position::Waypoint{
            time: se2_wp.time,
            position: se2_wp.position.translation.vector.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    initial_wp: Waypoint,
    final_wp: Waypoint,
}

impl Motion {
    pub fn in_time_range(&self, time: &TimePoint) -> Result<(), InterpError> {
        if time.nanos_since_zero < self.initial_wp.time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
        }

        if self.final_wp.time.nanos_since_zero < time.nanos_since_zero {
            return Err(InterpError::OutOfBounds);
        }

        return Ok(());
    }
}

impl motion::Motion<Position, Velocity> for Motion {
    fn compute_position(&self, time: &TimePoint) -> Result<Position, InterpError> {
        self.in_time_range(time)?;
        let delta_t = (*time - self.initial_wp.time).as_secs_f64();
        let t_range = (self.final_wp.time - self.initial_wp.time).as_secs_f64();
        return Ok(self.initial_wp.position.lerp_slerp(&self.final_wp.position, delta_t/t_range));
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<Velocity, InterpError> {
        self.in_time_range(time)?;

        // TODO(MXG): Since velocity is taken to be constant across the whole
        // range, this could be precomputed once and saved inside the Motion object.
        let t_range = (self.final_wp.time - self.initial_wp.time).as_secs_f64();
        let p0 = &self.initial_wp.position.translation.vector;
        let p1 = &self.final_wp.position.translation.vector;
        let linear_v = (p1 - p0)/t_range;

        let r0 = &self.initial_wp.position.rotation;
        let r1 = &self.final_wp.position.rotation;
        let angular_v = (r1/r0).angle()/t_range;
        return Ok(
            Velocity{
                translational: linear_v,
                rotational: angular_v,
            }
        );
    }
}

impl Interpolation<Position, Velocity> for Waypoint {
    type Motion = Motion;

    fn interpolate(&self, up_to: &Self) -> Self::Motion {
        return Self::Motion{
            initial_wp: self.clone(),
            final_wp: up_to.clone()
        }
    }
}


/// What kind of steering does the agent have
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Steering {
    /// The agent uses a differential drive, meaning it needs to face the
    /// direction that it is translating towards.
    Differential,

    /// The agent has holonomic motion, meaning it can translate and rotate
    /// simultaneously and independently.
    Holonomic
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveLineFollow {
    /// What is the nominal translational speed that the agent will move with
    translational_speed: f64,

    /// What is the nominal rotaional speed that the agent will move with
    rotational_speed: f64,

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
            translational_threshold: motion::DEFAULT_TRANSLATIONAL_THRESHOLD,
            rotational_threshold: motion::DEFAULT_ROTATIONAL_THRESHOLD,
        })
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
    ) -> Result<ReachedTarget, NoError> {
        // NOTE: We trust that all properties in self have values greater
        // than zero because we enforce that for all inputs.
        let mut output: ArrayVec<Waypoint, 3> = ArrayVec::new();
        let mut current_time = from_waypoint.time;
        let mut current_yaw = from_waypoint.position.rotation;

        let p0 = Point::from(from_waypoint.position.translation.vector);
        let p1 = to_target;
        let delta_p = *p1 - p0;
        let distance = delta_p.norm();
        if distance > self.translational_threshold {
            let approach_yaw = nalgebra::UnitComplex::from_angle(delta_p[1].atan2(delta_p[0]));
            let delta_yaw_abs = (approach_yaw / from_waypoint.position.rotation).angle().abs();
            if delta_yaw_abs > self.rotational_threshold {
                current_time += time_point::Duration::from_secs_f64(delta_yaw_abs/self.rotational_speed);
                output.push(Waypoint {
                    time: current_time,
                    position: Position::from_parts(
                        from_waypoint.position.translation,
                        approach_yaw
                    )
                });
            }

            current_yaw = approach_yaw;
            current_time += time_point::Duration::from_secs_f64(distance/self.translational_speed);
            output.push(Waypoint{
                time: current_time,
                position: Position::new(p1.coords, approach_yaw.angle())
            });
        }

        return Ok(ReachedTarget{
            waypoints: output,
            time: current_time,
            yaw: current_yaw
        })
    }
}

struct ReachedTarget {
    waypoints: ArrayVec<Waypoint, 3>,
    time: TimePoint,
    yaw: nalgebra::UnitComplex<f64>,
}

impl Extrapolator<Waypoint, Position> for DifferentialDriveLineFollow {
    type Extrapolation<'a> = ArrayVec<Waypoint, 3>;
    type Error = NoError;

    fn extrapolate(
        &self,
        from_waypoint: &Waypoint,
        to_position: &Position
    ) -> Result<ArrayVec<Waypoint, 3>, NoError> {
        let mut arrival = self.move_towards_target(
            from_waypoint, &Point::from(to_position.translation.vector)
        )?;

        let delta_yaw_abs = (to_position.rotation / arrival.yaw).angle().abs();
        if delta_yaw_abs > self.rotational_threshold {
            // Rotate towards the target orientation if we're not already facing
            // it.
            arrival.time += time_point::Duration::from_secs_f64(
                delta_yaw_abs/self.rotational_speed
            );
            arrival.waypoints.push(Waypoint{
                time: arrival.time,
                position: *to_position
            });
        }

        return Ok(arrival.waypoints);
    }
}

impl Extrapolator<Waypoint, Point> for DifferentialDriveLineFollow {
    type Extrapolation<'a> = ArrayVec<Waypoint, 3>;
    type Error = NoError;

    fn extrapolate<'a>(
        &'a self,
        from_waypoint: &Waypoint,
        to_target: &Point,
    ) -> Result<ArrayVec<Waypoint, 3>, NoError> {
        self.move_towards_target(
            from_waypoint, to_target
        ).map(|arrival| arrival.waypoints )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motion::Motion;
    use approx::assert_relative_eq;

    #[test]
    fn test_interpolation() {
        let t0 = time_point::TimePoint::new(0);
        let t1 = t0 + time_point::Duration::from_secs_f64(2.0);
        let wp0 = Waypoint::new(t0, 1.0, 5.0, 10f64.to_radians());
        let wp1 = Waypoint::new(t1, 1.0, 10.0, -20f64.to_radians());

        let motion = wp0.interpolate(&wp1);
        let t = (t1 - t0)/2f64 + t0;
        let p = motion.compute_position(&t).ok().unwrap();
        assert_relative_eq!(p.translation.vector[0], 1f64, max_relative = 0.001);
        assert_relative_eq!(p.translation.vector[1], 7.5f64, max_relative = 0.001);
        assert_relative_eq!(p.rotation.angle(), -5f64.to_radians(), max_relative = 0.001);

        let v = motion.compute_velocity(&t).ok().unwrap();
        assert_relative_eq!(v.translational[0], 0f64, max_relative = 0.001);
        assert_relative_eq!(v.translational[1], 5.0/2.0, max_relative = 0.001);
        assert_relative_eq!(v.rotational, -30f64.to_radians()/2.0, max_relative = 0.001);
    }

    #[test]
    fn test_extrapolation() {
        let t0 = time_point::TimePoint::from_secs_f64(3.0);
        let wp0 = Waypoint::new(t0, 1.0, -3.0, -40f64.to_radians());
        let movement = DifferentialDriveLineFollow::new(2.0, 3.0).expect("Failed to make DifferentialLineFollow");
        let p_target = Position::new(Vector::new(1.0, 3.0), 60f64.to_radians());
        let waypoints = movement.extrapolate(&wp0, &p_target).expect("Failed to extrapolate");
        assert_eq!(waypoints.len(), 3);
        assert_relative_eq!(
            waypoints.last().unwrap().time.as_secs_f64(),
            (t0 + time_point::Duration::from_secs_f64(
                ((90f64 - (-40f64)).abs() + (60f64 - 90f64).abs()).to_radians() / movement.rotational_speed()
                + 6f64 / movement.translational_speed
            )).as_secs_f64()
        );

        assert_relative_eq!(
            waypoints.last().unwrap().position.translation.vector[0],
            p_target.translation.vector[0]
        );
        assert_relative_eq!(
            waypoints.last().unwrap().position.translation.vector[1],
            p_target.translation.vector[1]
        );
        assert_relative_eq!(
            waypoints.last().unwrap().position.rotation.angle(),
            p_target.rotation.angle()
        );

        let mut trajectory = motion::se2::LinearTrajectory::from_iter(waypoints.into_iter())
            .expect("Failed to create trajectory");
        trajectory.insert(wp0).expect("Waypoint insertion failed");
        assert_eq!(trajectory.len(), 4);
    }
}
