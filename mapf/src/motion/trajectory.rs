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

use super::{TimePoint, Duration, Motion, Interpolation, timed::TimeCmp};
use sorted_vec::{SortedSet, FindOrInsert};
use cached::UnboundCache;
use std::cell::RefCell;

pub trait Waypoint:
    super::timed::Timed
    + Interpolation<Self::Position, Self::Velocity>
    + Clone
    + std::fmt::Debug {

    type Position;
    type Velocity;
    type Motion: Motion<Self::Position, Self::Velocity>;
}

pub enum Find {
    /// The requested time is exactly on the waypoint at this index
    Exact(usize),

    /// The requested time is approaching the waypoint of this index
    Approaching(usize),

    /// The requested time is before the start of the trajectory
    BeforeStart,

    /// The requested time is after the trajectory is finished
    AfterFinish,
}

pub enum RemovalError {
    /// The specified index or range is outside the index bounds of the
    /// trajectory.
    OutOfBounds,

    /// The requested removal would leave the trajectory with fewer than 2
    /// waypoints.
    Depleting,
}

pub enum MutateError {
    /// The requested index is out of the bounds of the trajectory.
    OutOfBounds,

    /// The time of the waypoint was shifted too high or too low, which would
    /// cause its order within the trajectory to change. Its time value has been
    /// changed back to its original value.
    InvalidTimeChange,
}

#[derive(Clone, Debug)]
pub struct Trajectory<W>
where W: Waypoint {

    waypoints: SortedSet<TimeCmp<W>>,
    indefinite_start: bool,
    indefinite_finish: bool
}

impl<'a, W: Waypoint> Trajectory<W> {

    /// Create a new trajectory, starting with the given endpoints. If the
    /// endpoints have the same time value then this will return an Err.
    pub fn new(start: W, finish: W) -> Result<Self, ()> {
        if start.time() == finish.time() {
            return Result::Err(());
        }

        let mut result = Self{
            waypoints: SortedSet::new(),
            indefinite_start: false,
            indefinite_finish: false
        };

        result.waypoints.push(TimeCmp(start));
        result.waypoints.push(TimeCmp(finish));
        return Result::Ok(result);
    }

    /// Create a new trajectory that holds at a waypoint until a certain time.
    /// If the finish time is equal to the start time, then this will return an
    /// Err.
    pub fn hold(from: W, until: TimePoint) -> Result<Self, ()> {
        if *from.time() == until {
            return Result::Err(());
        }

        let mut finish = from.clone();
        finish.set_time(until);
        return Self::new(from, finish);
    }

    /// Drains elements out of the given iterator type and constructs a
    /// Trajectory with them. If the final number of elements that would be in
    /// the trajectory is less than 2, then this function returns an Err.
    pub fn from_iter<I: Iterator<Item=W>>(iter: I) -> Result<Self, ()> {
        let mut result = Self{
            waypoints: SortedSet::new(),
            indefinite_start: false,
            indefinite_finish: false
        };

        for element in iter {
            result.waypoints.push(TimeCmp(element));
        }

        if result.waypoints.len() < 2 {
            return Result::Err(());
        } else {
            return Result::Ok(result);
        }
    }

    /// Attemtps to insert the waypoint into this trajectory. If a waypoint
    /// already exists with the same time value as the waypoint, then this
    /// returns an error with the index of the existing waypoint.
    pub fn insert(&mut self, waypoint: W) -> Result<usize, usize> {
        match self.waypoints.find_or_push(TimeCmp(waypoint)) {
            FindOrInsert::Found(index) => Result::Err(index),
            FindOrInsert::Inserted(index) => Result::Ok(index)
        }
    }

    /// Inserts the waypoint into this trajectory, or if the trajectory already
    /// has a waypoint at an equivalent time, then the value of this waypoint
    /// gets assigned to the old one.
    pub fn insert_or_assign(&mut self, waypoint: W) -> usize {
        return self.waypoints.insert(TimeCmp(waypoint));
    }

    /// Attempt to remove the waypoint at the specified index. If the erasure
    /// would leave less than 2 waypoints in the trajectory, then this function
    /// has no effect and instead returns an Err.
    pub fn remove_index(&mut self, index: usize) -> Result<(), RemovalError> {
        if self.waypoints.len() <= 2 {
            return Result::Err(RemovalError::Depleting);
        }

        if index >= self.waypoints.len() {
            return Result::Err(RemovalError::OutOfBounds);
        }

        self.waypoints.remove_index(index);
        return Result::Ok(());
    }

    /// Find the segment of the trajectory that matches this point in time.
    pub fn find(&self, time: &TimePoint) -> Find {
        match self.waypoints.binary_search_by(|x| x.partial_cmp(time).unwrap())  {
            Result::Ok(index) => return Find::Exact(index),
            Result::Err(index) => {
                if index == 0 {
                    return Find::BeforeStart;
                } else if index == self.waypoints.len() {
                    return Find::AfterFinish;
                } else {
                    return Find::Approaching(index);
                }
            }
        }
    }

    /// Simultaneously adjust the times of all the waypoints in the trajectory
    pub fn adjust_times(&mut self, by: Duration) {
        // SAFETY: Uniformly changing the times of all the waypoints inside the
        // trajectory cannot change their relative ordering.
        unsafe {
            let vec = self.waypoints.get_unchecked_mut_vec();
            for element in vec.iter_mut() {
                let new_time = *element.0.time() + by;
                element.0.set_time(new_time);
            }
        }
    }

    /// Make changes to the waypoint at a specified index. If a change is made
    /// to the waypoint's time that would cause its order within the vector to
    /// change, then its time value will be reverted back to the original and
    /// an error will be returned.
    pub fn mutate_waypoint<F: FnOnce(&mut W)>(&mut self, index: usize, f: F)
    -> Result<(), MutateError> {
        if index >= self.waypoints.len() {
            return Result::Err(MutateError::OutOfBounds);
        }

        let mut lower_bound_opt = Option::None;
        let mut upper_bound_opt = Option::None;

        // SAFETY: All accesses of the vec are done within the checked bounds,
        // and we ensure that changes to the time value of the waypoint will
        // keep it within its current location in the trajectory.
        unsafe {
            let vec = self.waypoints.get_unchecked_mut_vec();
            if index > 0 {
                lower_bound_opt = Some(vec.get_unchecked(index - 1).0.time().clone());
            }

            if index < vec.len() - 1 {
                upper_bound_opt = Some(vec.get_unchecked(index + 1).0.time().clone());
            }

            let wp = vec.get_unchecked_mut(index);
            let original_time = wp.0.time().clone();

            f(&mut wp.0);

            if let Some(lower_bound) = lower_bound_opt {
                if *wp.0.time() <= lower_bound {
                    wp.0.set_time(original_time);
                    return Result::Err(MutateError::InvalidTimeChange);
                }
            }

            if let Some(upper_bound) = upper_bound_opt {
                if *wp.0.time() >= upper_bound {
                    wp.0.set_time(original_time);
                    return Result::Err(MutateError::InvalidTimeChange);
                }
            }
        }

        return Result::Ok(());
    }

    /// Get a motion for this trajectory
    pub fn motion(&'a self) -> TrajectoryMotion<'a, W> {
        return TrajectoryMotion{
            trajectory: self,
            motion_cache: RefCell::new(UnboundCache::new())
        }
    }
}

pub struct TrajectoryMotion<'a, W: Waypoint> {
    trajectory: &'a Trajectory<W>,
    motion_cache: RefCell<UnboundCache<usize, <W as Waypoint>::Motion>>,
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector2;
    use nalgebra::Isometry2;
    use num_complex::Complex64;
    use std::f64::consts::PI;

    #[test]
    fn test_nalgebra() {
        let p0 = Vector2::<f64>::new(0f64, 1f64);
        let p1 = Vector2::<f64>::new(10f64, 2f64);

        let delta_p = p1 - p0;
        assert_eq!(delta_p[0], 10f64);
        assert_eq!(delta_p[1], 1f64);

        let tf0 = Isometry2::<f64>::new(
            p0.clone(),
            PI/2.0
        );

        let tf1 = Isometry2::<f64>::new(
            p1.clone(),
            -PI/2.0
        );

        let delta_p = tf1.translation.vector - tf0.translation.vector;
        assert_eq!(delta_p[0], 10f64);
        assert_eq!(delta_p[1], 1f64);

        let result = tf0.lerp_slerp(&tf1, 0.5);
        // println!("Slerped Angle: {}", result.rotation.angle()*180.0/PI);
        // assert!(result.rotation.angle() < -135.0*PI/180.0);

        // let z0 = nalgebra::geometry::UnitComplex::<f64>::from_complex(
        //     Complex64::new(-1.0, 0.0)
        // );
        // let z1 = nalgebra::geometry::UnitComplex::<f64>::from_complex(
        //     Complex64::new(-1.0, 0.0)
        // );

        let z0 = nalgebra::geometry::UnitComplex::<f64>::new(0.0*PI/180.0);
        let z1 = nalgebra::geometry::UnitComplex::<f64>::new(180.0*PI/180.0);

        let result = z0.slerp(&z1, 0.5);
        println!("Slerped angle: {}", result.angle()*180.0/PI);
    }
}
