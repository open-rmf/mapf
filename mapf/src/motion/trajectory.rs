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

use super::{
    timed::TimeCmp,
    Duration, InterpError, Motion, TimePoint, Waypoint,
};
use cached::{Cached, UnboundCache};
use sorted_vec::{FindOrInsert, SortedSet};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq, Eq)]
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

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RemovalError {
    /// The specified index or range is outside the index bounds of the
    /// trajectory.
    OutOfBounds,

    /// The requested removal would leave the trajectory with fewer than 2
    /// waypoints.
    Depleting,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MutateError {
    /// The requested index is out of the bounds of the trajectory.
    OutOfBounds,

    /// The time of the waypoint was shifted too high or too low, which would
    /// cause its order within the trajectory to change. Its time value has been
    /// changed back to its original value.
    InvalidTimeChange,
}

#[derive(Clone, PartialEq, Eq)]
pub struct Trajectory<W: Waypoint> {
    waypoints: SortedSet<TimeCmp<W>>,
}

impl<'a, W: Waypoint> Trajectory<W> {
    /// Create a new trajectory, starting with the given endpoints. If the
    /// endpoints have the same time value then this will return an Err.
    pub fn new(start: W, finish: W) -> Result<Self, ()> {
        if start.time() == finish.time() {
            return Result::Err(());
        }

        let mut result = Self {
            waypoints: SortedSet::new(),
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
    pub fn from_iter<I: std::iter::IntoIterator<Item = W>>(iter: I) -> Result<Self, ()> {
        let mut result = Self {
            waypoints: SortedSet::new(),
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
            FindOrInsert::Inserted(index) => Result::Ok(index),
        }
    }

    /// Inserts the waypoint into this trajectory, or if the trajectory already
    /// has a waypoint at an equivalent time, then the value of this waypoint
    /// gets assigned to the old one.
    pub fn insert_or_assign(&mut self, waypoint: W) -> usize {
        return self.waypoints.replace(TimeCmp(waypoint)).0;
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
        match self
            .waypoints
            .binary_search_by(|x| x.partial_cmp(time).unwrap())
        {
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

    /// Get the waypoint at the requested index if it is available, otherwise
    /// get None.
    pub fn get(&self, index: usize) -> Option<&W> {
        // TODO(@mxgrey): Investigate how SliceIndex can be used here. The TimeCmp
        // wrapper complicates this.
        return self.waypoints.get(index).map(|x| &x.0);
    }

    /// Get the time duration of the trajectory.
    pub fn duration(&self) -> Duration {
        return self.finish_time() - self.initial_time();
    }

    /// Trajectories always have at least two values, so we can always get the
    /// first waypoint.
    pub fn initial(&self) -> &W {
        &self.waypoints.first().unwrap().0
    }

    pub fn finish(&self) -> &W {
        &self.waypoints.last().unwrap().0
    }

    /// Get the time that the trajectory starts.
    pub fn initial_time(&self) -> TimePoint {
        *self.initial().time()
    }

    /// Get the time that the trajectory finishes.
    pub fn finish_time(&self) -> TimePoint {
        *self.finish().time()
    }

    /// Make changes to the waypoint at a specified index. If a change is made
    /// to the waypoint's time that would cause its order within the vector to
    /// change, then its time value will be reverted back to the original and
    /// an error will be returned.
    pub fn mutate_waypoint<F: FnOnce(&mut W)>(
        &mut self,
        index: usize,
        f: F,
    ) -> Result<(), MutateError> {
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

    /// Unsafe access to a waypoint in the trajectory
    pub unsafe fn get_unchecked(&self, index: usize) -> &W {
        &self.waypoints.get_unchecked(index).0
    }

    /// Get the number of Waypoint elements in the trajectory.
    pub fn len(&self) -> usize {
        return self.waypoints.len();
    }

    /// Reserve space in the trajectory for `additional` more Waypoints.
    /// This will not change the length of the trajectory, but it can improve
    /// performance by preventing redundant memory allocations if the trajectory
    /// needs to grow much further.
    pub fn reserve(&mut self, additional: usize) {
        self.waypoints.reserve(additional);
    }

    /// Returns the number of Waypoints that the trajectory can hold without
    /// reallocating.
    pub fn capacity(&self) -> usize {
        return self.waypoints.capacity();
    }

    /// Get a motion for this trajectory
    pub fn motion(&'a self) -> TrajectoryMotion<'a, W> {
        return TrajectoryMotion {
            trajectory: self,
            motion_cache: RefCell::new(UnboundCache::new()),
        };
    }

    /// Iterate through this trajectory.
    ///
    /// Note: We return a slice whose item is wrapped in the TimeCmp newtype.
    /// This makes getting the actual waypoint out of the iterator slightly less
    /// convenient because you will need to dereference it, but it allows us to
    /// provide all the functionality of a slice without any custom
    /// implementations and without any unsafe blocks.
    pub fn iter(&self) -> std::slice::Iter<'_, TimeCmp<W>> {
        self.waypoints.iter()
    }
}

impl<W: Waypoint> std::fmt::Debug for Trajectory<W> {
    fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
        let mut builder = fmt.debug_list();
        for wp in self.waypoints.iter() {
            builder.entry(&wp.0);
        }

        return builder.finish();
    }
}

impl<W: Waypoint> std::ops::Deref for Trajectory<W> {
    type Target = [TimeCmp<W>];

    fn deref(&self) -> &Self::Target {
        &self.waypoints
    }
}

pub struct TrajectoryMotion<'a, W: Waypoint> {
    trajectory: &'a Trajectory<W>,
    motion_cache: RefCell<UnboundCache<usize, Rc<W::Motion>>>,
}

impl<'a, W: Waypoint> TrajectoryMotion<'a, W> {
    fn find_motion_segment(&self, time: &TimePoint) -> Result<Rc<W::Motion>, InterpError> {
        match self.trajectory.find(time) {
            Find::Exact(index) => {
                if index == 0 {
                    return Ok(self.get_motion_segment(index + 1));
                } else {
                    return Ok(self.get_motion_segment(index));
                }
            }
            Find::Approaching(index) => {
                return Ok(self.get_motion_segment(index));
            }
            Find::BeforeStart | Find::AfterFinish => {
                return Err(InterpError::OutOfBounds);
            }
        }
    }

    fn get_motion_segment(&self, index: usize) -> Rc<W::Motion> {
        return self
            .motion_cache
            .borrow_mut()
            .cache_get_or_set_with(index, || {
                // SAFETY: This should only be called by find_motion_segment
                // which should only call this function with an index greater
                // than zero and less than the length of the underlying vec.
                unsafe {
                    let wp0 = self.trajectory.get_unchecked(index - 1);
                    let wp1 = self.trajectory.get_unchecked(index);
                    return Rc::new(wp0.interpolate(wp1));
                }
            })
            .clone();
    }
}

impl<'a, W: Waypoint> Motion<W::Position, W::Velocity> for TrajectoryMotion<'a, W> {
    fn compute_position(&self, time: &TimePoint) -> Result<W::Position, InterpError> {
        return self.find_motion_segment(time)?.compute_position(time);
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<W::Velocity, InterpError> {
        return self.find_motion_segment(time)?.compute_velocity(time);
    }
}

pub trait CostCalculator<W: Waypoint>: std::fmt::Debug {
    type Cost;

    fn compute_cost(&self, trajectory: &Trajectory<W>) -> Self::Cost;
}

#[derive(Debug)]
pub struct DurationCostCalculator;
impl<W: Waypoint> CostCalculator<W> for DurationCostCalculator {
    type Cost = i64;
    fn compute_cost(&self, trajectory: &Trajectory<W>) -> Self::Cost {
        trajectory.duration().nanos
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use crate::motion::se2;
//     use crate::motion::se2::timed_position::Waypoint as WaypointSE2;
//     use crate::motion::Motion;
//     use approx::assert_relative_eq;

//     #[test]
//     fn test_valid_motion() {
//         let t0 = time_point::TimePoint::new(0);
//         let mut trajectory = se2::LinearTrajectory::new(
//             WaypointSE2::new(t0, 0.0, 0.0, 0.0),
//             WaypointSE2::new(
//                 t0 + time_point::Duration::from_secs(2),
//                 1.0,
//                 0.0,
//                 90f64.to_radians(),
//             ),
//         )
//         .expect("Trajectory failed to be created");

//         let insertion = trajectory.insert(WaypointSE2::new(
//             t0 + time_point::Duration::from_secs(3),
//             1.0,
//             1.0,
//             180f64.to_radians(),
//         ));
//         assert_eq!(insertion.ok(), Some(2));

//         let insertion = trajectory.insert(WaypointSE2::new(
//             t0 + time_point::Duration::from_secs(1),
//             0.0,
//             1.0,
//             -45f64.to_radians(),
//         ));
//         assert_eq!(insertion.ok(), Some(1));

//         assert_eq!(trajectory.len(), 4);

//         let motion = trajectory.motion();
//         let p = motion
//             .compute_position(&(t0 + time_point::Duration::from_secs_f64(0.5)))
//             .expect("Failed to calculate position");
//         assert_relative_eq!(p.translation.vector[0], 0.0);
//         assert_relative_eq!(p.translation.vector[1], 0.5);
//         assert_relative_eq!(p.rotation.angle(), (-45f64 / 2.0).to_radians());

//         let v = motion
//             .compute_velocity(&(t0 + time_point::Duration::from_secs_f64(0.6788612)))
//             .expect("Failed to calculate velocity");
//         assert_relative_eq!(v.translational[0], 0.0);
//         assert_relative_eq!(v.translational[1], 1.0);
//         assert_relative_eq!(v.rotational, -45f64.to_radians());

//         let p = motion
//             .compute_position(&(t0 + time_point::Duration::from_secs(3)))
//             .expect("Failed to calculate position");
//         assert_relative_eq!(p.translation.vector[0], 1.0);
//         assert_relative_eq!(p.translation.vector[1], 1.0);
//         assert_relative_eq!(p.rotation.angle(), 180f64.to_radians());

//         let v = motion
//             .compute_velocity(&(t0 + time_point::Duration::from_secs(3)))
//             .expect("Failed to compute velocity");
//         assert_relative_eq!(v.translational[0], 0.0);
//         assert_relative_eq!(v.translational[1], 1.0);
//         assert_relative_eq!(v.rotational, 90f64.to_radians());

//         let p = motion
//             .compute_position(&t0)
//             .expect("Failed to compute velocity");
//         assert_relative_eq!(p.translation.vector[0], 0.0);
//         assert_relative_eq!(p.translation.vector[1], 0.0);
//         assert_relative_eq!(p.rotation.angle(), 0.0);

//         let v = motion
//             .compute_velocity(&t0)
//             .expect("Failed to calculate velocity");
//         assert_relative_eq!(v.translational[0], 0.0);
//         assert_relative_eq!(v.translational[1], 1.0);
//         assert_relative_eq!(v.rotational, -45f64.to_radians());

//         let err = motion.compute_position(&(t0 - time_point::Duration::new(1)));
//         assert_eq!(err, Err(InterpError::OutOfBounds));

//         let err = motion.compute_velocity(&(t0 - time_point::Duration::new(1)));
//         assert_eq!(err, Err(InterpError::OutOfBounds));
//     }
// }
