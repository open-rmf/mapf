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

use super::{timed::TimeCmp, Duration, InterpError, Motion, TimePoint, Waypoint};
use cached::{Cached, UnboundCache};
use sorted_vec::{FindOrInsert, SortedSet};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FindWaypoint {
    /// The requested time is exactly on the waypoint at this index
    Exact(usize),

    /// The requested time is approaching the waypoint of this index. This is
    /// always greater than 0. If the time point was less than the initial time
    /// of the trajectory then BeforeStart will be returned instead.
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
    indefinite_initial_time: bool,
    indefinite_finish_time: bool,
}

impl<W: Waypoint> Trajectory<W> {
    /// Create a new trajectory, starting with the given endpoints. If the
    /// endpoints have the same time value then this will return an Err.
    pub fn new(start: W, finish: W) -> Result<Self, ()> {
        if start.time() == finish.time() {
            return Result::Err(());
        }

        let mut result = Self {
            waypoints: SortedSet::new(),
            indefinite_initial_time: false,
            indefinite_finish_time: false,
        };

        result.waypoints.push(TimeCmp(start));
        result.waypoints.push(TimeCmp(finish));
        return Result::Ok(result);
    }

    pub fn with_indefinite_initial_time(mut self, value: bool) -> Self {
        self.indefinite_initial_time = value;
        self
    }

    pub fn has_indefinite_initial_time(&self) -> bool {
        self.indefinite_initial_time
    }

    pub fn with_indefinite_finish_time(mut self, value: bool) -> Self {
        self.indefinite_finish_time = value;
        self
    }

    pub fn has_indefinite_finish_time(&self) -> bool {
        self.indefinite_finish_time
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
            indefinite_initial_time: false,
            indefinite_finish_time: false,
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
    pub fn find(&self, time: &TimePoint) -> FindWaypoint {
        match self
            .waypoints
            .binary_search_by(|x| x.partial_cmp(time).unwrap())
        {
            Result::Ok(index) => return FindWaypoint::Exact(index),
            Result::Err(index) => {
                if index == 0 {
                    return FindWaypoint::BeforeStart;
                } else if index == self.waypoints.len() {
                    return FindWaypoint::AfterFinish;
                } else {
                    return FindWaypoint::Approaching(index);
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

    /// Get the time duration of the trajectory. If the start or finish are
    pub fn duration(&self) -> Option<Duration> {
        if let (Some(initial), Some(finish)) = (self.initial_time(), self.finish_time()) {
            Some(finish - initial)
        } else {
            None
        }
    }

    pub fn motion_duration(&self) -> Duration {
        *self.finish_motion().time() - *self.initial_motion().time()
    }

    /// Trajectories always have at least two values, so we can always get the
    /// first waypoint.
    pub fn initial_motion(&self) -> &W {
        &self.waypoints.first().unwrap().0
    }

    pub fn finish_motion(&self) -> &W {
        &self.waypoints.last().unwrap().0
    }

    /// Get the time that the trajectory starts.
    pub fn initial_time(&self) -> Option<TimePoint> {
        if self.indefinite_initial_time {
            None
        } else {
            Some(self.initial_motion_time())
        }
    }

    pub fn initial_motion_time(&self) -> TimePoint {
        *self.initial_motion().time()
    }

    /// Get the time that the trajectory finishes.
    pub fn finish_time(&self) -> Option<TimePoint> {
        if self.indefinite_finish_time {
            None
        } else {
            Some(self.finish_motion_time())
        }
    }

    pub fn finish_motion_time(&self) -> TimePoint {
        *self.finish_motion().time()
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
    pub fn motion<'a>(&'a self) -> TrajectoryMotion<'a, W> {
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
    pub fn iter(&self) -> TrajectoryIter<'_, W> {
        TrajectoryIter::new(self, self.initial_motion_time(), None)
    }

    /// Iterate through this trajectory, starting at the requested time.
    ///
    /// If the requested time is in between two waypoints, the iterator will
    /// begin with the waypoint that comes immediately before the requested
    /// time.
    ///
    /// If the requested time is before the time of the trajectory's initial
    /// waypoint and the trajectory has an indefinite initial time, then the
    /// iterator will begin with a waypoint that is a clone of the initial
    /// waypoint but with the input `time` as its time value.
    ///
    /// If the trajectory has a definite initial time which comes after the
    /// requested time, then the iterator will begin with the first waypoint
    /// in the trajectory.
    pub fn iter_from(&self, time: TimePoint) -> TrajectoryIter<'_, W> {
        TrajectoryIter::new(self, time, None)
    }

    pub fn iter_range(&self, from_time: TimePoint, to_time: TimePoint) -> TrajectoryIter<'_, W> {
        TrajectoryIter::new(self, from_time, Some(to_time))
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
    // TODO(@mxgrey): We could probably use references with explicit lifetimes
    // instead of Rc.
    motion_cache: RefCell<UnboundCache<usize, Rc<W::Motion>>>,
}

impl<'a, W: Waypoint> TrajectoryMotion<'a, W> {
    fn find_motion_segment(&self, time: &TimePoint) -> Result<MotionSegment<W>, InterpError> {
        match self.trajectory.find(time) {
            FindWaypoint::Exact(index) => {
                if index == 0 {
                    return Ok(MotionSegment::Interp(self.get_motion_segment(index + 1)));
                } else {
                    return Ok(MotionSegment::Interp(self.get_motion_segment(index)));
                }
            }
            FindWaypoint::Approaching(index) => {
                return Ok(MotionSegment::Interp(self.get_motion_segment(index)));
            }
            FindWaypoint::BeforeStart => {
                if self.trajectory.has_indefinite_initial_time() {
                    return Ok(MotionSegment::Holding(self.trajectory.initial_motion().position()));
                }
            }
            FindWaypoint::AfterFinish => {
                if self.trajectory.has_indefinite_finish_time() {
                    return Ok(MotionSegment::Holding(self.trajectory.finish_motion().position()));
                }
            }
        }

        Err(InterpError::OutOfBounds {
            range: [
                self.trajectory.initial_motion_time(),
                self.trajectory.finish_motion_time(),
            ],
            request: *time,
        })
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

enum MotionSegment<W: Waypoint> {
    Interp(Rc<W::Motion>),
    Holding(W::Position),
}

impl<'a, W: Waypoint> Motion<W::Position, W::Velocity> for TrajectoryMotion<'a, W> {
    fn compute_position(&self, time: &TimePoint) -> Result<W::Position, InterpError> {
        match self.find_motion_segment(time)? {
            MotionSegment::Interp(motion) => motion.compute_position(time),
            MotionSegment::Holding(p) => Ok(p),
        }
    }

    fn compute_velocity(&self, time: &TimePoint) -> Result<W::Velocity, InterpError> {
        match self.find_motion_segment(time)? {
            MotionSegment::Interp(motion) => motion.compute_velocity(time),
            MotionSegment::Holding(_) => Ok(W::zero_velocity()),
        }
    }
}

pub struct TrajectoryIter<'a, W: Waypoint> {
    trajectory: &'a Trajectory<W>,
    next_element: TrajectoryIterNext,
    begin: TimePoint,
    until: Option<TimePoint>,
}

impl<'a, W: Waypoint> TrajectoryIter<'a, W> {
    pub fn pairs(self) -> TrajectoryIterPairs<'a, W> {
        TrajectoryIterPairs {
            base: self,
            previous: None,
        }
    }

    fn new(trajectory: &'a Trajectory<W>, begin: TimePoint, until: Option<TimePoint>) -> Self {
        let next_element = match trajectory.find(&begin) {
            FindWaypoint::BeforeStart => {
                if trajectory.indefinite_initial_time {
                    TrajectoryIterNext::PreInitial(begin)
                } else {
                    TrajectoryIterNext::Index(0)
                }
            }
            FindWaypoint::Exact(index) => TrajectoryIterNext::Index(index),
            FindWaypoint::Approaching(index) => TrajectoryIterNext::Index(index - 1),
            FindWaypoint::AfterFinish => TrajectoryIterNext::StartPostFinish(begin),
        };

        Self {
            trajectory,
            next_element,
            begin,
            until,
        }
    }
}

enum TrajectoryIterNext {
    PreInitial(TimePoint),
    Index(usize),
    StartPostFinish(TimePoint),
    ReachedPostFinish(TimePoint),
    Depleted,
}

impl<'a, W: Waypoint> Iterator for TrajectoryIter<'a, W> {
    type Item = W;

    fn next(&mut self) -> Option<W> {
        match self.next_element {
            TrajectoryIterNext::PreInitial(t) => {
                if let Some(t_f) = self.until {
                    if t_f < t {
                        self.next_element = TrajectoryIterNext::Depleted;
                        return None;
                    }
                }

                let mut wp = self.trajectory.initial_motion().clone();
                wp.set_time(t);
                self.next_element = TrajectoryIterNext::Index(0);
                Some(wp)
            }
            TrajectoryIterNext::Index(index) => {
                let wp = self.trajectory.get(index).map(|wp| wp.clone());
                if let (Some(t_f), Some(wp)) = (self.until, &wp) {
                    if t_f <= *wp.time() {
                        // We only include the first element that exceeds the
                        // finish time.
                        self.next_element = TrajectoryIterNext::Depleted;
                        if *wp.time() < self.begin {
                            // This element comes before the begin time.
                            // This is not supposed to happen, but let's handle
                            // it gracefully anyway.
                            return None;
                        }
                        return Some(wp.clone());
                    }
                }

                self.next_element = if index + 1 < self.trajectory.len() {
                    TrajectoryIterNext::Index(index + 1)
                } else {
                    if let Some(t_f) = self.until {
                        TrajectoryIterNext::ReachedPostFinish(t_f)
                    } else {
                        TrajectoryIterNext::Depleted
                    }
                };

                wp
            }
            TrajectoryIterNext::StartPostFinish(t) => {
                if !self.trajectory.has_indefinite_finish_time() {
                    self.next_element = TrajectoryIterNext::Depleted;
                    return None;
                }

                let mut wp = self.trajectory.finish_motion().clone();
                wp.set_time(t);
                if let Some(t_f) = self.until {
                    self.next_element = TrajectoryIterNext::ReachedPostFinish(t_f);
                } else {
                    self.next_element = TrajectoryIterNext::Depleted;
                }
                return Some(wp);
            }
            TrajectoryIterNext::ReachedPostFinish(t) => {
                if !self.trajectory.has_indefinite_finish_time() {
                    self.next_element = TrajectoryIterNext::Depleted;
                    return None;
                }

                let mut wp = self.trajectory.finish_motion().clone();
                wp.set_time(t);
                self.next_element = TrajectoryIterNext::Depleted;
                return Some(wp);
            }
            TrajectoryIterNext::Depleted => None,
        }
    }
}

// TODO(@mxgrey): Consider how to make this more general so that it can work on
// N-sized windows. E.g. use a circular buffer array of a fixed size to store
// N previous waypoints.
pub struct TrajectoryIterPairs<'a, W: Waypoint> {
    base: TrajectoryIter<'a, W>,
    previous: Option<W>,
}

impl<'a, W: Waypoint> Iterator for TrajectoryIterPairs<'a, W> {
    type Item = [W; 2];

    fn next(&mut self) -> Option<[W; 2]> {
        let previous = match &self.previous {
            Some(wp) => wp.clone(),
            None => match self.base.next() {
                Some(wp) => wp,
                None => return None,
            },
        };

        match self.base.next() {
            Some(wp) => {
                self.previous = Some(wp.clone());
                Some([previous, wp])
            }
            None => None,
        }
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
