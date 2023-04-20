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

use time_point::{Duration, TimePoint};

pub trait Timed: MaybeTimed {
    fn time(&self) -> TimePoint;
    fn set_time(&mut self, new_time: TimePoint);

    fn with_time(mut self, new_time: TimePoint) -> Self
    where
        Self: Sized,
    {
        self.set_time(new_time);
        self
    }

    fn time_shifted_by(mut self, delta_t: Duration) -> Self
    where
        Self: Sized,
    {
        self.set_time(self.time() + delta_t);
        self
    }
}

pub trait MaybeTimed {
    fn maybe_time(&self) -> Option<TimePoint>;
}

impl MaybeTimed for usize {
    fn maybe_time(&self) -> Option<TimePoint> {
        None
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug)]
pub struct TimeCmp<W: Timed>(pub W);

impl<W: Timed> std::cmp::PartialEq<TimePoint> for TimeCmp<W> {
    fn eq(&self, other: &TimePoint) -> bool {
        return self.0.time().nanos_since_zero == other.nanos_since_zero;
    }

    fn ne(&self, other: &TimePoint) -> bool {
        return self.0.time().nanos_since_zero != other.nanos_since_zero;
    }
}

impl<W: Timed> std::cmp::PartialEq<Self> for TimeCmp<W> {
    fn eq(&self, other: &Self) -> bool {
        return self.0.time().nanos_since_zero == other.0.time().nanos_since_zero;
    }

    fn ne(&self, other: &Self) -> bool {
        return self.0.time().nanos_since_zero != other.0.time().nanos_since_zero;
    }
}

impl<W: Timed> std::cmp::Eq for TimeCmp<W> {}

impl<W: Timed> std::cmp::PartialOrd<TimePoint> for TimeCmp<W> {
    fn partial_cmp(&self, other: &TimePoint) -> Option<std::cmp::Ordering> {
        return self
            .0
            .time()
            .nanos_since_zero
            .partial_cmp(&other.nanos_since_zero);
    }
}

impl<W: Timed> std::cmp::PartialOrd<Self> for TimeCmp<W> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return self
            .0
            .time()
            .nanos_since_zero
            .partial_cmp(&other.0.time().nanos_since_zero);
    }
}

impl<W: Timed> std::cmp::Ord for TimeCmp<W> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self
            .0
            .time()
            .nanos_since_zero
            .cmp(&other.0.time().nanos_since_zero);
    }
}

impl<W: Timed> std::ops::Deref for TimeCmp<W> {
    type Target = W;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
