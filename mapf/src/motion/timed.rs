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

pub trait Timed {
    fn time(&self) -> &TimePoint;
    fn set_time(&mut self, new_time: TimePoint);
}

#[derive(Clone, Debug)]
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

impl<W: Timed> std::cmp::Eq for TimeCmp<W> { }

impl<W: Timed> std::cmp::PartialOrd<TimePoint> for TimeCmp<W> {
    fn partial_cmp(&self, other: &TimePoint) -> Option<std::cmp::Ordering> {
        return self.0.time().nanos_since_zero.partial_cmp(
            &other.nanos_since_zero
        );
    }
}

impl<W: Timed> std::cmp::PartialOrd<Self> for TimeCmp<W> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return self.0.time().nanos_since_zero.partial_cmp(
            &other.0.time().nanos_since_zero
        );
    }
}

impl<W: Timed> std::cmp::Ord for TimeCmp<W> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.0.time().nanos_since_zero.cmp(
            &other.0.time().nanos_since_zero
        );
    }
}
