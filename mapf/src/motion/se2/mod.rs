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

pub type Position = nalgebra::geometry::Isometry2<f64>;
pub type Point = nalgebra::geometry::Point2<f64>;
pub type Orientation = nalgebra::geometry::UnitComplex<f64>;
pub type Vector = nalgebra::Vector2<f64>;
pub type Rotation = nalgebra::UnitComplex<f64>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    pub translational: Vector,
    pub rotational: f64,
}

impl Velocity {
    pub fn zero() -> Self {
        Velocity {
            translational: Vector::zeros(),
            rotational: 0.0,
        }
    }
}

pub mod timed_position;

pub use timed_position::*;
pub mod space;
pub use space::*;

pub mod oriented;
pub use oriented::*;

pub type LinearTrajectorySE2 = super::Trajectory<WaypointSE2>;

#[cfg(feature = "serde")]
use serde::de::{Deserializer, Error, SeqAccess, Visitor};
#[cfg(feature = "serde")]
use serde::ser::{Serialize, SerializeSeq, Serializer};

#[cfg(feature = "serde")]
impl serde::Serialize for LinearTrajectorySE2 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_seq(Some(self.len()))?;
        for e in self.iter() {
            seq.serialize_element(&e)?;
        }
        seq.end()
    }
}

#[cfg(feature = "serde")]
struct SE2TrajectoryVisitor;

#[cfg(feature = "serde")]
impl<'de> Visitor<'de> for SE2TrajectoryVisitor {
    type Value = LinearTrajectorySE2;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        formatter.write_str("SE2 key value sequence.")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let start = if let Some(wp) = seq.next_element::<WaypointSE2>()? {
            Some(wp)
        } else {
            None
        };

        let second = if let Some(wp) = seq.next_element::<WaypointSE2>()? {
            Some(wp)
        } else {
            None
        };

        if start.is_some() && second.is_some() {
            if let Ok(mut traj) = Trajectory::new(start.unwrap(), second.unwrap()) {
                while let Some(wp) = seq.next_element::<WaypointSE2>()? {
                    traj.insert(wp);
                }
                return Ok(traj);
            }
        }
        return Err(A::Error::custom("Trajectory needs at least 2 points"));
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for LinearTrajectorySE2 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_seq(SE2TrajectoryVisitor)
    }
}

pub mod quickest_path;
pub use quickest_path::{QuickestPathHeuristic, QuickestPathPlanner};

pub mod differential_drive_line_follow;
pub use differential_drive_line_follow::*;

use super::Trajectory;
