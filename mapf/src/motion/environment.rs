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

use crate::{
    motion::{
        Waypoint, Trajectory,
        r2::{WaypointR2, Point},
    },
    error::ThisError,
};
use std::{
    collections::HashMap,
    sync::{Arc, RwLock, RwLockReadGuard},
    ops::Deref,
};

type Vector2 = nalgebra::Vector2<f64>;

pub trait Environment<Profile, Obstacle> {
    type Obstacles<'a>: IntoIterator<Item=&'a Obstacle> + 'a
    where
        Self: 'a,
        Profile: 'a,
        Obstacle: 'a;

    fn agent_profile(&self) -> &Profile;

    fn obstacles<'a>(&'a self) -> Self::Obstacles<'a>;
}

pub trait WithEnvironment {
    type Environment;

    // TODO(@mxgrey): Is Deref really the best trait bound for this associated type?
    type EnvironmentReader<'a>: Deref<Target=Self::Environment> + 'a
    where
        Self: 'a,
        Self::Environment: 'a;
    type EnvironmentError;

    fn read_environment<'a>(
        &'a self
    ) -> Result<Self::EnvironmentReader<'a>, Self::EnvironmentError>;

    fn modify_environment<U, F: FnOnce(&mut Self::Environment) -> U>(
        &mut self,
        op: F
    ) -> Result<U, Self::EnvironmentError>;
}

#[derive(Debug, Clone)]
pub struct DynamicEnvironment<W: Waypoint> {
    pub profile: CircularProfile,
    pub obstacles: Vec<DynamicCircularObstacle<W>>,
}

impl<W: Waypoint> Environment<CircularProfile, DynamicCircularObstacle<W>> for DynamicEnvironment<W> {
    type Obstacles<'a> = impl Iterator<Item=&'a DynamicCircularObstacle<W>>
    where
        W: 'a;

    fn agent_profile(&self) -> &CircularProfile {
        &self.profile
    }

    fn obstacles<'a>(&'a self) -> Self::Obstacles<'a> {
        self.obstacles.iter()
    }
}

impl<W: Waypoint> DynamicEnvironment<W> {
    pub fn new(profile: CircularProfile) -> Self {
        Self { profile, obstacles: Vec::new() }
    }
}

pub struct DynamicEnvironmentOverlay<W: Waypoint> {
    pub profile: Option<CircularProfile>,
    pub obstacles: HashMap<usize, DynamicCircularObstacle<W>>,
}

impl<W: Waypoint> Default for DynamicEnvironmentOverlay<W> {
    fn default() -> Self {
        Self { profile: None, obstacles: HashMap::new() }
    }
}

pub struct OverlayedDynamicEnvironment<W: Waypoint> {
    base: Arc<DynamicEnvironment<W>>,
    overlay: DynamicEnvironmentOverlay<W>,
}

impl<W: Waypoint> OverlayedDynamicEnvironment<W> {
    pub fn new(base: Arc<DynamicEnvironment<W>>) -> Self {
        Self { base, overlay: Default::default() }
    }

    /// Overlay an obstacle. If there was already an overlay for this obstacle
    /// then get back the previous overlay.
    ///
    /// Passing in None will revert the obstacle to the base version. To remove
    /// the presence of an obstacle pass in `Some(obstacle.with_trajectory(None))`.
    ///
    /// This will return an `Err` if the base environment does not have an entry
    /// for the obstacle. Any obstacle overlays that do not exist in the base
    /// environment will not be iterated over when calling [`Environment::obstacles`].
    /// However, the overlaid data will be retained and will become applicable
    /// if the base environment is changed to one that does contain the obstacle
    /// entry.
    pub fn overlay_obstacle(
        &mut self,
        index: usize,
        obstacle: Option<DynamicCircularObstacle<W>>,
    ) -> Result<Option<DynamicCircularObstacle<W>>, Option<DynamicCircularObstacle<W>>> {
        let r = match obstacle {
            Some(obstacle) => self.overlay.obstacles.insert(index, obstacle),
            None => self.overlay.obstacles.remove(&index),
        };

        if index < self.base.obstacles.len() {
            Ok(r)
        } else {
            Err(r)
        }
    }

    pub fn change_base(&mut self, base: Arc<DynamicEnvironment<W>>) {
        self.base = base;
    }
}

impl<W: Waypoint> Environment<CircularProfile, DynamicCircularObstacle<W>> for OverlayedDynamicEnvironment<W> {
    type Obstacles<'a> = impl Iterator<Item=&'a DynamicCircularObstacle<W>>
    where
        W: 'a;

    fn agent_profile(&self) -> &CircularProfile {
        self.overlay.profile.as_ref().unwrap_or(&self.base.profile)
    }

    fn obstacles<'a>(&'a self) -> Self::Obstacles<'a> {
        self
        .base
        .obstacles
        .iter()
        .enumerate()
        .map(|(i, obs)| self.overlay.obstacles.get(&i).unwrap_or(obs))
    }
}

pub struct SharedEnvironment<Env> {
    // TODO(@mxgrey): It might be possible to implement this using RefCell
    // instead of RwLock if users are only allowed to modify the environment
    // using modify_environment since that function requires mutable access.
    inner: Arc<RwLock<Env>>,
}

impl<Env> SharedEnvironment<Env> {
    pub fn new(env: Env) -> Self {
        Self { inner: Arc::new(RwLock::new(env)) }
    }
}

impl<Env> Clone for SharedEnvironment<Env> {
    fn clone(&self) -> Self {
        Self { inner: self.inner.clone() }
    }
}

impl<Env> WithEnvironment for SharedEnvironment<Env> {
    type Environment = Env;
    type EnvironmentReader<'a> = RwLockReadGuard<'a, Env>
    where
        Env: 'a;
    type EnvironmentError = SharedEnvironmentError;

    fn read_environment<'a>(&'a self) -> Result<Self::EnvironmentReader<'a>, Self::EnvironmentError> {
        self.inner.read().map_err(|_| SharedEnvironmentError::PoisonedMutex)
    }

    fn modify_environment<U, F: FnOnce(&mut Env) -> U>(&mut self, op: F) -> Result<U, Self::EnvironmentError> {
        let mut guard = self.inner.write()
            .map_err(|_| SharedEnvironmentError::PoisonedMutex)?;
        Ok(op(&mut guard))
    }
}

#[derive(Debug, ThisError)]
pub enum SharedEnvironmentError {
    #[error("The mutex used by the shared environment has been poisoned")]
    PoisonedMutex,
}

#[derive(Debug, Clone, Copy)]
pub struct CircularProfile {
    /// Radius that encompasses the physical footprint of the robot
    footprint_radius: f64,
    /// Distance that the agent should try to keep its footprint away from the
    /// location where the footprint of an obstacle will collide with it. When
    /// two agents in contention have different `safety_distance` values, the
    /// larger value will be used by both.
    safety_buffer: f64,

    /// When an agent is following an obstacle or another agent (the dot
    /// product of their velocities is positive), the agent's movements should
    /// be broken into segments of this size
    follow_buffer: f64,
}

impl CircularProfile {

    pub fn new(
        footprint_radius: f64,
        safety_buffer: f64,
        follow_buffer: f64,
    ) -> Result<Self, ()> {
        if footprint_radius < 0.0 || safety_buffer < 0.0 || follow_buffer < 0.0 {
            return Err(());
        }

        Ok(Self { footprint_radius, safety_buffer, follow_buffer })
    }

    pub fn with_footprint_radius(mut self, footprint_radius: f64) -> Result<Self, ()> {
        if footprint_radius < 0.0 {
            return Err(());
        }
        self.footprint_radius = footprint_radius;
        Ok(self)
    }

    pub fn with_safety_distance(mut self, safety_distance: f64) -> Result<Self, ()> {
        if safety_distance < 0.0 {
            return Err(());
        }
        self.safety_buffer = safety_distance;
        Ok(self)
    }

    pub fn with_follow_distance(mut self, follow_distance: f64) -> Result<Self, ()> {
        if follow_distance < 0.0 {
            return Err(());
        }
        self.follow_buffer = follow_distance;
        Ok(self)
    }

    /// The critical distance is the distance between two traffic participants
    /// where they must not approach each other any closer. Use this value when
    /// calculating an acceptable stopping location for an agent.
    ///
    /// See also [`Profile::conflict_distance_for`]
    pub fn critical_distance_for(&self, other: &CircularProfile) -> f64 {
        let d = self.footprint_radius + other.footprint_radius;
        f64::max(d, 1e-3)
    }

    /// When two traffic participants are at or within this distance, then we
    /// will consider them to be in-conflict if they move any closer towards
    /// each other.
    ///
    /// See also [`Profile::critical_distance_for`]
    pub fn conflict_distance_for(&self, other: &CircularProfile) -> f64 {
        let d = self.footprint_radius + other.footprint_radius;
        f64::max(d - 1e-3, 0.0)
    }

    pub fn critical_distance_squared_for(&self, other: &CircularProfile) -> f64 {
        self.critical_distance_for(other).powi(2)
    }

    pub fn conflict_distance_squared_for(&self, other: &CircularProfile) -> f64 {
        self.conflict_distance_for(other).powi(2)
    }

    pub fn safety_distance_for(&self, other: &CircularProfile) -> f64 {
        let d = self.critical_distance_for(other);
        f64::max(self.safety_buffer, other.safety_buffer) + d
    }

    pub fn follow_distance_for(&self, other: &CircularProfile) -> f64 {
        let d = self.critical_distance_for(&other);
        f64::max(f64::max(self.follow_buffer, other.follow_buffer), d) + d
    }
}

#[derive(Debug, Clone)]
pub struct DynamicCircularObstacle<W: Waypoint> {
    profile: CircularProfile,
    trajectory: Option<Trajectory<W>>,
    bounding_box: Option<BoundingBox>,
}

impl<W: Waypoint + Into<WaypointR2>> DynamicCircularObstacle<W> {
    pub fn new(profile: CircularProfile) -> Self {
        Self {
            profile,
            trajectory: None,
            bounding_box: None,
        }
    }

    pub fn with_trajectory(self, trajectory: Option<Trajectory<W>>) -> Self {
        Self {
            bounding_box: trajectory.as_ref().map(
                |t| BoundingBox::for_trajectory(&self.profile, t)
            ),
            profile: self.profile,
            trajectory,
        }
    }

    pub fn profile(&self) -> &CircularProfile {
        &self.profile
    }

    pub fn set_profile(&mut self, profile: CircularProfile) {
        self.profile = profile;
        self.bounding_box = self.trajectory.as_ref().map(
            |t| BoundingBox::for_trajectory(&self.profile, t)
        );
    }

    pub fn trajectory(&self) -> Option<&Trajectory<W>> {
        self.trajectory.as_ref()
    }

    pub fn set_trajectory(&mut self, trajectory: Option<Trajectory<W>>) {
        self.trajectory = trajectory;
        self.bounding_box = self.trajectory.as_ref().map(
            |t| BoundingBox::for_trajectory(&self.profile, t)
        );
    }

    pub fn bounding_box(&self) -> Option<&BoundingBox> {
        self.bounding_box.as_ref()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    min: Vector2,
    max: Vector2,
}

impl BoundingBox {
    pub fn overlaps(&self, other: Option<BoundingBox>) -> bool {
        let other = match other {
            Some(b) => b,
            None => return false,
        };
        if other.max.x < self.min.x {
            return false;
        }
        if other.max.y < self.min.y {
            return false;
        }
        if self.max.x < other.min.x {
            return false;
        }
        if self.max.y < other.min.y {
            return false;
        }
        return true;
    }

    pub fn for_point(p: Point) -> Self {
        Self {
            min: p.coords,
            max: p.coords,
        }
    }

    pub fn for_line(profile: &CircularProfile, wp0: &WaypointR2, wp1: &WaypointR2) -> Self {
        Self::for_point(wp0.position)
        .incorporating(wp1.position)
        .inflated_by(profile.footprint_radius)
    }

    pub fn for_trajectory<W>(profile: &CircularProfile, trajectory: &Trajectory<W>) -> Self
    where
        W: Into<WaypointR2> + Waypoint,
    {
        let initial_bb = BoundingBox::for_point(
            trajectory.initial_motion().clone().into().position
        );

        trajectory
        .iter()
        .fold(initial_bb, |b: Self, p| b.incorporating(p.into().position))
        .inflated_by(profile.footprint_radius)
    }

    pub fn incorporating(self, p: Point) -> Self {
        Self {
            min: Vector2::new(
                f64::min(self.min.x, p.x),
                f64::min(self.min.y, p.y),
            ),
            max: Vector2::new(
                f64::max(self.max.x, p.x),
                f64::max(self.max.y, p.y),
            ),
        }
    }

    pub fn inflated_by(self, r: f64) -> Self {
        Self {
            min: self.min - Vector2::from_element(r),
            max: self.max + Vector2::from_element(r),
        }
    }
}
