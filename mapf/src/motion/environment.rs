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
};
use std::{
    collections::{HashMap, hash_map::Entry},
    sync::Arc,
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

#[derive(Debug, Clone)]
pub struct DynamicEnvironmentOverlay<W: Waypoint> {
    pub profile: Option<CircularProfile>,
    pub obstacles: HashMap<usize, DynamicCircularObstacle<W>>,
}

impl<W: Waypoint> Default for DynamicEnvironmentOverlay<W> {
    fn default() -> Self {
        Self { profile: None, obstacles: HashMap::new() }
    }
}

#[derive(Debug, Clone)]
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
    /// This will return an `Err` if the base environment does not have an entry
    /// for the obstacle. Any obstacle overlays that do not exist in the base
    /// environment will not be iterated over when calling [`Environment::obstacles`].
    /// However, the overlaid data will be retained and will become applicable
    /// if the base environment is changed to one that does contain the obstacle
    /// entry.
    pub fn overlay_obstacle(
        &mut self,
        index: usize,
        obstacle: DynamicCircularObstacle<W>,
    ) -> Result<Option<DynamicCircularObstacle<W>>, Option<DynamicCircularObstacle<W>>> {
        let r = self.overlay.obstacles.insert(index, obstacle);
        if index < self.base.obstacles.len() {
            Ok(r)
        } else {
            Err(r)
        }
    }

    /// Revert an obstacle to its base version, removing it from the overlay.
    ///
    /// This will return an `Err` if the base environment does not have an entry
    /// for the obstacle, but you will still receive a copy of the obstacle that
    /// existed in the overlay if it was there.
    pub fn revert_obstacle(
        &mut self,
        index: usize,
    ) -> Result<Option<DynamicCircularObstacle<W>>, Option<DynamicCircularObstacle<W>>> {
        let r = self.overlay.obstacles.remove(&index);
        if index < self.base.obstacles.len() {
            Ok(r)
        } else {
            Err(r)
        }
    }

    /// Overlay a trajectory for the specified obstacle. If there was already an
    /// overlay for the obstacle, get back the trajectory that was there.
    ///
    /// Passing in [`None`] will make it appear that the obstacle has no presence
    /// in the world.
    ///
    /// If the base environment does not have a matching obstacle entry, this
    /// will return `Err`. If the overlay also does not have a matching obstacle
    /// entry then the input trajectory will be passed back in the `Err` and it
    /// will not be retained in the overlay at all. This is because we cannot
    /// infer a profile for the obstacle if it is not already present in either
    /// the base environment or in the overlay.
    pub fn overlay_trajectory(
        &mut self,
        index: usize,
        trajectory: Option<Trajectory<W>>,
    ) -> Result<Option<Option<Trajectory<W>>>, Option<Option<Trajectory<W>>>>
    where
        W: Into<WaypointR2>,
    {
        let prior_trajectory = match self.overlay.obstacles.entry(index) {
            Entry::Occupied(mut obstacle) => {
                let obstacle = obstacle.get_mut();
                let prior = obstacle.trajectory.take();
                obstacle.set_trajectory(trajectory);
                Some(prior)
            }
            Entry::Vacant(vacant) => {
                let base_obs = match self.base.obstacles.get(index) {
                    Some(base_obs) => base_obs,
                    None => return Err(Some(trajectory)),
                };
                let overlay_obs = DynamicCircularObstacle::new(base_obs.profile)
                    .with_trajectory(trajectory);
                vacant.insert(overlay_obs);
                None
            }
        };

        if index < self.base.obstacles.len() {
            Ok(prior_trajectory)
        } else {
            Err(prior_trajectory)
        }
    }

    /// Set the base environment to a specific shared environment.
    pub fn set_base(&mut self, base: Arc<DynamicEnvironment<W>>) {
        self.base = base;
    }

    /// Change something within the base environment. This will avoid cloning
    /// the base environment if it is not being shared, but will incur a cloning
    /// cost if it is being shared.
    pub fn modify_base<F>(mut self, f: F) -> Self
    where
        F: FnOnce(&mut DynamicEnvironment<W>),
    {
        let mut env = match Arc::try_unwrap(self.base) {
            Ok(base) => base,
            Err(arc_base) => (*arc_base).clone()
        };

        f(&mut env);
        self.base = Arc::new(env);
        self
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

impl<Env, Profile, Obstacle> Environment<Profile, Obstacle> for Arc<Env>
where
    Env: Environment<Profile, Obstacle>
{
    type Obstacles<'a> = Env::Obstacles<'a>
    where
        Env: 'a,
        Profile: 'a,
        Obstacle: 'a;

    fn agent_profile(&self) -> &Profile {
        self.as_ref().agent_profile()
    }

    fn obstacles<'a>(&'a self) -> Self::Obstacles<'a> {
        self.as_ref().obstacles()
    }
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

    pub fn footprint_radius(&self) -> f64 {
        self.footprint_radius
    }

    pub fn with_safety_distance(mut self, safety_distance: f64) -> Result<Self, ()> {
        if safety_distance < 0.0 {
            return Err(());
        }
        self.safety_buffer = safety_distance;
        Ok(self)
    }

    pub fn safety_buffer(&self) -> f64 {
        self.safety_buffer
    }

    pub fn with_follow_distance(mut self, follow_distance: f64) -> Result<Self, ()> {
        if follow_distance < 0.0 {
            return Err(());
        }
        self.follow_buffer = follow_distance;
        Ok(self)
    }

    pub fn follow_buffer(&self) -> f64 {
        self.follow_buffer
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
