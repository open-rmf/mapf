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

use crate::{
    domain::{Backtrack, Domain},
    error::ThisError,
    motion::{Duration, IntegrateWaypoints, TimePoint, Timed, Trajectory, Waypoint},
};
use smallvec::SmallVec;

#[derive(Debug, Clone)]
pub struct Path<State, Action, Cost> {
    pub initial_state: State,
    pub sequence: Vec<(Action, State)>,
    pub total_cost: Cost,
}

impl<S, A, C> Path<S, A, C> {
    pub fn backtrack<ReverseDomain>(
        self,
        reverse_domain: &ReverseDomain,
    ) -> Result<Path<S, A, C>, ReverseDomain::Error>
    where
        ReverseDomain: Domain + Backtrack<S, A>,
        ReverseDomain::BacktrackError: Into<ReverseDomain::Error>,
    {
        let (_, mut parent_forward_state) = reverse_domain
            .flip_endpoints(
                &self.initial_state,
                self.sequence
                    .last()
                    .map(|s| &s.1)
                    .unwrap_or(&self.initial_state),
            )
            .map_err(Into::into)?;
        let mut parent_reverse_state = self.initial_state;

        let mut sequence = Vec::new();
        for (reverse_action, child_reverse_state) in self.sequence {
            let (forward_action, child_forward_state) = reverse_domain
                .backtrack(
                    &parent_forward_state,
                    &parent_reverse_state,
                    &reverse_action,
                    &child_reverse_state,
                )
                .map_err(Into::into)?;

            sequence.push((forward_action, parent_forward_state));
            parent_forward_state = child_forward_state;
            parent_reverse_state = child_reverse_state;
        }

        sequence.reverse();

        Ok(Path {
            initial_state: parent_forward_state,
            sequence,
            total_cost: self.total_cost,
        })
    }

    pub fn final_state(&self) -> &S {
        self.sequence
            .last()
            .map(|(_, s)| s)
            .unwrap_or(&self.initial_state)
    }

    /// Make a trajectory out of this path if possible. It might not be possible
    /// to make a trajectory if the path does not involve any motion.
    ///
    /// To always get a trajectory out of the path (provided there are no errors),
    /// then use [`make_trajectory_or_hold`].
    pub fn make_trajectory<W: Waypoint>(
        &self,
    ) -> Result<
        Option<MetaTrajectory<W, S>>,
        WaypointIntegrationError<S::WaypointIntegrationError, A::WaypointIntegrationError>,
    >
    where
        S: IntegrateWaypoints<W> + Clone + std::fmt::Debug + Timed,
        A: IntegrateWaypoints<W> + std::fmt::Debug,
    {
        let mut decision_points = Vec::new();
        decision_points.push(DecisionPoint {
            index: 0,
            state: self.initial_state.clone(),
        });
        let mut waypoints = self
            .initial_state
            .integrated_waypoints(None)
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .map_err(WaypointIntegrationError::State)?;

        let mut initial_wp = waypoints.last().cloned();
        for (action, state) in self.sequence.iter() {
            let wp_count = waypoints.len();
            waypoints.extend(
                action
                    .integrated_waypoints(initial_wp)
                    .into_iter()
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(WaypointIntegrationError::Action)?,
            );

            waypoints.sort_by_key(|w| w.time());
            waypoints.dedup_by_key(|w| w.time());

            initial_wp = waypoints.last().cloned();
            if wp_count < waypoints.len() {
                // dbg!((waypoints.len(), action, state));
                assert_eq!(waypoints.last().unwrap().time(), state.time());
                decision_points.push(DecisionPoint {
                    index: waypoints.len() - 1,
                    state: state.clone(),
                });
            }
        }

        Ok(Trajectory::from_iter(waypoints)
            .ok()
            .map(|trajectory| MetaTrajectory {
                trajectory,
                decision_points,
                initial_state: self.initial_state.clone(),
                final_state: self.final_state().clone(),
            }))
    }

    /// Make a trajectory out of this path if possible. If producing a path is
    /// not possible because no motion is necessary, then create a trajectory
    /// that holds the agent in place at the start location for the given
    /// `hold_duration`.
    ///
    /// If an initial waypoint cannot be drawn out of the initial state, then
    /// this will return an error instead of not returning a trajectory.
    ///
    /// If `hold_duration` is zero, 1ns will be used instead.
    pub fn make_trajectory_or_hold<W: Waypoint>(
        &self,
        hold_duration: Duration,
    ) -> Result<
        MetaTrajectory<W, S>,
        WaypointIntegrationError<S::WaypointIntegrationError, A::WaypointIntegrationError>,
    >
    where
        S: IntegrateWaypoints<W> + Clone + std::fmt::Debug + Timed,
        A: IntegrateWaypoints<W> + std::fmt::Debug,
    {
        if let Some(mt) = self.make_trajectory()? {
            return Ok(mt);
        }

        let wp0: W = match self
            .initial_state
            .integrated_waypoints(None)
            .into_iter()
            .last()
        {
            Some(r) => r.map_err(WaypointIntegrationError::State)?,
            None => return Err(WaypointIntegrationError::EmptyInitialState),
        };

        let hold_duration = if hold_duration.nanos == 0 {
            Duration::new(1)
        } else {
            hold_duration
        };

        Ok(MetaTrajectory {
            trajectory: Trajectory::new(wp0.clone(), wp0.time_shifted_by(hold_duration)).unwrap(),
            decision_points: Vec::from_iter([DecisionPoint {
                index: 0,
                state: self.initial_state.clone(),
            }]),
            initial_state: self.initial_state.clone(),
            final_state: self.final_state().clone(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct MetaTrajectory<W: Waypoint, State> {
    pub trajectory: Trajectory<W>,
    pub decision_points: Vec<DecisionPoint<State>>,
    pub initial_state: State,
    pub final_state: State,
}

#[derive(Debug, Clone, Copy)]
pub struct DecisionPoint<State> {
    pub index: usize,
    pub state: State,
}

#[derive(Debug, Clone, Copy)]
pub enum DecisionRange<S> {
    Between([DecisionPoint<S>; 2]),
    Before(S, TimePoint),
    After(S, TimePoint),
}

impl<S> DecisionRange<S> {
    pub fn initial_state(&self) -> &S {
        match self {
            DecisionRange::Between([s0, _]) => &s0.state,
            DecisionRange::Before(s, _) => s,
            DecisionRange::After(s, _) => s,
        }
    }

    pub fn final_state(&self) -> &S {
        match self {
            DecisionRange::Between([_, s1]) => &s1.state,
            DecisionRange::Before(s, _) => s,
            DecisionRange::After(s, _) => s,
        }
    }
}

impl<W: Waypoint, S: Clone> MetaTrajectory<W, S> {
    pub fn get_decision_range(&self, trajectory_index: usize) -> DecisionRange<S> {
        // dbg!((trajectory_index, &self));
        for i in 1..self.decision_points.len() {
            if trajectory_index < self.decision_points[i].index {
                // dbg!(i);
                // if dbg!(trajectory_index) < dbg!(self.decision_points[i].index) {
                return DecisionRange::Between([
                    self.decision_points[i - 1].clone(),
                    self.decision_points[i].clone(),
                ]);
            }
        }
        // If we couldn't find suitable decision points, then assume the conflict
        // runs past the end of the trajectory
        DecisionRange::After(
            self.final_state.clone(),
            self.trajectory.finish_motion_time() + Duration::from_secs(1),
        )
    }

    pub fn get_trajectory_segment(&self, range: &DecisionRange<S>) -> Trajectory<W> {
        match range {
            DecisionRange::Before(_, t) => {
                // This implies that we want an indefinite start trajectory leading
                // up to the first waypoint.
                let wp1 = self.trajectory.initial_motion().clone();
                let t = if *t == wp1.time() {
                    *t - Duration::from_secs(1)
                } else {
                    *t
                };
                let wp0 = wp1.clone().with_time(t);
                Trajectory::new(wp0, wp1).unwrap()
            }
            DecisionRange::After(_, t) => {
                let wp0 = self.trajectory.finish_motion().clone();
                let t = if *t == wp0.time() {
                    *t + Duration::from_secs(1)
                } else {
                    *t
                };
                let wp1 = wp0.clone().with_time(t);
                Trajectory::new(wp0, wp1).unwrap()
            }
            DecisionRange::Between(range) => {
                let i_max = self.trajectory.len() - 1;
                let mut wps: SmallVec<[_; 10]> = SmallVec::new();
                for i in range[0].index..=range[1].index {
                    wps.push(self.trajectory[usize::min(i, i_max)].0.clone());
                }

                // Remove any possible duplicated points
                wps.sort_by_key(|wp| wp.time());
                wps.dedup_by_key(|wp| wp.time());

                if wps.len() < 2 {
                    // If only one waypoint was extracted then it must be the final
                    // waypoint. Add a holding point so we can create a valid trajectory.
                    let wp0 = wps.last().unwrap();
                    wps.push(wp0.clone().time_shifted_by(Duration::from_secs(1)));
                }

                Trajectory::from_iter(wps).unwrap()
            }
        }
    }

    pub fn decision_start_time(&self, range: &DecisionRange<S>) -> TimePoint {
        match range {
            DecisionRange::Before(..) => self.trajectory.initial_motion_time(),
            DecisionRange::After(..) => self.trajectory.finish_motion_time(),
            DecisionRange::Between(range) => self
                .trajectory
                .get(range[0].index)
                .map(|wp| wp.time())
                .unwrap_or(self.trajectory.finish_motion_time()),
        }
    }

    pub fn with_indefinite_finish_time(mut self, value: bool) -> MetaTrajectory<W, S> {
        self.trajectory.set_indefinite_finish_time(value);
        self
    }
}

#[derive(Debug, ThisError)]
pub enum WaypointIntegrationError<S, A> {
    State(S),
    Action(A),
    EmptyInitialState,
}
