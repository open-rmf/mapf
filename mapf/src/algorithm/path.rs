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
    domain::{Domain, Backtrack},
    motion::{Trajectory, Waypoint, IntegrateWaypoints},
    error::ThisError,
};

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
        let (_, mut parent_forward_state) = reverse_domain.flip_endpoints(
            &self.initial_state,
            self.sequence.last().map(|s| &s.1).unwrap_or(&self.initial_state),
        ).map_err(Into::into)?;
        let mut parent_reverse_state = self.initial_state;

        let mut sequence = Vec::new();
        for (reverse_action, child_reverse_state) in self.sequence {
            let (forward_action, child_forward_state) = reverse_domain.backtrack(
                &parent_forward_state,
                &parent_reverse_state,
                &reverse_action,
                &child_reverse_state,
            ).map_err(Into::into)?;

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

    pub fn make_trajectory<W: Waypoint>(
        &self
    ) -> Result<
        Option<Trajectory<W>>,
        WaypointIntegrationError<S::WaypointIntegrationError, A::WaypointIntegrationError>
    >
    where
        S: IntegrateWaypoints<W>,
        A: IntegrateWaypoints<W>,
    {
        let mut waypoints = self
            .initial_state
            .integrated_waypoints(None)
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .map_err(WaypointIntegrationError::State)?;

        let mut initial_wp = waypoints.last().cloned();
        for (action, _) in self.sequence.iter() {
            waypoints.extend(
                action
                .integrated_waypoints(initial_wp)
                .into_iter()
                .collect::<Result<Vec<_>, _>>()
                .map_err(WaypointIntegrationError::Action)?
            );

            initial_wp = waypoints.last().cloned();
        }

        Ok(Trajectory::from_iter(waypoints).ok())
    }
}

#[derive(Debug, ThisError)]
pub enum WaypointIntegrationError<S, A> {
    State(S),
    Action(A),
}
