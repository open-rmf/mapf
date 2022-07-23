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
    node::Keyed,
    motion::{
        Extrapolator, TimePoint,
        trajectory::CostCalculator,
        r2,
    },
    heuristic::Heuristic,
    graph::Graph,
};
use num::Zero;
use std::sync::Arc;

#[derive(Debug)]
pub struct DirectTravelHeuristic<G: Graph<Vertex=r2::Position>, C: CostCalculator<r2::timed_position::Waypoint>> {
    pub graph: Arc<G>,
    pub cost_calculator: Arc<C>,
    pub extrapolator: r2::timed_position::LineFollow,
}

impl<G, C, S, Goal> Heuristic<S, Goal, C::Cost> for DirectTravelHeuristic<G, C>
where
    G: Graph<Vertex=r2::Position>,
    C: CostCalculator<r2::timed_position::Waypoint>,
    S: Into<G::Key> + Clone,
    Goal: Keyed<Key=G::Key>,
{
    type Error = <r2::timed_position::LineFollow as Extrapolator<r2::timed_position::Waypoint, r2::Position>>::Error;

    fn estimate_cost(
        &self,
        from_state: &S,
        to_goal: &Goal,
    ) -> Result<Option<C::Cost>, Self::Error> {
        let p0 = {
            // Should this be an error instead? The current state is off the
            // graph entirely.
            if let Some(p) = self.graph.vertex(from_state.clone().into()) {
                p
            } else {
                return Ok(None);
            }
        };

        let p1 = {
            if let Some(p) = self.graph.vertex(to_goal.key().clone()) {
                p
            } else {
                return Ok(None);
            }
        };

        let wp0 = r2::timed_position::Waypoint{
            time: TimePoint::zero(),
            position: *p0,
        };

        let cost = self.extrapolator.make_trajectory(wp0, p1)?
            .map(|t| self.cost_calculator.compute_cost(&t))
            .unwrap_or(C::Cost::zero());
        Ok(Some(cost))
    }
}
