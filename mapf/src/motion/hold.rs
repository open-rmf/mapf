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
    error::NoError,
    expander::traits::{Expander, Goal, Targeted},
    motion::{
        movable::{ArcMovable, Movable},
        trajectory::CostCalculator,
        Duration, Trajectory, Waypoint,
    },
    node::Informed,
};
use std::sync::Arc;

pub const DEFAULT_HOLD_SECS: i64 = 1;

pub struct Hold<W: Waypoint, C: CostCalculator<W>, N> {
    cost_calculator: Arc<C>,
    duration: Duration,
    _ignore: std::marker::PhantomData<(W, N)>,
}

impl<W: Waypoint, C: CostCalculator<W>, N: Movable<W>> Hold<W, C, N> {
    pub fn new(cost_calculator: Arc<C>) -> Self {
        Self {
            cost_calculator,
            duration: Duration::from_secs(DEFAULT_HOLD_SECS),
            _ignore: Default::default(),
        }
    }

    pub fn for_duration(mut self, duration: Duration) -> Result<Self, ()> {
        if duration.nanos <= 0 {
            return Err(());
        }

        self.duration = duration;
        Ok(self)
    }
}

impl<W: Waypoint, C: CostCalculator<W>, N> Expander for Hold<W, C, N> {
    type Node = N;
}

impl<W: Waypoint, C: CostCalculator<W, Cost = N::Cost>, N: Informed + Movable<W>, G: Goal<N>>
    Targeted<G> for Hold<W, C, N>
{
    type TargetedError = NoError;
    type TargetedExpansion<'a> = impl Iterator<Item=Result<Arc<N>, NoError>> where W: 'a, N: 'a, C: 'a, G: 'a ;

    fn expand<'a>(&'a self, parent: &'a Arc<N>, _: &'a G) -> Self::TargetedExpansion<'a> {
        // SAFETY: The value check on for_duration makes sure that duration is
        // always positive so the hold will always be valid.
        let until_time = *parent.state().time() + self.duration;
        let trajectory = Trajectory::hold(parent.state().clone(), until_time).unwrap();
        let cost_from_parent = self.cost_calculator.compute_cost(&trajectory);
        [Ok(parent.clone().moved_with(
            parent.partial_key().cloned(),
            cost_from_parent,
            parent.remaining_cost_estimate(),
            Some(trajectory),
        ))]
        .into_iter()
    }
}
