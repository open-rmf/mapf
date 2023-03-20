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

use crate::{motion::Timed, error::NoError};
use arrayvec::ArrayVec;

/// If a trait is reversible then you can obtain its Reverse whose choices and
/// dynamics can be used to search this domain in reverse.
///
/// This is useful for bidirectional search algorithms. Domains that implement
/// this trait will often have to implement [`Backtrack`] as well.
pub trait Reversible {
    /// What kind of error can happen if the domain has the wrong values in it.
    type ReversalError;

    /// Get the reverse of this domain
    fn reversed(&self) -> Result<Self, Self::ReversalError> where Self: Sized;
}

/// For a reverse domain created by [`Reversible`], the `Backtrack` trait can
/// be used to construct a forward path from the reverse states and reverse
/// actions. When making a [`Reversible`] domain it will usually be necessary to
/// implement this trait for the [`Reversible::Reverse`] type.
///
/// `ReverseState` should be the state of the domain that implements this trait
/// while `ReverseAction` should be the [`crate::domain::Activity::ActivityAction`]
/// implemented by the domain that this trait implements.
pub trait Backtrack<State, Action> {
    type BacktrackError;

    /// Change a final reverse state into its initial forward state
    /// counter-part. This is the first step in backtracking a reverse solution
    /// into a forward solution.
    ///
    /// * `initial_reverse_state` - The state where the reverse path begins
    /// * `final_reverse_state` - The state where the reverse path ends. May be
    /// a reference to the same instance as `initial_reverse_state` if there is
    /// only one state in the path.
    ///
    /// Returns `(initial_forward_state, final_forward_state)` effectively
    /// swapping the start and end states while modifying them as needed to
    /// construct an appropriate forward path.
    ///
    /// For example, in a reverse path the `initial_reverse_state` may have a
    /// time value of zero while `final_reverse_state` has a negative time
    /// value. This function can swap the states while modifying their time
    /// values so that the `initial_forward_state` has a time value of zero
    /// while `final_forward_state` has a positive time value.
    fn flip_endpoints(
        &self,
        initial_reverse_state: &State,
        final_reverse_state: &State,
    ) -> Result<(State, State), Self::BacktrackError>;

    /// Advance from a forward state using a reverse-action counter-part.
    fn backtrack(
        &self,
        parent_forward_state: &State,
        parent_reverse_state: &State,
        reverse_action: &Action,
        child_reverse_state: &State,
    ) -> Result<(Action, State), Self::BacktrackError>;
}

impl Reversible for () {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<(), Self::ReversalError> {
        Ok(())
    }
}

/// Use this to implement `flip_endpoints` for [`Backtrack`] when the only thing
/// that needs to happen is adjusting the times so that the
/// `initial_forward_state` time equals the `initial_reverse_state` time and
/// the `final_forward_state` time gets adjusted forward in time to match.
/// Everything else about the two states will be flipped.
pub fn flip_endpoint_times<State: Clone + Timed>(
    initial_reverse_state: &State,
    final_reverse_state: &State,
) -> Result<(State, State), NoError> {
    let delta_t = *initial_reverse_state.time() - *final_reverse_state.time();
    let mut initial_forward_state = final_reverse_state.clone();
    initial_forward_state.set_time(
        *initial_forward_state.time() + delta_t
    );

    let mut final_forward_state = initial_reverse_state.clone();
    final_forward_state.set_time(
        *final_forward_state.time() + delta_t
    );

    Ok((initial_forward_state, final_forward_state))
}

pub fn backtrack_times<State: Clone + Timed + std::fmt::Debug, const N: usize>(
    parent_forward_state: &State,
    parent_reverse_state: &State,
    reverse_action: &ArrayVec<State, N>,
    child_reverse_state: &State,
) -> Result<(ArrayVec<State, N>, State), NoError> {
    let dt = *parent_forward_state.time() - *parent_reverse_state.time();

    let mut child_forward_state = child_reverse_state.clone();
    child_forward_state.set_time(*child_forward_state.time() + dt);

    let mut forward_action = reverse_action.clone();
    // Check swap_endpoints now and save it because the length will change
    // when we pop the last waypoint.
    let swap_endpoints = !forward_action.is_empty();
    if swap_endpoints {
        // Pop this endpoint to replace it with the parent_forward_state later
        forward_action.pop();
    }
    forward_action.reverse();
    for wp in &mut forward_action {
        // Adjust the times of each waypoint
        wp.set_time(*wp.time() + dt);
    }
    if swap_endpoints {
        // Add the waypoint of the parent_forward_state, which is the
        // endpoint of this action
        forward_action.push(parent_forward_state.clone());
    }

    Ok((forward_action, child_forward_state))
}
