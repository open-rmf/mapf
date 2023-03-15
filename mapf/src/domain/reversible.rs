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

/// If a trait is reversible then you can obtain its Reverse whose choices and
/// dynamics can be used to search this domain in reverse.
///
/// This is useful for bidirectional search algorithms. Domains that implement
/// this trait will often have to implement [`Backtrack`] as well.
pub trait Reversible {
    /// The type of the reverse domain
    type Reverse;

    /// What kind of error can happen if the domain has the wrong values in it.
    type ReversalError;

    /// Get the reverse of this domain
    fn reversed(&self) -> Result<Self::Reverse, Self::ReversalError>;
}

/// For a reverse domain created by [`Reversible`], the `Backtrack` trait can
/// be used to construct a forward path from the reverse states and reverse
/// actions. When making a [`Reversible`] domain it will usually be necessary to
/// implement this trait for the [`Reversible::Reverse`] type.
///
/// `ReverseState` should be the state of the domain that implements this trait
/// while `ReverseAction` should be the [`crate::domain::Activity::ActivityAction`]
/// implemented by the domain that this trait implements.
pub trait Backtrack<ReverseState, ReverseAction> {
    type ForwardState;
    type ForwardAction;
    type BacktrackError;

    /// Change a final reverse state into its initial forward state
    /// counter-part. This is the first step in backtracking a reverse solution
    /// into a forward solution.
    fn flip_state(
        &self,
        final_state: &ReverseState
    ) -> Result<Self::ForwardState, Self::BacktrackError>;

    /// Advance from a forward state using a reverse-action counter-part.
    fn backtrack(
        &self,
        parent_forward_state: &Self::ForwardState,
        parent_reverse_state: &ReverseState,
        reverse_action: &ReverseAction,
        child_reverse_state: &ReverseState,
    ) -> Result<(Self::ForwardAction, Self::ForwardState), Self::BacktrackError>;
}
