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

/// A trait for domains whose states can be "closed" meaning it is possible to
/// identify when a state is being repeated in a search and prune that state
/// out of any further search effort.
pub trait Closable<State> {
    type ClosedSet<T>: ClosedSet<State, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T>;
}

/// The result of attempting to close a state.
#[must_use]
pub enum CloseResult<'a, T> {
    /// The state was successfully closed. This state was never previously
    /// closed.
    Accepted,

    /// The state was previously closed. See a reference to the previously
    /// closed value.
    Rejected {
        /// The value that has been rejected is returned back to the caller
        value: T,
        /// A mutable reference to the prior value is provided
        prior: &'a mut T,
    },
}

impl<'a, T> CloseResult<'a, T> {
    /// Was the attempt to close this entry accepted.
    pub fn accepted(&self) -> bool {
        match self {
            Self::Accepted => true,
            Self::Rejected { .. } => false,
        }
    }

    /// Opposite of [`accepted`]
    pub fn rejected(&self) -> bool {
        !self.accepted()
    }
}

/// The status of whether a state is closed yet.
#[must_use]
pub enum ClosedStatus<'a, T> {
    /// The state is open (it has never been assigned a value).
    Open,

    /// The state has been closed with the specified value.
    Closed(&'a T),
}

impl<'a, T> ClosedStatus<'a, T> {
    pub fn is_open(&self) -> bool {
        match self {
            Self::Open => true,
            Self::Closed(_) => false,
        }
    }

    pub fn is_closed(&self) -> bool {
        !self.is_open()
    }

    pub fn closed(self) -> Option<&'a T> {
        match self {
            Self::Open => None,
            Self::Closed(t) => Some(t),
        }
    }
}

impl<'a, T> From<Option<&'a T>> for ClosedStatus<'a, T> {
    fn from(value: Option<&'a T>) -> Self {
        match value {
            Some(v) => ClosedStatus::Closed(v),
            None => ClosedStatus::Open,
        }
    }
}

/// The trait for the closed set of a domain. The underlying struct should be
/// some kind of container that can keep track of a T for each closed State.
pub trait ClosedSet<State, T> {
    /// Close a state with a value. If the state was already closed with a prior
    /// value, it will retain its original value.
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T>;

    /// Replace a state's value. It will take on the new value regardless of
    /// whether the state was already closed. If a value already existed for
    /// this state, it will be returned.
    fn replace(&mut self, state: &State, value: T) -> Option<T>;

    /// Get the status of the specified state.
    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T>;

    type ClosedSetIter<'a>: IntoIterator<Item = &'a T> + 'a
    where
        Self: 'a,
        State: 'a,
        T: 'a;

    fn iter_closed<'a>(&'a self) -> Self::ClosedSetIter<'a>
    where
        Self: 'a,
        State: 'a,
        T: 'a;
}

/// This trait can supplement [`ClosedSet`] by allowing the closed items to be
/// looked up by a key. If there are multiple closed items per key then it is
/// up to the implementation to decide which to return.
pub trait ClosedStatusForKey<Key, T> {
    /// Get the closed status for this key.
    fn status_for_key<'a>(&'a self, key: &Key) -> ClosedStatus<'a, T>;

    /// How many keys have been closed in this set.
    fn closed_keys_len(&self) -> usize;
}

/// A trait for closables that can be converted to time variant.
/// This can also be implemented for closables that are already time variant as
/// they can return themselves unchanged.
pub trait AsTimeVariant {
    type TimeVariantClosable;
    fn as_time_variant(self) -> Self::TimeVariantClosable;
}

/// A trait for closables that can be converted to time invariant.
/// This can also be implemented for closables that are already time invariant
/// as they can return themselves unchanged.
pub trait AsTimeInvariant {
    type TimeInvariantClosable;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable;
}

pub mod keyed_closed_set;
pub use keyed_closed_set::*;

pub mod partial_keyed_closed_set;
pub use partial_keyed_closed_set::*;

pub mod time_variant_keyed_closed_set;
pub use time_variant_keyed_closed_set::*;

pub mod time_variant_partial_keyed_closed_set;
pub use time_variant_partial_keyed_closed_set::*;
