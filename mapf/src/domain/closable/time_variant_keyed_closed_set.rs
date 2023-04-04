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

use super::{
    AsTimeInvariant, AsTimeVariant, Closable, CloseResult, ClosedSet, ClosedStatus,
    ClosedStatusForKey, KeyedCloser,
};
use crate::{
    domain::{Keyed, Keyring, Reversible},
    error::NoError,
    motion::Timed,
};
use std::{
    borrow::Borrow,
    collections::{hash_map::Entry, HashMap},
};

pub const DEFAULT_TIME_THRESH: i64 = 100_000_000;

/// Factory for [`TimeVariantKeyedClosedSet`]. Provide this to your domain,
/// e.g. [`crate::template::InformedSearch`], to implement the [`Closable`]
/// trait for search spaces that are keyed and time variant.
#[derive(Debug, Clone)]
pub struct TimeVariantKeyedCloser<Ring> {
    pub ring: Ring,
    pub time_thresh: i64,
}

impl<Ring: Clone> Reversible for TimeVariantKeyedCloser<Ring> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        Ok(self.clone())
    }
}

impl<Ring> TimeVariantKeyedCloser<Ring> {
    pub fn new(ring: Ring) -> Self {
        Self {
            ring,
            time_thresh: DEFAULT_TIME_THRESH,
        }
    }
}

impl<Ring: Default> Default for TimeVariantKeyedCloser<Ring> {
    fn default() -> Self {
        Self {
            ring: Default::default(),
            time_thresh: DEFAULT_TIME_THRESH,
        }
    }
}

impl<Ring> AsTimeInvariant for TimeVariantKeyedCloser<Ring> {
    type TimeInvariantClosable = KeyedCloser<Ring>;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable {
        KeyedCloser(self.ring)
    }
}

impl<Ring> AsTimeVariant for TimeVariantKeyedCloser<Ring> {
    type TimeVariantClosable = Self;
    fn as_time_variant(self) -> Self::TimeVariantClosable {
        self
    }
}

impl<State, Ring> Closable<State> for TimeVariantKeyedCloser<Ring>
where
    Ring: Keyring<State> + Clone,
    Ring::Key: Clone,
    State: Timed,
{
    type ClosedSet<T> = TimeVariantKeyedClosedSet<Ring, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        TimeVariantKeyedClosedSet::new(self.ring.clone(), self.time_thresh)
    }
}

#[derive(Debug, Clone)]
pub struct TimeVariantKeyedClosedSet<Ring: Keyed, T> {
    keyring: Ring,
    container: HashMap<Ring::Key, HashMap<i64, T>>,
    time_thresh: i64,
}

impl<Ring: Keyed + Default, T> Default for TimeVariantKeyedClosedSet<Ring, T> {
    fn default() -> Self {
        Self {
            keyring: Default::default(),
            container: Default::default(),
            time_thresh: DEFAULT_TIME_THRESH,
        }
    }
}

impl<Ring: Keyed, T> TimeVariantKeyedClosedSet<Ring, T> {
    pub fn new(keyring: Ring, time_thresh: i64) -> Self {
        Self {
            keyring,
            container: Default::default(),
            time_thresh,
        }
    }
}

impl<Ring, T, State> ClosedSet<State, T> for TimeVariantKeyedClosedSet<Ring, T>
where
    Ring: Keyring<State>,
    Ring::Key: Clone,
    State: Timed,
{
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = key_ref.borrow();
        let time_key = state.time().nanos_since_zero / self.time_thresh;

        match self
            .container
            .entry(key.clone())
            .or_default()
            .entry(time_key)
        {
            Entry::Occupied(entry) => CloseResult::Rejected {
                value,
                prior: entry.into_mut(),
            },
            Entry::Vacant(entry) => {
                entry.insert(value);
                CloseResult::Accepted
            }
        }
    }

    fn replace(&mut self, state: &State, value: T) -> Option<T> {
        let key_ref = self.keyring.key_for(state);
        let key = key_ref.borrow();
        let time_key = state.time().nanos_since_zero / self.time_thresh;

        self.container
            .entry(key.clone())
            .or_default()
            .insert(time_key, value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = key_ref.borrow();
        let time_key = state.time().nanos_since_zero / self.time_thresh;

        self.container
            .get(key)
            .map(|c| c.get(&time_key))
            .flatten()
            .into()
    }

    type ClosedSetIter<'a> = impl Iterator<Item=&'a T> + 'a
    where
        Self: 'a,
        State: 'a,
        T: 'a;

    fn iter_closed<'a>(&'a self) -> Self::ClosedSetIter<'a>
    where
        Self: 'a,
        State: 'a,
        T: 'a,
    {
        self.container.values().flat_map(|c| c.values())
    }
}

impl<Ring: Keyed, T> ClosedStatusForKey<Ring::Key, T> for TimeVariantKeyedClosedSet<Ring, T> {
    fn status_for_key<'a>(&'a self, key: &Ring::Key) -> ClosedStatus<'a, T> {
        self.container
            .get(key)
            .map(|c| c.iter().min_by_key(|(t, _)| **t).map(|(_, value)| value))
            .flatten()
            .into()
    }

    fn closed_keys_len(&self) -> usize {
        self.container.len()
    }
}
