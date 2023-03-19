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
    domain::{PartialKeyed, Keyed, Keyring, Reversible},
    motion::Timed,
    error::NoError,
};
use super::{
    Closable, ClosedSet, CloseResult, ClosedStatus, ClosedStatusForKey,
    AsTimeInvariant, AsTimeVariant, PartialKeyedCloser,
    DEFAULT_TIME_THRESH,
};
use std::{
    collections::{HashMap, hash_map::Entry},
    borrow::Borrow,
};

/// Factory for [`TimeVariantPartialKeyedClosedSet`]. Provide this to your domain,
/// e.g. [`crate::templates::InformedSearch`], to implement the [`Closable`]
/// trait for search spaces that are partially keyed and time variant.
#[derive(Debug, Clone)]
pub struct TimeVariantPartialKeyedCloser<Ring> {
    pub ring: Ring,
    pub time_thresh: i64,
}

impl<Ring: Clone> Reversible for TimeVariantPartialKeyedCloser<Ring> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError> where Self: Sized {
        Ok(self.clone())
    }
}

impl<Ring> TimeVariantPartialKeyedCloser<Ring> {
    pub fn new(ring: Ring) -> Self {
        Self { ring, time_thresh: DEFAULT_TIME_THRESH }
    }
}

impl<Ring: Default> Default for TimeVariantPartialKeyedCloser<Ring> {
    fn default() -> Self {
        Self {
            ring: Default::default(),
            time_thresh: DEFAULT_TIME_THRESH,
        }
    }
}

impl<Ring> AsTimeInvariant for TimeVariantPartialKeyedCloser<Ring> {
    type TimeInvariantClosable = PartialKeyedCloser<Ring>;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable {
        PartialKeyedCloser(self.ring)
    }
}

impl<Ring> AsTimeVariant for TimeVariantPartialKeyedCloser<Ring> {
    type TimeVariantClosable = Self;
    fn as_time_variant(self) -> Self::TimeVariantClosable {
        self
    }
}

/// Similar to [`super::PartialKeyedClosedSet`] except values are also considered
/// unique based on their time value.
impl<State, Ring> Closable<State> for TimeVariantPartialKeyedCloser<Ring>
where
    Ring: PartialKeyed + Keyring<State, Key=Option<Ring::PartialKey>> + Clone,
    Ring::PartialKey: Clone,
    State: Timed,
{
    type ClosedSet<T> = TimeVariantPartialKeyedClosedSet<Ring, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        TimeVariantPartialKeyedClosedSet::new(self.ring.clone(), self.time_thresh)
    }
}

#[derive(Debug, Clone)]
pub struct TimeVariantPartialKeyedClosedSet<Ring: PartialKeyed, T> {
    keyring: Ring,
    container: HashMap<Ring::PartialKey, HashMap<i64, T>>,
    time_thresh: i64,
}

impl<Ring: PartialKeyed + Default, T> Default for TimeVariantPartialKeyedClosedSet<Ring, T> {
    fn default() -> Self {
        Self {
            keyring: Default::default(),
            container: Default::default(),
            time_thresh: DEFAULT_TIME_THRESH,
        }
    }
}

impl<Ring: PartialKeyed, T> TimeVariantPartialKeyedClosedSet<Ring, T> {
    pub fn new(keyring: Ring, time_thresh: i64) -> Self {
        Self {
            keyring,
            container: Default::default(),
            time_thresh,
        }
    }
}

impl<Ring, T, State> ClosedSet<State, T> for TimeVariantPartialKeyedClosedSet<Ring, T>
where
    Ring: PartialKeyed + Keyed<Key=Option<Ring::PartialKey>> + Keyring<State>,
    Ring::PartialKey: Clone,
    State: Timed,
{
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return CloseResult::Accepted,
        };
        let time_key = state.time().nanos_since_zero/self.time_thresh;

        match self.container.entry(key.clone()).or_default().entry(time_key) {
            Entry::Occupied(entry) => {
                CloseResult::Rejected { value, prior: entry.into_mut() }
            }
            Entry::Vacant(entry) => {
                entry.insert(value);
                CloseResult::Accepted
            }
        }
    }

    fn replace(&mut self, state: &State, value: T) -> Option<T> {
        let key_ref = self.keyring.key_for(state);
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return None,
        };
        let time_key = state.time().nanos_since_zero/self.time_thresh;

        self.container.entry(key.clone()).or_default().insert(time_key, value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return ClosedStatus::Open,
        };
        let time_key = state.time().nanos_since_zero/self.time_thresh;

        self.container.get(key).map(|c| c.get(&time_key)).flatten().into()
    }
}

impl<Ring, T> ClosedStatusForKey<Option<Ring::PartialKey>, T> for TimeVariantPartialKeyedClosedSet<Ring, T>
where
    Ring: PartialKeyed + Keyed<Key=Option<Ring::PartialKey>>,
{
    fn status_for_key<'a>(&'a self, key: &Option<Ring::PartialKey>) -> ClosedStatus<'a, T> {
        if let Some(key) = key {
            self.container.get(key).map(|c|
                c
                .iter()
                .min_by_key(|(t, _)| **t)
                .map(|(_, value)| value)
            ).flatten().into()
        } else {
            ClosedStatus::Open
        }
    }

    fn closed_keys_len(&self) -> usize {
        self.container.len()
    }
}
