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

use crate::domain::{Keyed, Keyring};
use super::{
    Closable, ClosedSet, CloseResult, ClosedStatus, ClosedStatusForKey,
    AsTimeInvariant, AsTimeVariant, TimeVariantKeyedCloser,
};
use std::{
    collections::{HashMap, hash_map::Entry},
    borrow::Borrow,
};

/// [`KeyedCloser`] implements the [`Closable`] trait by providing a
/// [`KeyedClosedSet`].
pub struct KeyedCloser<Ring>(pub Ring);

impl<State, Ring> Closable<State> for KeyedCloser<Ring>
where
    Ring: Keyring<State> + Clone,
    Ring::Key: Clone,
{
    type ClosedSet<T> = KeyedClosedSet<Ring, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        KeyedClosedSet::new(self.0.clone())
    }
}

impl<Ring> AsTimeInvariant for KeyedCloser<Ring> {
    type TimeInvariantClosable = Self;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable {
        self
    }
}

impl<Ring> AsTimeVariant for KeyedCloser<Ring> {
    type TimeVariantClosable = TimeVariantKeyedCloser<Ring>;
    fn as_time_variant(self) -> Self::TimeVariantClosable {
        TimeVariantKeyedCloser::new(self.0)
    }
}

pub struct KeyedClosedSet<Ring: Keyed, T> {
    keyring: Ring,
    container: HashMap<Ring::Key, T>,
}

impl<Ring: Keyed + Default, T> Default for KeyedClosedSet<Ring, T> {
    fn default() -> Self {
        KeyedClosedSet {
            keyring: Default::default(),
            container: Default::default(),
        }
    }
}

impl<Ring: Keyed, T> KeyedClosedSet<Ring, T> {
    pub fn new(keyring: Ring) -> Self {
        Self {
            keyring,
            container: Default::default(),
        }
    }
}

impl<State, Ring, T> ClosedSet<State, T> for KeyedClosedSet<Ring, T>
where
    Ring: Keyring<State>,
    Ring::Key: Clone,
{
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        let key = self.keyring.key_for(state);
        match self.container.entry(key.borrow().clone()) {
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
        let key = self.keyring.key_for(state);
        self.container.insert(key.borrow().clone(), value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key = self.keyring.key_for(state);
        self.container.get(key.borrow()).into()
    }
}

impl<Ring: Keyed, T> ClosedStatusForKey<Ring::Key, T> for KeyedClosedSet<Ring, T> {
    fn status_for_key<'a>(&'a self, key: &Ring::Key) -> ClosedStatus<'a, T> {
        self.container.get(key).into()
    }

    fn closed_keys_len(&self) -> usize {
        self.container.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{Keyed, SelfKey, SelfKeyring};

    struct TestState {
        index: usize,
        time: f64,
    }

    impl TestState {
        fn new(index: usize, time: f64) -> Self {
            TestState { index, time }
        }
    }

    impl Keyed for TestState {
        type Key = usize;
    }

    impl SelfKey for TestState {
        type KeyRef<'a> = &'a Self::Key;
        fn key<'a>(&'a self) -> &'a Self::Key
        where
            Self: 'a,
        {
            &self.index
        }
    }

    #[test]
    fn test_keyed_closed_set() {
        let mut closed_set = KeyedClosedSet::new(SelfKeyring::<usize>::new());
        assert!(closed_set.close(&TestState::new(1, 0.24), 0).accepted());
        assert!(closed_set.status(&TestState::new(1, 0.55)).is_closed());
        assert!(closed_set.close(&TestState::new(1, 0.1), 32).rejected());
    }
}
