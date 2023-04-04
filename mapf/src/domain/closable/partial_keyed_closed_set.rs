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
    ClosedStatusForKey, TimeVariantPartialKeyedCloser,
};
use crate::{
    domain::{Keyed, Keyring, PartialKeyed, Reversible},
    error::NoError,
};
use std::{
    borrow::Borrow,
    collections::{hash_map::Entry, HashMap},
};

/// Factory for [`PartialKeyedClosedSet`]. Provide this to your domain, e.g.
/// [`crate::templates::InformedSearch`], to implement the [`Closable`] trait
/// for search spaces that are partially keyed.
#[derive(Debug, Clone)]
pub struct PartialKeyedCloser<Ring>(pub Ring);

impl<Ring: Clone> Reversible for PartialKeyedCloser<Ring> {
    type ReversalError = NoError;
    fn reversed(&self) -> Result<Self, Self::ReversalError>
    where
        Self: Sized,
    {
        Ok(self.clone())
    }
}

impl<State, Ring> Closable<State> for PartialKeyedCloser<Ring>
where
    Ring: PartialKeyed + Keyring<State, Key = Option<Ring::PartialKey>> + Clone,
    Ring::PartialKey: Clone,
{
    type ClosedSet<T> = PartialKeyedClosedSet<Ring, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        PartialKeyedClosedSet::new(self.0.clone())
    }
}

impl<Ring> AsTimeInvariant for PartialKeyedCloser<Ring> {
    type TimeInvariantClosable = Self;
    fn as_time_invariant(self) -> Self::TimeInvariantClosable {
        self
    }
}

impl<Ring> AsTimeVariant for PartialKeyedCloser<Ring> {
    type TimeVariantClosable = TimeVariantPartialKeyedCloser<Ring>;
    fn as_time_variant(self) -> Self::TimeVariantClosable {
        TimeVariantPartialKeyedCloser::new(self.0)
    }
}

/// A PartialKeyedClosedSet is able to act as a closed set for States with the
/// PartialKeyed trait. It behaves the same as KeyedClosedSet except any states
/// that cannot map to a unique key will always be Accepted when told to close,
/// and will always return an Open result when queried for the status.
///
/// Note that keyless states will not be stored at all in the container of this
/// set.
#[derive(Debug, Clone)]
pub struct PartialKeyedClosedSet<Ring: PartialKeyed, T> {
    keyring: Ring,
    container: HashMap<Ring::PartialKey, T>,
}

impl<Ring: Default + PartialKeyed, T> Default for PartialKeyedClosedSet<Ring, T> {
    fn default() -> Self {
        PartialKeyedClosedSet {
            keyring: Default::default(),
            container: Default::default(),
        }
    }
}

impl<Ring: PartialKeyed, T> PartialKeyedClosedSet<Ring, T> {
    pub fn new(keyring: Ring) -> Self {
        Self {
            keyring,
            container: Default::default(),
        }
    }
}

impl<State, Ring, T> ClosedSet<State, T> for PartialKeyedClosedSet<Ring, T>
where
    Ring: PartialKeyed + Keyed<Key = Option<Ring::PartialKey>> + Keyring<State>,
    Ring::PartialKey: Clone,
{
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return CloseResult::Accepted,
        };
        match self.container.entry(key.clone()) {
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
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return None,
        };
        self.container.insert(key.clone(), value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key_ref = self.keyring.key_for(state);
        let key = match key_ref.borrow() {
            Some(key) => key,
            None => return ClosedStatus::Open,
        };
        self.container.get(key).into()
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
        self.container.values()
    }
}

impl<Ring, T> ClosedStatusForKey<Option<Ring::PartialKey>, T> for PartialKeyedClosedSet<Ring, T>
where
    Ring: PartialKeyed + Keyed<Key = Option<Ring::PartialKey>>,
{
    fn status_for_key<'a>(&'a self, key: &Option<Ring::PartialKey>) -> ClosedStatus<'a, T> {
        if let Some(key) = key {
            self.container.get(key).into()
        } else {
            ClosedStatus::Open
        }
    }

    fn closed_keys_len(&self) -> usize {
        self.container.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{PartialKeyed, SelfKey, SelfPartialKeyring};

    struct TestState {
        index: Option<usize>,
        #[allow(unused)]
        time: f64,
    }

    impl TestState {
        fn new(index: Option<usize>, time: f64) -> Self {
            TestState { index, time }
        }
    }

    impl PartialKeyed for TestState {
        type PartialKey = usize;
    }

    impl Keyed for TestState {
        type Key = Option<usize>;
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
    fn test_partial_keyed_closed_set() {
        let mut closed_set = PartialKeyedClosedSet::new(SelfPartialKeyring::<usize>::new());
        assert!(closed_set
            .close(&TestState::new(Some(1), 0.24), 0)
            .accepted());
        assert!(closed_set
            .status(&TestState::new(Some(1), 0.55))
            .is_closed());
        assert!(closed_set
            .close(&TestState::new(Some(1), 0.1), 32)
            .rejected());

        assert!(closed_set.close(&TestState::new(None, 0.4), 0).accepted());
        assert!(closed_set.status(&TestState::new(None, 123.4)).is_open());
        assert!(closed_set.close(&TestState::new(None, 0.01), 3).accepted());
    }
}
