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
use super::{Closable, ClosedSet, CloseResult, ClosedStatus};
use std::{collections::{HashMap, hash_map::Entry}};

/// [`KeyedCloser`] implements the [`Closable`] trait by providing a
/// [`KeyedClosedSet`].
pub struct KeyedCloser<Ring>(pub Ring);

impl<State, Ring: Keyring<State> + Clone> Closable<State> for KeyedCloser<Ring> {
    type ClosedSet<T> = KeyedClosedSet<Ring, T>;
    fn new_closed_set<T>(&self) -> Self::ClosedSet<T> {
        KeyedClosedSet::new(self.0.clone())
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

impl<State, Ring: Keyring<State>, T> ClosedSet<State, T> for KeyedClosedSet<Ring, T> {
    fn close<'a>(&'a mut self, state: &State, value: T) -> CloseResult<'a, T> {
        let key = self.keyring.key_for(state);
        match self.container.entry(key) {
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
        self.container.insert(key, value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key = self.keyring.key_for(state);
        self.container.get(&key).into()
    }
}

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
        fn key(&self) -> Self::Key {
            self.index
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