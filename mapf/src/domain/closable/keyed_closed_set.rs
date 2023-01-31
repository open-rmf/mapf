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

use super::{ClosedSet, CloseResult, ClosedStatus};
use std::{cmp::Eq, hash::Hash, collections::{HashMap, hash_map::Entry}};

/// The `Key` trait describes objects that can be used as Keys by KeyedClosedSet
pub trait Key: Hash + Eq + Send + Sync + 'static {}
impl<T: Hash + Eq + Send + Sync + 'static> Key for T {}

/// The `Keyed` trait should be implemented by states to be used with
/// KeyedClosedSet
// TODO(MXG): Provide a derive macro for giving the Keyed trait to structs with
// the necessary traits to be the key themselves.
pub trait Keyed {
    type Key: Key;
    fn key(&self) -> Self::Key;
}

pub struct KeyedClosedSet<State: Keyed, T> {
    container: HashMap<State::Key, T>,
}

impl<State: Keyed, T> Default for KeyedClosedSet<State, T> {
    fn default() -> Self {
        KeyedClosedSet { container: Default::default() }
    }
}

impl<State: Keyed, T> ClosedSet<State, T> for KeyedClosedSet<State, T> {
    fn close<'a>(&'a mut self, state: State, value: T) -> CloseResult<'a, T> {
        let key = state.key();
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

    fn replace(&mut self, state: State, value: T) -> Option<T> {
        let key = state.key();
        self.container.insert(key, value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key = state.key();
        self.container.get(&key).into()
    }
}

mod tests {
    use super::*;

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
        fn key(&self) -> Self::Key {
            self.index
        }
    }

    #[test]
    fn test_keyed_closed_set() {
        let mut closed_set = KeyedClosedSet::default();
        assert!(closed_set.close(TestState::new(1, 0.24), 0).accepted());
        assert!(closed_set.status(&TestState::new(1, 0.55)).is_closed());
        assert!(closed_set.close(TestState::new(1, 0.1), 32).rejected());
    }
}
