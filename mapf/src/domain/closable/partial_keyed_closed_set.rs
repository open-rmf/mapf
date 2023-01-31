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

use super::{ClosedSet, CloseResult, ClosedStatus, keyed_closed_set::{Key, Keyed}};
use std::collections::{HashMap, hash_map::Entry};

/// The `PartialKeyed` trait is similar to `Keyed` except that some unique
/// states might not be able to map to a unique key. A [`PartialKeyedClosedSet`]
/// can be used to prune some state duplication but not all.
pub trait PartialKeyed {
    type PartialKey: Key;
    fn partial_key(&self) -> Option<Self::PartialKey>;
}

impl<T: Keyed> PartialKeyed for T {
    type PartialKey = T::Key;
    fn partial_key(&self) -> Option<Self::PartialKey> {
        Some(self.key())
    }
}

/// A PartialKeyedClosedSet is able to act as a closed set for States with the
/// PartialKeyed trait. It behaves the same as KeyedClosedSet except any states
/// that cannot map to a unique key will always be Accepted when told to close,
/// and will always return an Open result when queried for the status.
///
/// Note that keyless states will not be stored at all in the container of this
/// set.
pub struct PartialKeyedClosedSet<State: PartialKeyed, T> {
    container: HashMap<State::PartialKey, T>,
}

impl<State: PartialKeyed, T> Default for PartialKeyedClosedSet<State, T> {
    fn default() -> Self {
        PartialKeyedClosedSet { container: Default::default() }
    }
}

impl<State: PartialKeyed, T> ClosedSet<State, T> for PartialKeyedClosedSet<State, T> {
    fn close<'a>(&'a mut self, state: State, value: T) -> CloseResult<'a, T> {
        let key = match state.partial_key() {
            Some(key) => key,
            None => return CloseResult::Accepted,
        };
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
        let key = match state.partial_key() {
            Some(key) => key,
            None => return None,
        };
        self.container.insert(key, value)
    }

    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T> {
        let key = match state.partial_key() {
            Some(key) => key,
            None => return ClosedStatus::Open,
        };
        self.container.get(&key).into()
    }
}

mod tests {
    use super::*;

    struct TestState {
        index: Option<usize>,
        time: f64,
    }

    impl TestState {
        fn new(index: Option<usize>, time: f64) -> Self {
            TestState { index, time }
        }
    }

    impl PartialKeyed for TestState {
        type PartialKey = usize;
        fn partial_key(&self) -> Option<usize> {
            self.index
        }
    }

    #[test]
    fn test_partial_keyed_closed_set() {
        let mut closed_set = PartialKeyedClosedSet::default();
        assert!(closed_set.close(TestState::new(Some(1), 0.24), 0).accepted());
        assert!(closed_set.status(&TestState::new(Some(1), 0.55)).is_closed());
        assert!(closed_set.close(TestState::new(Some(1), 0.1), 32).rejected());

        assert!(closed_set.close(TestState::new(None, 0.4), 0).accepted());
        assert!(closed_set.status(&TestState::new(None, 123.4)).is_open());
        assert!(closed_set.close(TestState::new(None, 0.01), 3).accepted());
    }
}

