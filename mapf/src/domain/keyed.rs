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

use std::{cmp::Eq, hash::Hash};

/// The `Key` trait describes objects that can be used as Keys by Keyed domains
pub trait Key: Hash + Eq + Send + Sync + 'static {}
impl<T: Hash + Eq + Send + Sync + 'static> Key for T {}

/// `Keyed` is used to indicate an associated [`Key`] type for a struct.
pub trait Keyed {
    type Key: Key;
}

impl<T: Key + Clone> Keyed for T {
    type Key = T;
}

pub trait PartialKeyed {
    type PartialKey: Key;
}

/// The `Keyring` trait is implemented by structs that can produce a key for a
/// given state.
pub trait Keyring<State>: Keyed {
    fn key_for(&self, state: &State) -> Self::Key;
}

/// If a State contains its own key, it can implement SelfKey so that its key
/// can be obtained without a [`Keyring`].
pub trait SelfKey: Keyed {
    fn key(&self) -> Self::Key;
}

impl<T: Key + Clone> SelfKey for T {
    fn key(&self) -> Self::Key {
        self.clone()
    }
}

/// Implements a [`Keyring`] for states that implement [`SelfKey`].
pub struct SelfKeyring<K>(std::marker::PhantomData<K>);

impl<K> SelfKeyring<K> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<K> Default for SelfKeyring<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Key> Keyed for SelfKeyring<K> {
    type Key = K;
}

impl<State: SelfKey> Keyring<State> for SelfKeyring<State::Key> {
    fn key_for(&self, state: &State) -> Self::Key {
        state.key()
    }
}

pub struct SelfPartialKeyring<K>(std::marker::PhantomData<K>);

impl<K> SelfPartialKeyring<K> {
    pub fn new() -> Self {
        Self(Default::default())
    }
}

impl<K> Default for SelfPartialKeyring<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Key> PartialKeyed for SelfPartialKeyring<K> {
    type PartialKey = K;
}

impl<K: Key> Keyed for SelfPartialKeyring<K> {
    type Key = Option<K>;
}

impl<K: Key, State: SelfKey<Key=Option<K>>> Keyring<State> for SelfPartialKeyring<K> {
    fn key_for(&self, state: &State) -> Self::Key {
        state.key()
    }
}
