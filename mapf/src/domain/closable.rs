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

use super::*;

pub trait Closable<State, T> {
    type ClosedSet: ClosedSet<State, T>;
    fn new_closed_set(&self) -> Self::ClosedSet;
}

pub enum CloseResult<'a, T> {
    /// The state was successfully closed. This state was never previously
    /// closed.
    Closed,

    /// The state was previously closed. See a reference to the previously
    /// closed value.
    Prior(&'a T),
}

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
}

pub trait ClosedSet<State, T> {
    /// Close a state with a value. If the state was already closed with a prior
    /// value, it will retain its original value.
    fn close<'a>(&'a mut self, state: State, value: T) -> CloseResult<'a, T>;

    /// Replace a state's value. It will take on the new value regardless of
    /// whether the state was already closed.
    fn replace(&mut self, state: State, value: T) -> Option<T>;

    /// Get the status of the specified state.
    fn status<'a>(&'a self, state: &State) -> ClosedStatus<'a, T>;
}

