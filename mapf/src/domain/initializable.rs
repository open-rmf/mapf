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

use super::prelude::*;
use crate::error::NoError;

pub trait Initializable<Start, State> {
    /// What kind of error can happen if a bad Start value is provided
    type InitialError;

    type InitialStates<'a>: IntoIterator<Item = Result<State, Self::InitialError>>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        State: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        State: 'a;
}

/// This struct implements Initializable for any Start that implements Into<State>
pub struct InitFrom<Start> {
    _ignore: std::marker::PhantomData<Start>,
}

impl<Start> InitFrom<Start> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}

impl<State, Start: Into<State>> Initializable<Start, State> for InitFrom<Start> {
    type InitialError = NoError;
    type InitialStates<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        Start: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        State: 'a,
        Start: 'a,
    {
        Some(Ok(from_start.into()))
    }
}

/// This struct implements Initializable for any Start that implements Into<Option<State>>
pub struct InitFromMaybe<Start> {
    _ignore: std::marker::PhantomData<Start>,
}

impl<State> InitFromMaybe<State> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}

impl<State, Start: Into<Option<State>>> Initializable<Start, State> for InitFromMaybe<Start> {
    type InitialError = NoError;
    type InitialStates<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        Start: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        State: 'a,
        Start: 'a
    {
        from_start.into().map(|s| Ok(s))
    }
}

/// Use this struct to allow your domain to take in an iterator of Starts
/// instead of only one Start.
pub struct ManyInit<Init>(pub Init);

impl<State, StartIter, Init> Initializable<StartIter, State> for ManyInit<Init>
where
    StartIter: IntoIterator,
    Init: Initializable<StartIter::Item, State>,
{
    type InitialError = Init::InitialError;
    type InitialStates<'a> = impl Iterator<Item = Result<State, Self::InitialError>> + 'a
    where
        Self: 'a,
        Self::InitialError: 'a,
        StartIter: 'a,
        State: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: StartIter,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        StartIter: 'a,
        State: 'a,
    {
        from_start.into_iter().flat_map(|start| self.0.initialize(start))
    }
}

impl<Base, Prop, Start> Initializable<Start, Base::State> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Initializable<Start, Base::State>,
    Prop::InitialError: Into<Base::Error>,
{
    type InitialError = Base::Error;
    type InitialStates<'a> = impl Iterator<Item=Result<Base::State, Base::Error>> + 'a
    where
        Self: 'a,
        Base::Error: 'a,
        Base::State: 'a,
        Start: 'a;

    fn initialize<'a>(
        &'a self,
        from_start: Start,
    ) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Base::Error: 'a,
        Base::State: 'a,
        Start: 'a,
    {
        self.prop
            .initialize(from_start)
            .into_iter()
            .map(|r| r.map(Into::into).map_err(Into::into))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Point = nalgebra::Point2<f64>;

    #[derive(Debug, PartialEq)]
    enum TestState {
        OnGraph(usize),
        OffGraph(Point)
    }
    impl From<usize> for TestState {
        fn from(value: usize) -> Self {
            TestState::OnGraph(value)
        }
    }
    impl From<Point> for TestState {
        fn from(value: Point) -> Self {
            TestState::OffGraph(value)
        }
    }

    #[test]
    fn test_single_start() {
        let domain = DefineTrait::<TestState, ()>::new()
            .with(InitFrom::<usize>::new());

        let initial_state: Result<Vec<_>, _> = domain.initialize(5).into_iter().collect();
        let initial_state = initial_state.unwrap();
        assert!(initial_state.len() == 1);
        assert_eq!(initial_state[0], TestState::OnGraph(5));
    }

    #[test]
    fn test_multi_start() {
        let domain = DefineTrait::<TestState, ()>::new()
            .with(ManyInit(InitFrom::<Point>::new()));

        let initial_states: Result<Vec<_>, _> = domain.initialize(
            [Point::new(0.1, 0.2), Point::new(3.0, 4.0)]
        ).into_iter().collect();
        let initial_states = initial_states.unwrap();
        assert!(initial_states.len() == 2);
    }
}

