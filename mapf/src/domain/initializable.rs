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
use crate::error::NoError;

pub trait Initializable<Start, Goal, State> {
    /// What kind of error can happen if a bad Start value is provided
    type InitialError;

    type InitialStates<'a>: IntoIterator<Item = Result<State, Self::InitialError>>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: Start, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a;
}

// An empty tuple implements Initializable by simply accepting an initial state.
impl<Start: Into<State>, Goal, State> Initializable<Start, Goal, State> for () {
    type InitialError = NoError;
    type InitialStates<'a> = [Result<State, NoError>; 1]
    where
        Start: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: Start, _to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Start: 'a,
        Goal: 'a,
        State: 'a,
    {
        [Ok(from_start.into())]
    }
}

/// This struct implements Initializable for any Start that implements Into<State>
pub struct InitFrom;

impl<Start: Into<State>, Goal, State> Initializable<Start, Goal, State> for InitFrom {
    type InitialError = NoError;
    type InitialStates<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        Goal: 'a,
        Start: 'a;

    fn initialize<'a>(&'a self, from_start: Start, _to_goal: &Goal) -> Self::InitialStates<'a>
    where
        State: 'a,
        Goal: 'a,
        Start: 'a,
    {
        Some(Ok(from_start.into()))
    }
}

/// This struct implements [`Initializable`] for any Start that implements `Into<Option<State>>`
pub struct MaybeInitFrom;

impl<Start: Into<Option<State>>, Goal, State> Initializable<Start, Goal, State> for MaybeInitFrom {
    type InitialError = NoError;
    type InitialStates<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        Goal: 'a,
        Start: 'a;

    fn initialize<'a>(&'a self, from_start: Start, _to_goal: &Goal) -> Self::InitialStates<'a>
    where
        State: 'a,
        Goal: 'a,
        Start: 'a,
    {
        from_start.into().map(|s| Ok(s))
    }
}

/// Use this struct to allow your domain to take in an iterator of Starts
/// instead of only one Start.
pub struct ManyInit<Init>(pub Init);

impl<StartIter, Goal, State, Init> Initializable<StartIter, Goal, State> for ManyInit<Init>
where
    StartIter: IntoIterator,
    Init: Initializable<StartIter::Item, Goal, State>,
    Goal: Clone,
{
    type InitialError = Init::InitialError;
    type InitialStates<'a> = impl Iterator<Item = Result<State, Self::InitialError>> + 'a
    where
        Self: 'a,
        Self::InitialError: 'a,
        StartIter: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: StartIter, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        StartIter: 'a,
        Goal: 'a,
        State: 'a,
    {
        let to_goal = to_goal.clone();
        from_start
            .into_iter()
            .flat_map(move |start| self.0.initialize(start, &to_goal))
    }
}

pub struct LiftInit<Init>(pub Init);

impl<Start, Goal, State, Init> Initializable<Start, Goal, State> for LiftInit<Init>
where
    Init: Domain,
    Init::State: Into<State>,
    Init: Initializable<Start, Goal, State>,
{
    type InitialError = Init::InitialError;
    type InitialStates<'a> = impl Iterator<Item = Result<State, Self::InitialError>> + 'a
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a;

    fn initialize<'a>(&'a self, from_start: Start, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Self::InitialError: 'a,
        Start: 'a,
        Goal: 'a,
        State: 'a,
    {
        self.0
            .initialize(from_start, to_goal)
            .into_iter()
            .map(|s| s.into())
    }
}

impl<Base, Prop, Start, Goal> Initializable<Start, Goal, Base::State> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Initializable<Start, Goal, Base::State>,
    Prop::InitialError: Into<Base::Error>,
{
    type InitialError = Base::Error;
    type InitialStates<'a> = impl Iterator<Item=Result<Base::State, Base::Error>> + 'a
    where
        Self: 'a,
        Base::Error: 'a,
        Base::State: 'a,
        Start: 'a,
        Goal: 'a;

    fn initialize<'a>(&'a self, from_start: Start, to_goal: &Goal) -> Self::InitialStates<'a>
    where
        Self: 'a,
        Base::Error: 'a,
        Base::State: 'a,
        Start: 'a,
        Goal: 'a,
    {
        self.prop
            .initialize(from_start, to_goal)
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
        OffGraph(Point),
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
        let domain = DefineTrait::<TestState>::new().with(InitFrom);

        let initial_state: Result<Vec<_>, _> = domain.initialize(5, &()).into_iter().collect();
        let initial_state = initial_state.unwrap();
        assert!(initial_state.len() == 1);
        assert_eq!(initial_state[0], TestState::OnGraph(5));
    }

    #[test]
    fn test_multi_start() {
        let domain = DefineTrait::<TestState>::new().with(ManyInit(InitFrom));

        let initial_states: Result<Vec<_>, _> = domain
            .initialize([Point::new(0.1, 0.2), Point::new(3.0, 4.0)], &())
            .into_iter()
            .collect();
        let initial_states = initial_states.unwrap();
        assert!(initial_states.len() == 2);
    }
}
