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

use crate::error::NoError;

pub trait StateSubspace {
    type ProjectedState;
}

pub trait ProjectState<State>: StateSubspace {
    /// What kind of error can happen if a bad state is provided
    type ProjectionError;

    type Projections<'a>: IntoIterator<Item = Result<Self::ProjectedState, Self::ProjectionError>>
    where
        Self: 'a,
        Self::ProjectedState: 'a,
        Self::ProjectionError: 'a,
        State: 'a;

    /// Project a state down to the target state space.
    fn project<'a>(&'a self, state: State) -> Self::Projections<'a>
    where
        State: 'a;
}

pub trait LiftState<State>: StateSubspace {
    /// What kind of error can happen if a bad state is provided
    type LiftError;

    type States<'a>: IntoIterator<Item = Result<State, Self::LiftError>>
    where
        Self: 'a,
        Self::ProjectedState: 'a,
        Self::LiftError: 'a,
        State: 'a;

    /// Lift a projected state back to the original state space. An original
    /// state is provided for reference, to fill in information that might be
    /// otherwise missing.
    fn lift<'a>(&'a self, original: State, projection: Self::ProjectedState) -> Self::States<'a>
    where
        State: 'a;
}

/// The StateMap trait describes a domain property that can project the state of
/// a domain to another state space and then lift it back to the domain's state
/// space.
pub trait StateMap<State>: ProjectState<State> + LiftState<State> {}
impl<State, T: ProjectState<State> + LiftState<State>> StateMap<State> for T {}

/// NoStateSubspace is a trait that can be attached to structs that are
/// required to implement ProjectState<State> or LiftState<State but do not
/// actually have a subspace.
pub trait NoStateSubspace {
    type State;
}
impl<T: NoStateSubspace> StateSubspace for T {
    type ProjectedState = T::State;
}
impl<T: NoStateSubspace> ProjectState<T::State> for T {
    type ProjectionError = NoError;
    type Projections<'a> = Option<Result<T::State, NoError>>
    where
        T: 'a,
        T::State: 'a,
        <T as StateSubspace>::ProjectedState: 'a;

    fn project<'a>(&'a self, state: T::State) -> Self::Projections<'a>
    where
        T::State: 'a,
    {
        Some(Ok(state))
    }
}
impl<T: NoStateSubspace> LiftState<T::State> for T {
    type LiftError = NoError;
    type States<'a> = Option<Result<T::State, NoError>>
    where
        T: 'a,
        T::State: 'a,
        <T as StateSubspace>::ProjectedState: 'a;

    fn lift<'a>(&'a self, _: T::State, projection: T::State) -> Self::States<'a>
    where
        T::State: 'a,
    {
        Some(Ok(projection))
    }
}

/// StateInto implements state subspace traits for state subspaces that can
/// be mapped using Into
pub struct StateInto<ProjectedState> {
    _ignore: std::marker::PhantomData<ProjectedState>,
}
impl<ProjectedState> StateSubspace for StateInto<ProjectedState> {
    type ProjectedState = ProjectedState;
}
impl<State, ProjectedState> ProjectState<State> for StateInto<ProjectedState>
where
    State: Into<ProjectedState>,
{
    type ProjectionError = NoError;
    type Projections<'a> = Option<Result<ProjectedState, NoError>>
    where
        State: 'a,
        ProjectedState: 'a;

    fn project<'a>(&'a self, state: State) -> Self::Projections<'a>
    where
        State: 'a,
    {
        Some(Ok(state.into()))
    }
}
impl<State, ProjectedState> LiftState<State> for StateInto<ProjectedState>
where
    ProjectedState: Into<State>,
{
    type LiftError = NoError;
    type States<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        ProjectedState: 'a;

    fn lift<'a>(&'a self, _: State, projection: Self::ProjectedState) -> Self::States<'a>
    where
        State: 'a
    {
        Some(Ok(projection.into()))
    }
}

/// StateMaybeInto implements state subspace traits for state subspaces that can
/// be mapped using Into
pub struct StateMaybeInto<ProjectedState> {
    _ignore: std::marker::PhantomData<ProjectedState>,
}
impl<ProjectedState> StateSubspace for StateMaybeInto<ProjectedState> {
    type ProjectedState = ProjectedState;
}
impl<State, ProjectedState> ProjectState<State> for StateMaybeInto<ProjectedState>
where
    State: Into<Option<ProjectedState>>,
{
    type ProjectionError = NoError;
    type Projections<'a> = Option<Result<ProjectedState, NoError>>
    where
        State: 'a,
        ProjectedState: 'a;

    fn project<'a>(&'a self, state: State) -> Self::Projections<'a>
    where
        State: 'a,
    {
        Ok(state.into()).transpose()
    }
}
impl<State, ProjectedState> LiftState<State> for StateMaybeInto<ProjectedState>
where
    ProjectedState: Into<Option<State>>,
{
    type LiftError = NoError;
    type States<'a> = Option<Result<State, NoError>>
    where
        State: 'a,
        ProjectedState: 'a;

    fn lift<'a>(&'a self, _: State, projection: Self::ProjectedState) -> Self::States<'a>
    where
        State: 'a,
    {
        Ok(projection.into()).transpose()
    }
}
