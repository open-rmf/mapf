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

    /// Project a state down to the target state space.
    fn project(
        &self,
        state: &State
    ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError>;
}

pub trait LiftState<State>: StateSubspace {
    /// What kind of error can happen if a bad state is provided
    type LiftError;

    /// Lift a projected state back to the original state space. An original
    /// state is provided for reference, to fill in information that might be
    /// otherwise missing.
    fn lift(
        &self,
        original: &State,
        projection: Self::ProjectedState
    ) -> Result<Option<State>, Self::LiftError>;
}

/// The StateMap trait describes a domain property that can project the state of
/// a domain to another state space and then lift it back to the domain's state
/// space.
pub trait StateMap<State>: ProjectState<State> + LiftState<State> {}
impl<State, T: ProjectState<State> + LiftState<State>> StateMap<State> for T {}

/// NoStateSubspace is a struct that provides a no-op implementation of
/// StateSubspace, ProjectState, and LiftState. Used by DomainMap when an
/// ActionMap is not needed.
pub struct NoStateSubspace<State> {
    _ignore: std::marker::PhantomData<State>,
}
impl<State> NoStateSubspace<State> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}
impl<State> StateSubspace for NoStateSubspace<State> {
    type ProjectedState = State;
}
impl<State: Clone> ProjectState<State> for NoStateSubspace<State> {
    type ProjectionError = NoError;
    fn project(&self, state: &State) -> Result<Option<State>, NoError> {
        Ok(Some(state.clone()))
    }
}
impl<State> LiftState<State> for NoStateSubspace<State> {
    type LiftError = NoError;
    fn lift(&self, _: &State, projection: State) -> Result<Option<State>, NoError> {
        Ok(Some(projection))
    }
}

/// StateInto implements state subspace traits for state subspaces that can
/// be mapped using Into
#[derive(Debug)]
pub struct StateInto<ProjectedState> {
    _ignore: std::marker::PhantomData<ProjectedState>,
}
impl<ProjectedState> StateInto<ProjectedState> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}
impl<ProjectedState> StateSubspace for StateInto<ProjectedState> {
    type ProjectedState = ProjectedState;
}
impl<State: Clone + Into<ProjectedState>, ProjectedState> ProjectState<State> for StateInto<ProjectedState> {
    type ProjectionError = NoError;
    fn project(&self, state: &State) -> Result<Option<ProjectedState>, NoError> {
        Ok(Some(state.clone().into()))
    }
}
impl<State, ProjectedState: Into<State>> LiftState<State> for StateInto<ProjectedState> {
    type LiftError = NoError;
    fn lift(&self, _: &State, projection: Self::ProjectedState) -> Result<Option<State>, NoError> {
        Ok(Some(projection.into()))
    }
}
impl<ProjectedState> Clone for StateInto<ProjectedState> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

/// StateMaybeInto implements state subspace traits for state subspaces that can
/// be mapped using Into
#[derive(Debug)]
pub struct StateMaybeInto<ProjectedState> {
    _ignore: std::marker::PhantomData<ProjectedState>,
}
impl<ProjectedState> StateMaybeInto<ProjectedState> {
    pub fn new() -> Self {
        Self { _ignore: Default::default() }
    }
}
impl<ProjectedState> StateSubspace for StateMaybeInto<ProjectedState> {
    type ProjectedState = ProjectedState;
}
impl<State, ProjectedState> ProjectState<State> for StateMaybeInto<ProjectedState>
where
    State: Clone + Into<Option<ProjectedState>>,
{
    type ProjectionError = NoError;
    fn project(&self, state: &State) -> Result<Option<ProjectedState>, NoError> {
        Ok(state.clone().into())
    }
}
impl<State, ProjectedState> LiftState<State> for StateMaybeInto<ProjectedState>
where
    ProjectedState: Into<Option<State>>,
{
    type LiftError = NoError;
    fn lift(&self, _: &State, projection: Self::ProjectedState) -> Result<Option<State>, NoError> {
        Ok(projection.into())
    }
}
impl<ProjectedState> Clone for StateMaybeInto<ProjectedState> {
    fn clone(&self) -> Self {
        Self::new()
    }
}
