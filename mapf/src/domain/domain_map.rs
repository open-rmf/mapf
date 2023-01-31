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

pub struct DomainMap<StateMapImpl, ActionMapImpl> {
    state_map: StateMapImpl,
    action_map: ActionMapImpl,
}


pub struct DefineDomainMap<State=()>(std::marker::PhantomData<State>);

impl<State> DefineDomainMap<State> {
    pub fn for_actions<ActionMapImpl>(
        action_map: ActionMapImpl,
    ) -> DomainMap<NoStateSubspace<State>, ActionMapImpl> {
        DomainMap { state_map: NoStateSubspace::new(), action_map }
    }
}

impl DefineDomainMap<()> {
    pub fn with<StateMapImpl, ActionMapImpl>(
        state_map: StateMapImpl,
        action_map: ActionMapImpl,
    ) -> DomainMap<StateMapImpl, ActionMapImpl> {
        DomainMap { state_map, action_map }
    }

    pub fn for_subspace<StateMapImpl>(
        state_map: StateMapImpl,
    ) -> DomainMap<StateMapImpl, NoActionMap> {
        DomainMap { state_map, action_map: NoActionMap }
    }
}

impl<S: StateSubspace, A> StateSubspace for DomainMap<S, A> {
    type ProjectedState = S::ProjectedState;
}

impl<State, S: ProjectState<State>, A> ProjectState<State> for DomainMap<S, A> {
    type ProjectionError = S::ProjectionError;
    fn project(
        &self,
        state: State
    ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
        self.state_map.project(state)
    }
}

impl<State, S: LiftState<State>, A> LiftState<State> for DomainMap<S, A> {
    type LiftError = S::LiftError;
    fn lift(
        &self,
        original: State,
        projection: Self::ProjectedState
    ) -> Result<Option<State>, Self::LiftError> {
        self.state_map.lift(original, projection)
    }
}

impl<State, FromAction, S, A> ActionMap<State, FromAction> for DomainMap<S, A>
where
    S: StateSubspace,
    A: ActionMap<State, FromAction>,
{
    type ToAction = A::ToAction;
    type ActionMapError = A::ActionMapError;
    type ToActions<'a> = A::ToActions<'a>
    where
        Self: 'a,
        Self::ToAction: 'a,
        Self::ActionMapError: 'a,
        State: 'a,
        FromAction: 'a;

    fn map_actions<'a>(
        &'a self,
        from_state: State,
        from_action: FromAction,
    ) -> Self::ToActions<'a>
    where
        FromAction: 'a
    {
        self.action_map.map_actions(from_state, from_action)
    }
}
