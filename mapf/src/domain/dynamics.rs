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

/// The Dynamics trait describes how a state gets transformed by an action.
///
/// In graph theory terms, this produces the next vertex of a graph, given the
/// current vertex and an outgoing edge.
///
/// Note that when you chain Dynamics properties, the implementation of advance
/// will be called on each chained property in the order that they are chained
/// in. When chaining Dynamics properties, make sure the implementations are not
/// doubling up their effects on the state.
pub trait Dynamics<State, Action> {
    /// What kind of error can happen if a bad state and/or action is provided
    type Error;

    /// Advance the given state based on the given action.
    fn advance(&self, state: State, action: &Action) -> Result<State, Self::Error>;
}

trait NoDynamics { }
impl<S, A, E> NoDynamics for DefineDomain<S, A, E> { }

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Incorporated<Base, Prop>
where
    Base: Domain + NoDynamics,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::Error: Into<Base::Error>,
{
    type Error = Base::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Base::State, Self::Error> {
        self.prop.advance(state, action).map_err(Into::into)
    }
}

impl<Base, Prop> Dynamics<Base::State, Base::Action> for Chained<Base, Prop>
where
    Base: Domain + Dynamics<Base::State, Base::Action>,
    <Base as Dynamics<Base::State, Base::Action>>::Error: Into<<Base as Domain>::Error>,
    Prop: Dynamics<Base::State, Base::Action>,
    Prop::Error: Into<<Base as Domain>::Error>,
{
    type Error = <Base as Domain>::Error;
    fn advance(&self, state: Base::State, action: &Base::Action) -> Result<Base::State, Self::Error> {
        // NOTE: The base dynamics are being applied before the chained prop dynamics
        self.base.advance(state, action)
            .map_err(Into::into)
            .and_then(|state| self.prop.advance(state, action).map_err(Into::into))
    }
}
