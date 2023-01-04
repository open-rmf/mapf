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

/// The Activity trait describes an activity that can be performed within a
/// domain. An activity yields action choices for an agent where those choices
/// may depend on the state of the agent. These action choices can be examined
/// by a planner to discover a valid or optimal sequence of actions to reach a
/// goal.
///
/// In graph theory terms, an action an edge of a graph. An Activity is a
/// function that maps a graph vertex (agent state) to the outgoing edges from
/// that vertex.
pub trait Activity<State> {
    /// What kind of action is produced by this activity
    type Action;

    /// What kind of error can happen if a bad state is provided
    type Error;

    /// Concrete type for the returned container of choices
    type Choices<'a>: IntoIterator<Item = Result<Self::Action, Self::Error>>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::Error: 'a,
        State: 'a;

    /// What choices can be made related to this activity from the provided state
    fn choices<'a>(&'a self, from_state: &'a State) -> Self::Choices<'a>;
}

trait NoActivity { }
impl<S, A, E> NoActivity for DefineDomain<S, A, E> { }

impl<Base, Prop> Activity<Base::State> for Incorporated<Base, Prop>
where
    Base: Domain + NoActivity,
    Prop: Activity<Base::State>,
    Prop::Action: Into<Base::Action>,
    Prop::Error: Into<Base::Error>,
{
    type Action = Base::Action;
    type Error = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<Self::Action, Self::Error>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::Action: 'a,
        Self::Error: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: &'a Base::State) -> Self::Choices<'a> {
        self.prop.choices(from_state)
            .into_iter()
            .map(|c| c.map(Into::into).map_err(Into::into))
    }
}

impl<Base, Prop> Activity<Base::State> for Chained<Base, Prop>
where
    Base: Domain + Activity<Base::State>,
    <Base as Activity<Base::State>>::Action: Into<<Base as Domain>::Action>,
    <Base as Activity<Base::State>>::Error: Into<<Base as Domain>::Error>,
    Prop: Activity<Base::State>,
    Prop::Action: Into<<Base as Domain>::Action>,
    Prop::Error: Into<<Base as Domain>::Error>,
{
    type Action = <Base as Domain>::Action;
    type Error = <Base as Domain>::Error;
    type Choices<'a> = impl Iterator<Item=Result<Self::Action, Self::Error>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::Action: 'a,
        Self::Error: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: &'a Base::State) -> Self::Choices<'a> {
        self.base.choices(from_state)
            .into_iter()
            .map(|c| c.map(Into::into).map_err(Into::into))
            .chain(
                self.prop.choices(from_state).into_iter()
                .map(|c| c.map(Into::into).map_err(Into::into))
            )
    }
}

impl<Base, Prop> Activity<Base::State> for Mapped<Base, Prop>
where
    Base: Domain + Activity<Base::State>,
    Prop: ActionMap<
        Base::State,
        <Base as Activity<Base::State>>::Action,
        <Base as Activity<Base::State>>::Error,
    >,
    Prop::ToAction: Into<<Base as Domain>::Action>,
    Prop::Error: Into<<Base as Domain>::Error>,
{
    type Action = <Base as Domain>::Action;
    type Error = <Base as Domain>::Error;
    type Choices<'a> = impl Iterator<Item=Result<Self::Action, Self::Error>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::Action: 'a,
        Self::Error: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: &'a Base::State) -> Self::Choices<'a> {
        self.prop.map_actions(from_state, self.base.choices(from_state))
            .into_iter()
            .map(|a| a.map(Into::into).map_err(Into::into))
    }
}
