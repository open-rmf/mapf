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
use crate::util::FlatResultMapTrait;

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

impl<Base, Prop> Activity<Base::State> for Incorporated<Base, Prop>
where
    Base: Domain,
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
    <Base as Activity<Base::State>>::Error: Into<<Base as Domain>::Error>,
    Prop: ActionMap<Base::State, <Base as Activity<Base::State>>::Action>,
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
        self.base.choices(from_state)
            .into_iter()
            .flat_map(
                |result| {
                    result
                        .map_err(Into::into)
                        .flat_result_map(|action| {
                            self.prop.map_actions(from_state, action)
                                .into_iter()
                                .map(|r| r
                                    .map_err(Into::into)
                                    .map(Into::into)
                                )
                        })
                        .map(|x| x.flatten())
                }
            )
    }
}

mod test {
    use super::*;
    use crate::error::NoError;
    use thiserror::Error as ThisError;

    struct Count {
        by_interval: Vec<u64>,
    }

    #[derive(Debug, PartialEq, Eq)]
    struct Interval(u64);

    impl Activity<u64> for Count {
        type Action = Interval;
        type Error = NoError;
        type Choices<'a> = impl Iterator<Item=Result<Interval, NoError>> + 'a;

        fn choices<'a>(&'a self, _: &'a u64) -> Self::Choices<'a> {
            self.by_interval.iter().map(|interval| Ok(Interval(*interval)))
        }
    }

    struct Multiplier(u64);
    impl ActionMap<u64, Interval> for Multiplier {
        type ToAction = Interval;
        type Error = NoError;
        type ToActions<'a> = impl IntoIterator<Item=Result<Interval, Self::Error>> + 'a;
        fn map_actions<'a>(
            &'a self,
            _: &'a u64,
            from_action: Interval,
        ) -> Self::ToActions<'a> {
            [Ok(Interval(from_action.0 * self.0))]
        }
    }

    struct DoubleTheOdds;
    impl ActionMap<u64, Interval> for DoubleTheOdds {
        type ToAction = Interval;
        type Error = NoError;
        type ToActions<'a> = impl IntoIterator<Item=Result<Interval, Self::Error>> + 'a;
        fn map_actions<'a>(
            &'a self,
            from_state: &'a u64,
            from_action: Interval,
        ) -> Self::ToActions<'a> {
            [if from_state & 1 != 0 {
                // If the value is odd, double it
                Ok(Interval(from_action.0 * 2))
            } else {
                // If the value is even, leave it alone
                Ok(from_action)
            }]
        }
    }

    #[derive(ThisError, Debug, PartialEq, Eq)]
    #[error("This is a test error")]
    struct TestError;

    struct MapToTestError;
    impl ActionMap<u64, Interval> for MapToTestError {
        type ToAction = Interval;
        type Error = TestError;
        type ToActions<'a> = impl IntoIterator<Item=Result<Interval, Self::Error>> + 'a;
        fn map_actions<'a>(
            &'a self,
            _: &'a u64,
            _: Interval,
        ) -> Self::ToActions<'a> {
            [Err(TestError)]
        }
    }

    #[test]
    fn test_activity_chain_map() {
        let domain = DefineTrait::<u64, Interval>::new()
            .with(Count { by_interval: vec![1, 2, 3]});

        let choices: Result<Vec<_>, _> = domain.choices(&0).collect();
        assert_eq!(choices.unwrap(), vec![Interval(1), Interval(2), Interval(3)]);

        let domain = domain.chain(
            Count { by_interval: vec![10, 20, 30] }
        );
        let choices: Result<Vec<_>, _> = domain.choices(&0).collect();
        assert_eq!(choices.unwrap(), vec![
            Interval(1), Interval(2), Interval(3),
            Interval(10), Interval(20), Interval(30),
        ]);

        let domain = domain.map(Multiplier(2));
        let choices: Result<Vec<_>, _> = domain.choices(&0).collect();
        assert_eq!(choices.unwrap(), vec![
            Interval(2), Interval(4), Interval(6),
            Interval(20), Interval(40), Interval(60),
        ]);

        let domain = domain.map(MapToTestError);
        let choices: Result<Vec<_>, _> = domain.choices(&0).collect();
        assert!(choices.is_err());
    }

    #[test]
    fn test_state_dependent_activity_map() {
        let domain = DefineTrait::<u64, Interval>::new()
            .with(Count { by_interval: vec![2, 3, 4, 5]})
            .map(DoubleTheOdds);

        let choices: Result<Vec<_>, _> = domain.choices(&0).collect();
        assert_eq!(choices.unwrap(), [2, 3, 4, 5].into_iter().map(Interval).collect::<Vec<_>>());

        let choices: Result<Vec<_>, _> = domain.choices(&13).collect();
        assert_eq!(choices.unwrap(), vec![4, 6, 8, 10].into_iter().map(Interval).collect::<Vec<_>>());
    }
}
