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
    fn choices<'a>(&'a self, from_state: State) -> Self::Choices<'a>;
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

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a> {
        self.prop.choices(from_state)
            .into_iter()
            .map(|c| c.map(Into::into).map_err(Into::into))
    }
}

impl<Base, Prop> Activity<Base::State> for Chained<Base, Prop>
where
    Base: Domain + Activity<Base::State>,
    Base::State: Clone,
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

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a> {
        self.base.choices(from_state.clone())
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
    Base::State: Clone,
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

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a> {
        self.base.choices(from_state.clone())
            .into_iter()
            .flat_map(
                move |result| {
                    let from_state = from_state.clone();
                    result
                        .map_err(Into::into)
                        .flat_result_map(move |action| {
                            self.prop.map_actions(from_state.clone(), action)
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

impl<Base, Lifter, Prop> Activity<Base::State> for Lifted<Base, Lifter, Prop>
where
    Base: Domain,
    Lifter: ProjectState<Base::State> + ActionMap<Base::State, Prop::Action, ToAction=Base::Action>,
    Lifter::Error: Into<Base::Error>,
    Lifter::ProjectionError: Into<Base::Error>,
    Lifter::ToAction: Into<Base::Action>,
    Prop: Activity<Lifter::ProjectedState>,
    Base::State: Clone,
    Prop::Error: Into<Base::Error>,
{
    type Action = Lifter::ToAction;
    type Error = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<Self::Action, Self::Error>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::Action: 'a,
        Self::Error: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a> {
        [self.lifter.project(from_state.clone())]
            .into_iter()
            .filter_map(|r: Result<Option<Lifter::ProjectedState>, Lifter::ProjectionError>| r.transpose())
            .flat_map(move |r: Result<Lifter::ProjectedState, Lifter::ProjectionError>| {
                let from_state = from_state.clone();
                r
                .map_err(Into::into)
                .flat_result_map(move |projected_state| {
                    let from_state = from_state.clone();
                    self.prop.choices(projected_state)
                        .into_iter()
                        .flat_map(move |r: Result<Prop::Action, Prop::Error>| {
                            let from_state = from_state.clone();
                            r
                            .map_err(Into::into)
                            .flat_result_map(move |action| {
                                self.lifter.map_actions(from_state.clone(), action)
                                    .into_iter()
                                    .map(|r| r
                                        .map_err(Into::into)
                                        .map(Into::into)
                                    )
                            })
                            .map(|x: Result<Result<Self::Action, Self::Error>, Self::Error>| x.flatten())
                        })
                })
                .map(|x: Result<Result<Self::Action, Self::Error>, Self::Error>| x.flatten())
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::NoError;
    use thiserror::Error as ThisError;
    use std::collections::HashSet;

    struct Count {
        by_interval: Vec<u64>,
    }

    #[derive(Debug, PartialEq, Eq)]
    struct Interval(u64);

    impl Activity<u64> for Count {
        type Action = Interval;
        type Error = NoError;
        type Choices<'a> = impl Iterator<Item=Result<Interval, NoError>> + 'a;

        fn choices<'a>(&'a self, _: u64) -> Self::Choices<'a> {
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
            _: u64,
            from_action: Interval,
        ) -> Self::ToActions<'a>
        where
            Interval: 'a,
        {
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
            from_state: u64,
            from_action: Interval,
        ) -> Self::ToActions<'a>
        where
            Interval: 'a,
        {
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
            _: u64,
            _: Interval,
        ) -> Self::ToActions<'a>
        where
            Interval: 'a,
        {
            [Err(TestError)]
        }
    }

    #[test]
    fn test_activity_chain_map() {
        let domain = DefineTrait::<u64, Interval>::new()
            .with(Count { by_interval: vec![1, 2, 3]});

        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(choices.unwrap(), vec![Interval(1), Interval(2), Interval(3)]);

        let domain = domain.chain(
            Count { by_interval: vec![10, 20, 30] }
        );
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(choices.unwrap(), vec![
            Interval(1), Interval(2), Interval(3),
            Interval(10), Interval(20), Interval(30),
        ]);

        let domain = domain.map(Multiplier(2));
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(choices.unwrap(), vec![
            Interval(2), Interval(4), Interval(6),
            Interval(20), Interval(40), Interval(60),
        ]);

        let domain = domain.map(MapToTestError);
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert!(choices.is_err());
    }

    #[test]
    fn test_state_dependent_activity_map() {
        let domain = DefineTrait::<u64, Interval>::new()
            .with(Count { by_interval: vec![2, 3, 4, 5]})
            .map(DoubleTheOdds);

        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(choices.unwrap(), [2, 3, 4, 5].into_iter().map(Interval).collect::<Vec<_>>());

        let choices: Result<Vec<_>, _> = domain.choices(13).collect();
        assert_eq!(choices.unwrap(), vec![4, 6, 8, 10].into_iter().map(Interval).collect::<Vec<_>>());
    }

    #[derive(Clone, Copy)]
    struct Inventory {
        apples: u64,
        bananas: u64,
        budget: u64,
    }

    #[derive(Clone, Copy)]
    struct Item {
        count: u64,
        budget: u64,
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct Buy(u64 /* price per unit */);
    impl Activity<Item> for Buy {
        type Action = Buy;
        type Error = NoError;
        type Choices<'a> = Option<Result<Buy, NoError>>;
        fn choices<'a>(&'a self, from_state: Item) -> Option<Result<Buy, NoError>> {
            if from_state.budget < self.0 {
                None
            } else {
                Some(Ok(*self))
            }
        }
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct Sell(u64 /* price per unit */);
    impl Activity<Item> for Sell {
        type Action = Sell;
        type Error = NoError;
        type Choices<'a> = Option<Result<Sell, NoError>>;
        fn choices<'a>(&'a self, from_state: Item) -> Self::Choices<'a> {
            if from_state.count <= 0 {
                None
            } else {
                Some(Ok(*self))
            }
        }
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    enum Transaction {
        Buy(Buy),
        Sell(Sell),
    }
    impl From<Buy> for Transaction {
        fn from(value: Buy) -> Self {
            Transaction::Buy(value)
        }
    }
    impl From<Sell> for Transaction {
        fn from(value: Sell) -> Self {
            Transaction::Sell(value)
        }
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    enum Order {
        Apples(Transaction),
        Bananas(Transaction),
    }

    struct JustApples;
    impl StateSubspace for JustApples {
        type ProjectedState = Item;
    }
    impl ProjectState<Inventory> for JustApples {
        type ProjectionError = NoError;
        fn project(
            &self,
            state: Inventory,
        ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
            Ok(Some(Item {
                count: state.apples,
                budget: state.budget,
            }))
        }
    }
    impl ActionMap<Inventory, Transaction> for JustApples {
        type Error = NoError;
        type ToAction = Order;
        type ToActions<'a> = Option<Result<Order, NoError>>;
        fn map_actions<'a>(
            &'a self,
            _: Inventory,
            from_action: Transaction,
        ) -> Self::ToActions<'a>
        where
            Transaction: 'a,
            Inventory: 'a
        {
            Some(Ok(Order::Apples(from_action)))
        }
    }

    struct JustBananas;
    impl StateSubspace for JustBananas {
        type ProjectedState = Item;
    }
    impl ProjectState<Inventory> for JustBananas {
        type ProjectionError = NoError;
        fn project(
            &self,
            state: Inventory
        ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
            Ok(Some(Item {
                count: state.bananas,
                budget: state.budget,
            }))
        }
    }
    impl ActionMap<Inventory, Transaction> for JustBananas {
        type Error = NoError;
        type ToAction = Order;
        type ToActions<'a> = Option<Result<Order, NoError>>;
        fn map_actions<'a>(
            &'a self,
            _: Inventory,
            from_action: Transaction,
        ) -> Self::ToActions<'a>
        where
            Transaction: 'a,
            Inventory: 'a
        {
            Some(Ok(Order::Bananas(from_action)))
        }
    }

    #[test]
    fn test_lifted_activity() {
        let domain = DefineTrait::<Inventory, Order>::new()
            .lift(
                JustApples,
                DefineTrait::<Item, Transaction>::new()
                    .with(Buy(20))
                    .chain(Sell(60))
            )
            .chain_lift(
                JustBananas,
                DefineTrait::<Item, Transaction>::new()
                    .with(Buy(30))
                    .chain(Sell(80))
            );

        let inventory = Inventory {
            apples: 5,
            bananas: 3,
            budget: 25,
        };
        let choices: Result<HashSet<_>, _> = domain.choices(inventory).collect();
        let choices = choices.unwrap();
        assert_eq!(choices.len(), 3);
        assert!(choices.contains(&Order::Apples(Transaction::Buy(Buy(20)))));
        assert!(choices.contains(&Order::Apples(Transaction::Sell(Sell(60)))));
        assert!(choices.contains(&Order::Bananas(Transaction::Sell(Sell(80)))));

        let inventory = Inventory {
            apples: 0,
            bananas: 3,
            budget: 40,
        };
        let choices: Result<HashSet<_>, _> = domain.choices(inventory).collect();
        let choices = choices.unwrap();
        assert_eq!(choices.len(), 3);
        assert!(choices.contains(&Order::Apples(Transaction::Buy(Buy(20)))));
        assert!(choices.contains(&Order::Bananas(Transaction::Buy(Buy(30)))));
        assert!(choices.contains(&Order::Bananas(Transaction::Sell(Sell(80)))));
    }
}
