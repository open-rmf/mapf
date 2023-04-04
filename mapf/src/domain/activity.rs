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
use crate::{error::NoError, util::FlatResultMapTrait};

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
    type ActivityAction;

    /// What kind of error can happen if a bad state is provided
    type ActivityError;

    /// Concrete type for the returned container of choices
    type Choices<'a>: IntoIterator<Item = Result<(Self::ActivityAction, State), Self::ActivityError>>
        + 'a
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        State: 'a;

    /// What choices can be made related to this activity from the provided state
    // TODO(@mxgrey): Investigate whether `from_state` can have a `&State` type
    fn choices<'a>(&'a self, from_state: State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        State: 'a;
}

/// [`NoActivity`] can be used as a placeholder where an Activity is required
/// but it doesn't need to do anything.
pub struct NoActivity<A>(std::marker::PhantomData<A>);
impl<State, A> Activity<State> for NoActivity<A> {
    type ActivityAction = A;
    type ActivityError = NoError;
    type Choices<'a> = [Result<(A, State), NoError>; 0]
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        State: 'a;

    fn choices<'a>(&'a self, _: State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        State: 'a,
    {
        []
    }
}

/// The ActivityModifier can be used to change the behaviors of an activity,
/// e.g. by enforcing constraints.
pub trait ActivityModifier<State, FromAction> {
    /// What kind of action can be output by this modifier
    type ModifiedAction;

    /// What kind of error can happen for this modifier
    type ModifiedActionError;

    type ModifiedChoices<'a>: IntoIterator<
        Item = Result<(Self::ModifiedAction, State), Self::ModifiedActionError>,
    >
    where
        Self: 'a,
        Self::ModifiedAction: 'a,
        Self::ModifiedActionError: 'a,
        State: 'a,
        FromAction: 'a;

    fn modify_action<'a>(
        &'a self,
        from_state: State,
        from_action: FromAction,
        to_state: State,
    ) -> Self::ModifiedChoices<'a>
    where
        FromAction: 'a,
        State: 'a;
}

// Allow an empty tuple to implement the ActivityModifier trait by not modifying
// the action.
impl<State, FromAction> ActivityModifier<State, FromAction> for () {
    type ModifiedAction = FromAction;
    type ModifiedActionError = NoError;
    type ModifiedChoices<'a> = [Result<(FromAction, State), NoError>; 1]
    where
        FromAction: 'a,
        State: 'a;

    fn modify_action<'a>(
        &'a self,
        _: State,
        from_action: FromAction,
        to_state: State,
    ) -> Self::ModifiedChoices<'a>
    where
        FromAction: 'a,
        State: 'a,
    {
        [Ok((from_action, to_state))]
    }
}

impl<Base, Prop> Activity<Base::State> for Incorporated<Base, Prop>
where
    Base: Domain,
    Prop: Activity<Base::State>,
    Prop::ActivityError: Into<Base::Error>,
{
    type ActivityAction = Prop::ActivityAction;
    type ActivityError = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<(Self::ActivityAction, Base::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a,
    {
        self.prop
            .choices(from_state)
            .into_iter()
            .map(|c| c.map(|(a, s)| (a.into(), s.into())).map_err(Into::into))
    }
}

impl<Base, Prop> Activity<Base::State> for Chained<Base, Prop>
where
    Base: Domain + Activity<Base::State>,
    Base::State: Clone,
    Base::ActivityAction: Into<Prop::ActivityAction>,
    Base::ActivityError: Into<Base::Error>,
    Prop: Activity<Base::State>,
    Prop::ActivityError: Into<Base::Error>,
{
    type ActivityAction = Prop::ActivityAction;
    type ActivityError = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<(Self::ActivityAction, Base::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a,
    {
        self.base
            .choices(from_state.clone())
            .into_iter()
            .map(|c| c.map(|(a, s)| (a.into(), s.into())).map_err(Into::into))
            .chain(
                self.prop
                    .choices(from_state)
                    .into_iter()
                    .map(|c| c.map(|(a, s)| (a.into(), s.into())).map_err(Into::into)),
            )
    }
}

impl<Base, Prop> Activity<Base::State> for Mapped<Base, Prop>
where
    Base: Domain + Activity<Base::State>,
    Base::State: Clone,
    Base::ActivityError: Into<Base::Error>,
    Prop: ActivityModifier<Base::State, Base::ActivityAction>,
    Prop::ModifiedActionError: Into<Base::Error>,
{
    type ActivityAction = Prop::ModifiedAction;
    type ActivityError = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<(Self::ActivityAction, Base::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a,
    {
        self.base
            .choices(from_state.clone())
            .into_iter()
            .flat_map(move |result| {
                let from_state = from_state.clone();
                result
                    .map_err(Into::into)
                    .flat_result_map(move |(action, to_state)| {
                        self.prop
                            .modify_action(from_state.clone(), action, to_state)
                            .into_iter()
                            .map(|r| r.map_err(Into::into).map(|(a, s)| (a, s.into())))
                    })
                    .map(|x| x.flatten())
            })
    }
}

impl<Base, Lifter, Prop> Activity<Base::State> for Lifted<Base, Lifter, Prop>
where
    Base: Domain,
    Lifter: ProjectState<Base::State>
        + LiftState<Base::State>
        + ActionMap<Base::State, Prop::ActivityAction>,
    Lifter::ActionMapError: Into<Base::Error>,
    Lifter::ProjectionError: Into<Base::Error>,
    Lifter::LiftError: Into<Base::Error>,
    Prop: Activity<Lifter::ProjectedState>,
    Base::State: Clone,
    Prop::ActivityAction: Clone,
    Prop::ActivityError: Into<Base::Error>,
{
    type ActivityAction = Lifter::ToAction;
    type ActivityError = Base::Error;
    type Choices<'a> = impl Iterator<Item=Result<(Self::ActivityAction, Base::State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Base: 'a,
        Prop: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a;

    fn choices<'a>(&'a self, from_state: Base::State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityAction: 'a,
        Self::ActivityError: 'a,
        Base::State: 'a,
    {
        [self.lifter.project(&from_state)]
            .into_iter()
            .filter_map(
                |r: Result<Option<Lifter::ProjectedState>, Lifter::ProjectionError>| r.transpose(),
            )
            .flat_map(
                move |r: Result<Lifter::ProjectedState, Lifter::ProjectionError>| {
                    let from_state = from_state.clone();
                    r.map_err(Into::into)
                        .flat_result_map(move |projected_state| {
                            let from_state = from_state.clone();
                            self.prop.choices(projected_state).into_iter().flat_map(
                                move |r: Result<
                                    (Prop::ActivityAction, Lifter::ProjectedState),
                                    Prop::ActivityError,
                                >| {
                                    let from_state = from_state.clone();
                                    r.map_err(Into::into)
                                        .flat_result_map(move |(action, state)| {
                                            let from_state = from_state.clone();
                                            self.lifter
                                                .lift(&from_state, state)
                                                .map_err(Into::into)
                                                .flat_result_map(
                                                    move |lifted_state_opt: Option<Base::State>| {
                                                        let from_state = from_state.clone();
                                                        let action = action.clone();
                                                        lifted_state_opt.into_iter().flat_map(
                                                            move |lifted_state: Base::State| {
                                                                let from_state = from_state.clone();
                                                                let action = action.clone();
                                                                self.lifter
                                                                    .map_action(from_state, action)
                                                                    .into_iter()
                                                                    .map(move |r| {
                                                                        let lifted_state =
                                                                            lifted_state.clone();
                                                                        r.map_err(Into::into).map(
                                                                            move |a| {
                                                                                (
                                                                                    a.into(),
                                                                                    lifted_state,
                                                                                )
                                                                            },
                                                                        )
                                                                    })
                                                            },
                                                        )
                                                    },
                                                )
                                                .map(|x| x.flatten())
                                        })
                                        .map(|x| x.flatten())
                                },
                            )
                        })
                        .map(|x| x.flatten())
                },
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::NoError;
    use std::collections::HashSet;
    use thiserror::Error as ThisError;

    struct Count {
        by_interval: Vec<u64>,
    }

    #[derive(Debug, PartialEq, Eq)]
    struct Interval(u64);

    impl Activity<u64> for Count {
        type ActivityAction = Interval;
        type ActivityError = NoError;
        type Choices<'a> = impl Iterator<Item = Result<(Interval, u64), NoError>> + 'a;

        fn choices<'a>(&'a self, s: u64) -> Self::Choices<'a>
        where
            Self: 'a,
            Self::ActivityAction: 'a,
            Self::ActivityError: 'a,
            u64: 'a,
        {
            self.by_interval
                .iter()
                .map(move |interval| Ok((Interval(*interval), s + *interval)))
        }
    }

    struct Multiplier(u64);
    impl ActivityModifier<u64, Interval> for Multiplier {
        type ModifiedAction = Interval;
        type ModifiedActionError = NoError;
        type ModifiedChoices<'a> =
            impl IntoIterator<Item = Result<(Interval, u64), Self::ModifiedActionError>> + 'a;
        fn modify_action<'a>(
            &'a self,
            from_state: u64,
            from_action: Interval,
            _: u64,
        ) -> Self::ModifiedChoices<'a>
        where
            Interval: 'a,
            u64: 'a,
        {
            let interval = from_action.0 * self.0;
            [Ok((Interval(interval), from_state + interval))]
        }
    }

    struct DoubleTheOdds;
    impl ActivityModifier<u64, Interval> for DoubleTheOdds {
        type ModifiedAction = Interval;
        type ModifiedActionError = NoError;
        type ModifiedChoices<'a> =
            impl IntoIterator<Item = Result<(Interval, u64), Self::ModifiedActionError>> + 'a;
        fn modify_action<'a>(
            &'a self,
            from_state: u64,
            from_action: Interval,
            _: u64,
        ) -> Self::ModifiedChoices<'a>
        where
            Interval: 'a,
            u64: 'a,
        {
            let interval = if from_state & 1 != 0 {
                // If the value is odd, double it
                from_action.0 * 2
            } else {
                // If the value is even, leave it alone
                from_action.0
            };

            [Ok((Interval(interval), from_state + interval))]
        }
    }

    #[derive(ThisError, Debug, PartialEq, Eq)]
    #[error("This is a test error")]
    struct TestError;

    struct MapToTestError;
    impl ActivityModifier<u64, Interval> for MapToTestError {
        type ModifiedAction = Interval;
        type ModifiedActionError = TestError;
        type ModifiedChoices<'a> =
            impl IntoIterator<Item = Result<(Interval, u64), Self::ModifiedActionError>> + 'a;
        fn modify_action<'a>(&'a self, _: u64, _: Interval, _: u64) -> Self::ModifiedChoices<'a>
        where
            Interval: 'a,
            u64: 'a,
        {
            [Err(TestError)]
        }
    }

    #[test]
    fn test_activity_chain_map() {
        let domain = DefineTrait::<u64>::new().with(Count {
            by_interval: vec![1, 2, 3],
        });

        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(
            choices.unwrap(),
            vec![(Interval(1), 1), (Interval(2), 2), (Interval(3), 3)]
        );

        let domain = domain.chain(Count {
            by_interval: vec![10, 20, 30],
        });
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(
            choices.unwrap(),
            vec![
                (Interval(1), 1),
                (Interval(2), 2),
                (Interval(3), 3),
                (Interval(10), 10),
                (Interval(20), 20),
                (Interval(30), 30),
            ]
        );

        let domain = domain.map(Multiplier(2));
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(
            choices.unwrap(),
            vec![
                (Interval(2), 2),
                (Interval(4), 4),
                (Interval(6), 6),
                (Interval(20), 20),
                (Interval(40), 40),
                (Interval(60), 60),
            ]
        );

        let domain = domain.map(MapToTestError);
        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert!(choices.is_err());
    }

    #[test]
    fn test_state_dependent_activity_map() {
        let domain = DefineTrait::<u64>::new()
            .with(Count {
                by_interval: vec![2, 3, 4, 5],
            })
            .map(DoubleTheOdds);

        let choices: Result<Vec<_>, _> = domain.choices(0).collect();
        assert_eq!(
            choices.unwrap(),
            [2, 3, 4, 5]
                .into_iter()
                .map(|x| (Interval(x), x))
                .collect::<Vec<_>>()
        );

        let choices: Result<Vec<_>, _> = domain.choices(13).collect();
        assert_eq!(
            choices.unwrap(),
            vec![4, 6, 8, 10]
                .into_iter()
                .map(|x| (Interval(x), 13 + x))
                .collect::<Vec<_>>()
        );
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

    impl Item {
        fn buy(&mut self, at_price: u64) {
            self.count += 1;
            self.budget -= at_price;
        }

        fn sell(&mut self, at_price: u64) {
            self.count -= 1;
            self.budget += at_price;
        }
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct Buy(u64 /* price per unit */);
    impl Activity<Item> for Buy {
        type ActivityAction = Buy;
        type ActivityError = NoError;
        type Choices<'a> = Option<Result<(Buy, Item), NoError>>;
        fn choices<'a>(&'a self, mut from_state: Item) -> Option<Result<(Buy, Item), NoError>>
        where
            Item: 'a,
        {
            if from_state.budget < self.0 {
                None
            } else {
                from_state.buy(self.0);
                Some(Ok((*self, from_state)))
            }
        }
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct Sell(u64 /* price per unit */);
    impl Activity<Item> for Sell {
        type ActivityAction = Sell;
        type ActivityError = NoError;
        type Choices<'a> = Option<Result<(Sell, Item), NoError>>;
        fn choices<'a>(&'a self, mut from_state: Item) -> Self::Choices<'a>
        where
            Item: 'a,
        {
            if from_state.count <= 0 {
                None
            } else {
                from_state.sell(self.0);
                Some(Ok((*self, from_state)))
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
            state: &Inventory,
        ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
            Ok(Some(Item {
                count: state.apples,
                budget: state.budget,
            }))
        }
    }
    impl LiftState<Inventory> for JustApples {
        type LiftError = NoError;
        fn lift(
            &self,
            original: &Inventory,
            projection: Self::ProjectedState,
        ) -> Result<Option<Inventory>, Self::LiftError> {
            Ok(Some(Inventory {
                apples: projection.count,
                budget: projection.budget,
                ..original.clone()
            }))
        }
    }
    impl<A: Into<Transaction>> ActionMap<Inventory, A> for JustApples {
        type ActionMapError = NoError;
        type ToAction = Order;
        type ToActions<'a> = Option<Result<Order, NoError>> where A: 'a;
        fn map_action<'a>(&'a self, _: Inventory, from_action: A) -> Self::ToActions<'a>
        where
            Transaction: 'a,
            Inventory: 'a,
            A: 'a,
        {
            Some(Ok(Order::Apples(from_action.into())))
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
            state: &Inventory,
        ) -> Result<Option<Self::ProjectedState>, Self::ProjectionError> {
            Ok(Some(Item {
                count: state.bananas,
                budget: state.budget,
            }))
        }
    }
    impl LiftState<Inventory> for JustBananas {
        type LiftError = NoError;
        fn lift(
            &self,
            original: &Inventory,
            projection: Self::ProjectedState,
        ) -> Result<Option<Inventory>, Self::LiftError> {
            Ok(Some(Inventory {
                bananas: projection.count,
                budget: projection.budget,
                ..original.clone()
            }))
        }
    }
    impl<A: Into<Transaction>> ActionMap<Inventory, A> for JustBananas {
        type ActionMapError = NoError;
        type ToAction = Order;
        type ToActions<'a> = Option<Result<Order, NoError>> where A: 'a;
        fn map_action<'a>(&'a self, _: Inventory, from_action: A) -> Self::ToActions<'a>
        where
            Transaction: 'a,
            Inventory: 'a,
            A: 'a,
        {
            Some(Ok(Order::Bananas(from_action.into())))
        }
    }

    #[test]
    fn test_lifted_activity() {
        let domain = DefineTrait::<Inventory>::new()
            .lift(JustApples, Buy(20))
            .chain_lift(JustApples, Sell(60))
            .chain_lift(JustBananas, Buy(30))
            .chain_lift(JustBananas, Sell(80));

        let inventory = Inventory {
            apples: 5,
            bananas: 3,
            budget: 25,
        };
        let choices: Result<HashSet<_>, _> = domain
            .choices(inventory)
            .map(|r| r.map(|(a, _)| a))
            .collect();
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
        let choices: Result<HashSet<_>, _> = domain
            .choices(inventory)
            .map(|r| r.map(|(a, _)| a))
            .collect();
        let choices = choices.unwrap();
        assert_eq!(choices.len(), 3);
        assert!(choices.contains(&Order::Apples(Transaction::Buy(Buy(20)))));
        assert!(choices.contains(&Order::Bananas(Transaction::Buy(Buy(30)))));
        assert!(choices.contains(&Order::Bananas(Transaction::Sell(Sell(80)))));
    }
}
