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

use crate::{
    domain::Domain,
    error::NoError
};

/// The Activity trait describes an activity that can be performed within a
/// domain. An activity yields action choices for an agent where those choices
/// may depend on the state of the agent. These action choices can be examined
/// by a planner to discover a valid or optimal sequence of actions to reach a
/// goal.
///
/// In graph theory terms, an action an edge of a graph. An Activity is a
/// function that maps a graph vertex (agent state) to the outgoing edges from
/// that vertex.
pub trait Activity<State, Action> {
    /// What kind of error can happen if a bad state is provided
    type ActivityError;

    /// Concrete type for the returned container of choices
    type Choices<'a>: IntoIterator<Item = Result<(Action, State), Self::ActivityError>> + 'a
    where
        Self: 'a,
        Self::ActivityError: 'a,
        State: 'a,
        Action: 'a;

    /// What choices can be made related to this activity from the provided state
    // TODO(@mxgrey): Investigate whether `from_state` can have a `&State` type
    fn choices<'a>(&'a self, from_state: State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityError: 'a,
        State: 'a,
        Action: 'a;
}

/// [`NoActivity`] can be used as a placeholder where an Activity is required
/// but it doesn't need to do anything.
pub struct NoActivity<A>(std::marker::PhantomData<A>);
impl<State, Action> Activity<State, Action> for NoActivity<Action> {
    type ActivityError = NoError;
    type Choices<'a>
        = [Result<(Action, State), NoError>; 0]
    where
        Self: 'a,
        Self::ActivityError: 'a,
        State: 'a,
        Action: 'a;

    fn choices<'a>(&'a self, _: State) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::ActivityError: 'a,
        State: 'a,
        Action: 'a,
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
    type ModifiedChoices<'a>
        = [Result<(FromAction, State), NoError>; 1]
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

    impl Activity<u64, Interval> for Count {
        type ActivityError = NoError;
        type Choices<'a> = Vec<Result<(Interval, u64), NoError>>;

        fn choices<'a>(&'a self, s: u64) -> Self::Choices<'a>
        where
            Self: 'a,
            Self::ActivityError: 'a,
        {
            self.by_interval
                .iter()
                .map(move |interval| Ok((Interval(*interval), s + *interval)))
                .collect()
        }
    }

    struct Multiplier(u64);
    impl ActivityModifier<u64, Interval> for Multiplier {
        type ModifiedAction = Interval;
        type ModifiedActionError = NoError;
        type ModifiedChoices<'a> = [Result<(Interval, u64), Self::ModifiedActionError>; 1];
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
        type ModifiedChoices<'a> = [Result<(Interval, u64), Self::ModifiedActionError>; 1];
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
        type ModifiedChoices<'a> = [Result<(Interval, u64), Self::ModifiedActionError>; 1];
        fn modify_action<'a>(&'a self, _: u64, _: Interval, _: u64) -> Self::ModifiedChoices<'a>
        where
            Interval: 'a,
            u64: 'a,
        {
            [Err(TestError)]
        }
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
    impl Activity<Item, Buy> for Buy {
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
    impl Activity<Item, Sell> for Sell {
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

    struct Apples<Tx>(Tx);

    impl<Tx> Activity<Inventory, Order> for Apples<Tx>
    where
        Tx: Activity<Item, Transaction>,
    {
        type ActivityError = Tx::ActivityError;
        type Choices<'a> = Vec<Result<(Order, Inventory), Tx::ActivityError>>;
        fn choices<'a>(&'a self, from_state: Inventory) -> Self::Choices<'a>
        where
            Self: 'a,
            Self::ActivityError: 'a,
            Inventory: 'a,
            Order: 'a
        {
            let item = Item {
                count: from_state.apples,
                budget: from_state.budget,
            };

            let mut choices = Vec::new();
            for choice in self.0.choices(item) {
                let choice = match choice {
                    Ok((tx, item)) => {
                        let state = Inventory {
                            apples: item.count,
                            budget: item.budget,
                            ..from_state.clone()
                        };
                        Ok((Order::Apples(tx), state))
                    }
                    Err(err) => Err(err),
                };

                choices.push(choice);
            }

            choices
        }
    }

    struct Bananas<Tx>(Tx);

    impl<Tx> Activity<Inventory, Order> for Bananas<Tx>
    where
        Tx: Activity<Item, Transaction>,
    {
        type ActivityError = Tx::ActivityError;
        type Choices<'a> = Vec<Result<(Order, Inventory), Tx::ActivityError>>;
        fn choices<'a>(&'a self, from_state: Inventory) -> Self::Choices<'a>
        where
            Self: 'a,
            Self::ActivityError: 'a,
            Inventory: 'a,
            Order: 'a
        {
            let item = Item {
                count: from_state.bananas,
                budget: from_state.budget,
            };

            let mut choices = Vec::new();
            for choice in self.0.choices(item) {
                let choice = match choice {
                    Ok((tx, item)) => {
                        let state = Inventory {
                            bananas: item.count,
                            budget: item.budget,
                            ..from_state.clone()
                        };
                        Ok((Order::Bananas(tx), state))
                    }
                    Err(err) => Err(err),
                };

                choices.push(choice);
            }

            choices
        }
    }

    #[derive(Domain)]
    #[domain(state = Inventory, action = Order)]
    struct InventoryDomain {
        #[activity]
        buy_apples: Apples<Buy>,

        #[activity]
        sell_apples: Apples<Sell>,

        #[activity]
        buy_bananas: Bananas<Buy>,

        #[activity]
        sell_bananas: Bananas<Sell>,
    }

    #[test]
    fn test_lifted_activity() {
        let domain = InventoryDomain {
            buy_apples: Apples(Buy(20)),
            sell_apples: Apples(Sell(60)),
            buy_bananas: Bananas(Buy(30)),
            sell_bananas: Bananas(Sell(80)),
        };

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
