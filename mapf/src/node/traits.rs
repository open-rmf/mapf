/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

use std::hash::Hash;
use std::sync::Arc;
use std::ops::Add;
use std::cmp::Ord;
use std::fmt::Debug;
use num::traits::Zero;
use std::cmp::Ordering;
use time_point::TimePoint;

/// A trait for nodes that are used by path search algorithms. The expectation
/// is that each node knows its parent node in the search.
pub trait PathSearch {
    /// Return the parent node that led up to this node.
    fn parent(&self) -> &Option<Arc<Self>>;
}

/// A trait for nodes that represent an agent that is taking actions which change
/// its state. This is expected to be used with the PathSearch trait where the
/// path represents a sequence of actions take by the agent.
pub trait Agent<State, Action>: PathSearch {
    fn state(&self) -> &State;

    fn action(&self) -> &Option<Action>;
}

/// A trait that describes what is needed to define a cost.
pub trait Cost: Ord + Add<Output=Self> + Sized + Copy + Zero + std::fmt::Debug { }
impl<T: Ord + Add<Output=Self> + Sized + Copy + Zero + std::fmt::Debug> Cost for T { }

/// A trait for nodes that have a cost associated with them.
pub trait Weighted: Sized {
    type Cost: Cost;

    /// Return the cost of reaching the state of this node. For path search,
    /// this means the cost of the path that leads up to this node from the root
    /// of the path search.
    fn cost(&self) -> Self::Cost;
}

/// A subset of Node which has an informed estimate of how far it is from its goal
pub trait Informed: Weighted {

    /// Get the estimate of the remaining cost from the goal
    fn remaining_cost_estimate(&self) -> Self::Cost;

    /// Get the estimate of the total cost from the goal. This should always be
    /// equal to cost() + remaining_cost_estimate(), but we make it a separate
    /// function so that implementers can choose to micro-optimize by adding and
    /// saving the result inside of the node itself.
    fn total_cost_estimate(&self) -> Self::Cost {
        self.cost() + self.remaining_cost_estimate()
    }
}

/// A trait for Nodes that have a time value
pub trait Timed {
    fn time(&self) -> &TimePoint;
}

/// Trait for nodes that can expand in reverse. HashOption is required for both
/// Self and Self::Reverse because the keys are used to match up when a forward
/// and reverse node are connected. Nodes that return None for their key cannot
/// connect to a complementary Node.
pub trait Reversible: PartialKeyed {
    /// The reversed type of this Node. Its key type must match the key type of
    /// the forward node so that they can be compared against each other.
    type Reverse: PartialKeyed<Key=<Self as PartialKeyed>::Key>;
}

pub type ReverseOf<N> = <N as Reversible>::Reverse;

/// The set of traits required for a Key
pub trait Key: Hash + Eq + Clone + Debug { }
impl<T: Hash + Eq + Clone + Debug> Key for T { }

/// A trait for nodes that can sometimes provide a unique key but other times
/// cannot.
pub trait PartialKeyed {

    type Key: Key;

    /// Attempt to get a key that uniquely identifies the state of this node.
    /// If the node cannot be uniquely identified by a key, this will return
    /// None.
    #[must_use]
    fn partial_key(&self) -> Option<&Self::Key>;
}

/// A trait for nodes that can always provide a unique key. The PartialKeyed
/// trait must be implemented, and its implementation is what will be used. If
/// PartialKeyed ever returns None, then generics which use the Keyed trait may
/// panic.
pub trait Keyed: PartialKeyed {
    #[must_use]
    fn key(&self) -> &Self::Key {
        self.partial_key().unwrap()
    }
}

pub struct CostCmp<N: Weighted>(pub Arc<N>);

impl<N: Weighted> Ord for CostCmp<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        return self.0.cost().cmp(&other.0.cost());
    }
}

impl<N: Weighted> PartialOrd for CostCmp<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        return self.0.cost().partial_cmp(&other.0.cost());
    }
}

impl<N: Weighted> PartialEq for CostCmp<N> {
    fn eq(&self, other: &Self) -> bool {
        return self.0.cost().eq(&other.0.cost());
    }
}

impl<N: Weighted> Eq for CostCmp<N> { }

#[derive(Debug, Clone)]
pub struct TotalCostEstimateCmp<N: Informed>(pub Arc<N>);

impl<N: Informed> Ord for TotalCostEstimateCmp<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        return self.0.total_cost_estimate().cmp(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> PartialOrd for TotalCostEstimateCmp<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        return self.0.total_cost_estimate().partial_cmp(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> PartialEq for TotalCostEstimateCmp<N> {
    fn eq(&self, other: &Self) -> bool {
        return self.0.total_cost_estimate().eq(&other.0.total_cost_estimate());
    }
}

impl<N: Informed> Eq for TotalCostEstimateCmp<N> { }

pub type KeyOf<N> = <N as PartialKeyed>::Key;
pub type CostOf<N> = <N as Weighted>::Cost;
