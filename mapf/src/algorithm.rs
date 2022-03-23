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

use super::expander;
use super::tracker;
use super::node;

pub enum Result {
    Incomplete,
    Impossible,
    Solved(Solution)
}

pub struct Solution {

}


pub trait Storage<Expander: expander::Expander> {
    fn node_count(&self) -> usize;
    fn top_cost_estimate(&self)
    -> <<Expander as expander::Expander>::Node as node::Node>::Cost;
}

pub trait Algorithm<Expander: expander::Expander>  {
    type Storage: Storage<Expander>;

    fn initialize<Goal, Tracker: tracker::Tracker<Expander::Node>>(
        &self,
        start: &Expander::Start,
        goal: &Goal,
        expander: &Expander,
        tracker: &Tracker
    ) -> Self::Storage;

}
