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
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status<Expander: expander::Expander> {
    Incomplete,
    Impossible,
    Solved(Expander::Solution)
}

pub trait Storage<Expander: expander::Expander> {
    fn node_count(&self) -> usize;
    fn top_cost_estimate(&self) -> Option<expander::Cost<Expander>>;
}

pub trait Algorithm<Expander: expander::Expander>  {
    type Storage: Storage<Expander>;

    fn initialize<Tracker: tracker::Tracker<Expander::Node>>(
        &self,
        expander: Rc<Expander>,
        start: &Expander::Start,
        goal: Expander::Goal,
        tracker: &mut Tracker,
    ) -> Self::Storage;

    fn step<Tracker: tracker::Tracker<Expander::Node>>(
        &self,
        storage: &mut Self::Storage,
        options: &Expander::Options,
        tracker: &mut Tracker,
    ) -> Status<Expander>;
}
