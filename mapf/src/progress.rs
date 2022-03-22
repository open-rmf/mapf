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

use std::rc::Rc;

use super::expander;

pub trait Tracker<Expander>: Default where
    Expander: expander::Expander{

    fn expanded_to(&mut self, node: &Expander::Node);
}

#[derive(Default)]
pub struct NoDebug;

impl<Expander: expander::Expander> Tracker<Expander> for NoDebug {

    fn expanded_to(&mut self, _: &Expander::Node) { }
}

pub struct Progress<Expander, Tracker=NoDebug> where
    Expander: expander::Expander {

    /// The object which determines patterns for expansion
    expander: Rc<Expander>,

    /// The object which tracks planning progress
    tracker: Tracker
}
