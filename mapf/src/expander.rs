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
use super::node;

pub trait Expander {

    type Start;
    type Node: node::Node;
    type Options: Clone;
    type Expansion: Iterator<Item=Rc<Self::Node>>;

    fn default_options(&self) -> Self::Options;

    fn expand(&self, parent: &Rc<Self::Node>) -> Self::Expansion;

    fn start(&self, start: Self::Start) -> Self::Expansion;
}

pub trait Reversible<Reverse: Expander>: Expander {
    /// Note: Reverse::Start must be equivalent to the Forward Expander's Goal
    fn reverse(&self) -> Rc<Reverse>;
}

pub trait Heuristic<Node: node::Node> {
    fn estimate(&self, node: &Node) -> Option<u64>;
}

pub trait Informed<Node: node::Node> {
    type Heuristic: Heuristic<Node>;

    fn heuristic(&self) -> Rc<Self::Heuristic>;
}
