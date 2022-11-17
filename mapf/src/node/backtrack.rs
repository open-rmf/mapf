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

use crate::node::PathSearch;
use std::sync::Arc;

pub struct Backtrack<N: PathSearch> {
    next_node: Option<Arc<N>>,
}

impl<N: PathSearch> Iterator for Backtrack<N> {
    type Item = Arc<N>;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(next_node) = self.next_node.clone() {
            self.next_node = next_node.parent().clone();
            return Some(next_node.clone());
        }

        return None;
    }
}

pub trait Backtrackable {
    type Node: PathSearch;
    fn backtrack(&self) -> Backtrack<Self::Node>;
}

impl<N: PathSearch> Backtrackable for Arc<N> {
    type Node = N;
    fn backtrack(&self) -> Backtrack<Self::Node> {
        Backtrack {
            next_node: Some(self.clone()),
        }
    }
}
