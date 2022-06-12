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

use crate::expander::traits::*;
use std::sync::Arc;
use std::fmt::Debug;

pub struct Closure<N, G: Goal<N>, Err: Debug, Exp: IntoIterator<Item=Result<Arc<N>, Err>>, F: Fn(&Arc<N>, Option<&G>) -> Exp> {
    closure: F,
    _ignore: std::marker::PhantomData<(N, G, Err, Exp)>,
}

impl<N, G, Err, Exp, F> Closure<N, G, Err, Exp, F>
where
    G: Goal<N>,
    Err: Debug,
    Exp: IntoIterator<Item=Result<Arc<N>, Err>>,
    F: Fn(&Arc<N>, Option<&G>) -> Exp
{
    pub fn new(closure: F) -> Self {
        Self{closure, _ignore: Default::default()}
    }
}

impl<N, G, Err, Exp, F> Expander for Closure<N, G, Err, Exp, F>
where
    G: Goal<N>,
    Err: Debug,
    Exp: IntoIterator<Item=Result<Arc<N>, Err>>,
    F: Fn(&Arc<N>, Option<&G>) -> Exp,
{
    type Node = N;
    type Goal = G;
}

impl<N, G, Err, Exp, F> Expandable for Closure<N, G, Err, Exp, F>
where
    G: Goal<N>,
    Err: Debug,
    Exp: IntoIterator<Item=Result<Arc<N>, Err>>,
    F: Fn(&Arc<N>, Option<&G>) -> Exp
{
    type ExpansionError = Err;
    type Expansion<'a> where Self: 'a = Exp;

    fn expand<'a>(
        &'a self,
        parent: &'a Arc<Self::Node>,
        goal: Option<&'a Self::Goal>,
    ) -> Self::Expansion<'a> {
        (self.closure)(parent, goal)
    }
}
