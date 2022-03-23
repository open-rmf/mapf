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
use std::ops::Fn;
use std::boxed::Box;

use super::expander;
use super::algorithm;
use super::algorithm::{Result, Storage};
use super::node;
use super::tracker;

pub trait Goal<Node: node::Node> {
    fn is_satisfied(node: &Node) -> bool;
}

pub struct Planner<Expander, Algorithm>  where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander>, {

    /// The object which determines the search pattern
    algorithm: Rc<Algorithm>,

    /// The object which determines patterns for expansion
    expander: Rc<Expander>,

    /// The default options to use when planning
    default_options: Expander::Options,
}

impl<Expander, Algorithm> Planner<Expander, Algorithm>  where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander> {

    /// Construct a new planner with the given configuration and a set of
    /// default options.
    pub fn new(
        expander: Rc<Expander>,
        default_options: Expander::Options
    ) -> Self
    where Algorithm: Default {
        return Self {
            algorithm: Rc::new(Algorithm::default()),
            expander,
            default_options
        }
    }

    pub fn with_algorithm(
        algorithm: Rc<Algorithm>,
        expander: Rc<Expander>,
        default_options: Expander::Options
    ) -> Self {
        return Self { algorithm, expander, default_options }
    }

    pub fn plan<GoalType: Goal<Expander::Node>>(
        &self,
        start: &Expander::Start,
        goal: GoalType
    ) -> Progress<Expander, Algorithm, GoalType, tracker::NoDebug> {
        let tracker = tracker::NoDebug::default();
        let storage = self.algorithm.initialize(
            &start, &goal, &self.expander, &tracker
        );

        return Progress {
            storage,
            algorithm: self.algorithm.clone(),
            expander: self.expander.clone(),
            goal,
            progression_options: ProgressionOptions::default(),
            expansion_options: None,
            tracker
        }
    }

    pub fn debug<GoalType, Tracker>(
        &self,
        start: &Expander::Start,
        goal: GoalType,
        tracker: Tracker
    ) -> Progress<Expander, Algorithm, GoalType, Tracker>
    where
        GoalType: Goal<Expander::Node>,
        Tracker: tracker::Tracker<Expander::Node> {

        let storage = self.algorithm.initialize(
            &start, &goal, &self.expander, &tracker
        );

        return Progress {
            storage,
            algorithm: self.algorithm.clone(),
            expander: self.expander.clone(),
            goal,
            progression_options: ProgressionOptions::default(),
            expansion_options: None,
            tracker
        }
    }
}

enum Interruption {
    Continue,
    Stop
}

pub struct ProgressionOptions<Expander: expander::Expander> {

    /// A function that can decide to interrupt the planning
    interrupter: Option<Box<dyn FnMut() -> Interruption>>,

    /// The maximum total cost estimate that the top node can reach before
    /// the solve attempt quits.
    max_cost_estimate: Option<<<Expander as expander::Expander>::Node as node::Node>::Cost>,

    /// The maximum size that the search queue can reach before the
    /// solve attemptp quits.
    search_queue_limit: Option<usize>,
}

impl<Expander: expander::Expander> Default for ProgressionOptions<Expander> {
    fn default() -> Self {
        return Self {
            interrupter: None,
            max_cost_estimate: None,
            search_queue_limit: None,
        }
    }
}

pub struct Progress<Expander, Algorithm, Goal, Tracker> where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander> {

    /// Storage container for the progress of the search algorithm
    storage: Algorithm::Storage,

    /// The object which determines the search pattern
    algorithm: Rc<Algorithm>,

    /// The object which determines patterns for expansion
    expander: Rc<Expander>,

    /// The goal that the planner should try to reach
    goal: Goal,

    /// The options that moderate the progress of the solving
    progression_options: ProgressionOptions<Expander>,

    /// The options that should be used by the expander
    expansion_options: Option<Expander::Options>,

    /// The object which tracks planning progress
    tracker: Tracker
}

impl<Expander, Algorithm, Goal, Tracker>
Progress<Expander, Algorithm, Goal, Tracker>
where
    Expander: expander::Expander,
    Algorithm: algorithm::Algorithm<Expander> {

    pub fn with_expansion_options(&mut self, expansion_opts: Expander::Options)
    -> &mut Self {
        self.expansion_options = Some(expansion_opts);
        return self;
    }

    pub fn expansion_options_mut(&mut self) -> &Expander::Options {
        return self.expansion_options.get_or_insert_with(
            ||{ self.expander.default_options() }
        );
    }

    pub fn expansion_options(&self) -> &Option<Expander::Options> {
        return &self.expansion_options;
    }

    pub fn with_progression_options(
        &mut self,
        progression_opts: ProgressionOptions<Expander>)
    -> &mut Self {
        self.progression_options = progression_opts;
        return self;
    }

    pub fn progression_options_mut(&mut self) -> &mut ProgressionOptions<Expander> {
        return &mut self.progression_options;
    }

    pub fn progression_options(&self) -> &ProgressionOptions<Expander> {
        return &self.progression_options;
    }

    pub fn solve(&mut self) -> Result {
        loop {
            if self.need_to_interrupt() {
                return Result::Incomplete;
            }

            let result = self.step();
            if let Result::Incomplete = result {
                continue;
            }

            return result;
        }
    }

    pub fn step(&mut self) -> Result {
        return Result::Impossible;
    }

    fn need_to_interrupt(&mut self) -> bool {
        if let Some(interrupter) = &mut self.progression_options.interrupter {
            if let Interruption::Stop = interrupter() {
                return true;
            }
        }

        if let Some(max_cost_estimate) = &self.progression_options.max_cost_estimate {
            if self.storage.top_cost_estimate() > *max_cost_estimate {
                return true;
            }
        }

        if let Some(search_queue_limit) = &self.progression_options.search_queue_limit {
            if self.storage.node_count() > *search_queue_limit {
                return true;
            }
        }

        return false;
    }
}
