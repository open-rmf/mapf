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
    domain::{
        Domain, Reversible, Keyed, Closable, Activity, Weighted, Initializable,
        Keyring, ClosedStatusForKey, Backtrack,
    },
    algorithm::{
        Algorithm, Coherent, Solvable, Status,
        tree::Path,
        dijkstra::{
            Dijkstra,
            forward::{Memory, DijkstraSearchError},
        },
    },
};
use std::ops::Add;

pub struct BackwardDijkstra<D: Reversible>
where
    D::Reverse: Domain
    + Keyed
    + Activity<ReverseStateOf<D>>
    + Weighted<ReverseStateOf<D>, ReverseActionOf<D>>
    + Closable<ReverseStateOf<D>>,
{
    backward: Dijkstra<D::Reverse>,
}

impl<D: Reversible> Algorithm for BackwardDijkstra<D>
where
    D::Reverse: Domain
    + Keyed
    + Activity<ReverseStateOf<D>>
    + Weighted<ReverseStateOf<D>, ReverseActionOf<D>>
    + Closable<ReverseStateOf<D>>,
{
    type Memory = BackwardMemory<D>;
}

pub struct BackwardMemory<D: Reversible>
where
    D::Reverse: Domain
    + Keyed
    + Activity<ReverseStateOf<D>>
    + Weighted<ReverseStateOf<D>, ReverseActionOf<D>>
    + Closable<ReverseStateOf<D>>,
{
    backward: Memory<D::Reverse>,
}

impl<D: Reversible, Start, Goal> Coherent<Start, Goal> for BackwardDijkstra<D>
where
    D::Reverse: Domain
    + Keyring<ReverseStateOf<D>>
    + Initializable<Start, ReverseStateOf<D>>
    + Initializable<Goal, ReverseStateOf<D>>
    + Activity<ReverseStateOf<D>>
    + Weighted<ReverseStateOf<D>, ReverseActionOf<D>>
    + Closable<ReverseStateOf<D>>,
    <D::Reverse as Initializable<Start, ReverseStateOf<D>>>::InitialError: Into<ReverseErrorOf<D>>,
    <D::Reverse as Initializable<Goal, ReverseStateOf<D>>>::InitialError: Into<ReverseErrorOf<D>>,
    <D::Reverse as Weighted<ReverseStateOf<D>, ReverseActionOf<D>>>::WeightedError: Into<ReverseErrorOf<D>>,
    ReverseStateOf<D>: Clone,
    ReverseCostOf<D>: Clone + Ord,
    ReverseKeyOf<D>: Clone,
    Start: Clone,
    Goal: Clone,
{
    type InitError = DijkstraSearchError<ReverseErrorOf<D>>;

    fn initialize(
        &self,
        start: Start,
        goal: &Goal,
    ) -> Result<Self::Memory, Self::InitError> {
        let memory = self.backward.initialize(goal.clone(), &start)?;
        Ok(BackwardMemory {
            backward: memory,
        })
    }
}

impl<D> Solvable<ReverseKeyOf<D>> for BackwardDijkstra<D>
where
    D: Domain
    + Reversible
    + Activity<D::State>
    + Weighted<D::State, D::ActivityAction>,
    D::Reverse: Domain
    + Keyring<ReverseStateOf<D>>
    + Activity<ReverseStateOf<D>>
    + Weighted<ReverseStateOf<D>, ReverseActionOf<D>, Cost = D::Cost>
    + Closable<ReverseStateOf<D>>
    + Backtrack<
        ReverseStateOf<D>,
        ReverseActionOf<D>,
        ForwardState = D::State,
        ForwardAction = D::ActivityAction,
    >,
    ReverseStateOf<D>: Clone,
    ReverseActionOf<D>: Clone,
    ReverseCostOf<D>: Clone + Ord + Add<ReverseCostOf<D>, Output = ReverseCostOf<D>>,
    <D::Reverse as Closable<ReverseStateOf<D>>>::ClosedSet<usize>: ClosedStatusForKey<ReverseKeyOf<D>, usize>,
    <D::Reverse as Activity<ReverseStateOf<D>>>::ActivityError: Into<ReverseErrorOf<D>>,
    <D::Reverse as Weighted<ReverseStateOf<D>, ReverseActionOf<D>>>::WeightedError: Into<ReverseErrorOf<D>>,
    <D::Reverse as Backtrack<ReverseStateOf<D>, ReverseActionOf<D>>>::BacktrackError: Into<ReverseErrorOf<D>>,
{
    type Solution = Path<D::State, D::ActivityAction, D::Cost>;
    type StepError = DijkstraSearchError<ReverseErrorOf<D>>;

    fn step(
        &self,
        memory: &mut Self::Memory,
        _: &ReverseKeyOf<D>,
    ) -> Result<Status<Self::Solution>, Self::StepError> {
        // Note: Passing in the goal doesn't matter because the Dijkstra
        // algorithm memory saves the goal information anyway.
        self.backward.step(&mut memory.backward, &())
            .and_then(|r|
                // If a solution is found, backtrack the path to make it run
                // forward instead of being in reverse.
                r.and_then(|path| path.backtrack(self.backward.domain()))
                    .map_err(DijkstraSearchError::Domain)
            )
    }
}

type ReverseStateOf<D> = <<D as Reversible>::Reverse as Domain>::State;
type ReverseActionOf<D> = <<D as Reversible>::Reverse as Activity<ReverseStateOf<D>>>::ActivityAction;
type ReverseKeyOf<D> = <<D as Reversible>::Reverse as Keyed>::Key;
type ReverseErrorOf<D> = <<D as Reversible>::Reverse as Domain>::Error;
type ReverseCostOf<D> = <<D as Reversible>::Reverse as Weighted<ReverseStateOf<D>, ReverseActionOf<D>>>::Cost;
