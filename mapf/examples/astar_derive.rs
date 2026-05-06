/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

use mapf::algorithm::{AStar, SearchStatus};
use mapf::domain::{Activity, Cost, Domain, Informed, Keyed, KeyedCloser, Keyring, Weighted};
use mapf::error::NoError;
use mapf::Planner;

/// A simple 2D point state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Point {
    x: i32,
    y: i32,
}

/// Define a domain using the new derive macro.
/// Adding Clone to GridDomain because AStar<D> needs to be Clone for the planner.
#[derive(Domain, Clone)]
#[domain(state = Point, error = NoError)]
struct GridDomain {
    #[activity]
    motion: GridMotion,

    #[weighted]
    cost: ConstantCost,

    #[informed]
    heuristic: ManhattanHeuristic,

    #[closer]
    closer: KeyedCloser<PointRing>,

    #[satisfier]
    satisfier: (), // Default satisfier uses PartialEq

    #[initializer]
    initializer: (), // Default initializer accepts Start: Into<State>
}

/// Move in 4 directions.
#[derive(Clone)]
struct GridMotion;
impl Activity<Point> for GridMotion {
    type Action = char;
    type ActivityError = NoError;
    type Choices<'a>
        = std::vec::IntoIter<Result<(char, Point), NoError>>
    where
        Self: 'a,
        Point: 'a;

    fn choices<'a>(&'a self, from_state: Point) -> Self::Choices<'a>
    where
        Self: 'a,
        Point: 'a,
    {
        vec![
            Ok((
                'N',
                Point {
                    x: from_state.x,
                    y: from_state.y + 1,
                },
            )),
            Ok((
                'S',
                Point {
                    x: from_state.x,
                    y: from_state.y - 1,
                },
            )),
            Ok((
                'E',
                Point {
                    x: from_state.x + 1,
                    y: from_state.y,
                },
            )),
            Ok((
                'W',
                Point {
                    x: from_state.x - 1,
                    y: from_state.y,
                },
            )),
        ]
        .into_iter()
    }
}

/// Every move costs 1.0. We use mapf::domain::Cost to get Ord for floats.
#[derive(Clone)]
struct ConstantCost;
impl Weighted<Point, char> for ConstantCost {
    type Cost = Cost<f64>;
    type WeightedError = NoError;
    fn cost(&self, _: &Point, _: &char, _: &Point) -> Result<Option<Cost<f64>>, NoError> {
        Ok(Some(Cost(1.0)))
    }
    fn initial_cost(&self, _: &Point) -> Result<Option<Cost<f64>>, NoError> {
        Ok(Some(Cost(0.0)))
    }
}

/// Manhattan distance heuristic.
#[derive(Clone)]
struct ManhattanHeuristic;
impl Informed<Point, Point> for ManhattanHeuristic {
    type CostEstimate = Cost<f64>;
    type InformedError = NoError;
    fn estimate_remaining_cost(
        &self,
        from: &Point,
        to: &Point,
    ) -> Result<Option<Cost<f64>>, NoError> {
        Ok(Some(Cost(
            ((from.x - to.x).abs() + (from.y - to.y).abs()) as f64,
        )))
    }
}

/// Keyring for Point state.
#[derive(Clone, Default)]
struct PointRing;
impl Keyed for PointRing {
    type Key = Point;
}
impl Keyring<Point> for PointRing {
    type KeyRef<'a>
        = &'a Point
    where
        Self: 'a,
        Point: 'a;
    fn key_for<'a>(&'a self, state: &'a Point) -> Self::KeyRef<'a>
    where
        Self: 'a,
        Point: 'a,
    {
        state
    }
}

fn main() {
    let domain = GridDomain {
        motion: GridMotion,
        cost: ConstantCost,
        heuristic: ManhattanHeuristic,
        closer: KeyedCloser(PointRing),
        satisfier: (),
        initializer: (),
    };

    let start = Point { x: 0, y: 0 };
    let goal = Point { x: 5, y: 5 };

    // Create a planner using AStar with our derived domain.
    let planner = Planner::new(AStar(domain));

    // Plan and solve.
    let result = planner.plan(start, goal).unwrap().solve().unwrap();

    if let SearchStatus::Solved(path) = result {
        println!("Goal reached!");
        println!("Initial state: {:?}", path.initial_state);
        for (action, state) in path.sequence {
            println!("  {:?} -> {:?}", action, state);
        }
    } else {
        println!("Failed to find path: {:?}", result);
    }
}
