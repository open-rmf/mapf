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

use mapf::domain::{Activity, Domain, Informed, Keyed, KeyedCloser, Keyring, Weighted};
use mapf::error::NoError;

#[derive(Domain)]
#[domain(state = f64, error = NoError)]
struct RobotDomain {
    #[activity]
    motion: MyActivity,

    #[weighted]
    cost: MyWeighted,

    #[informed]
    heuristic: MyInformed,

    #[closer]
    closer: KeyedCloser<MyRing>,
}

#[derive(Clone)]
struct MyActivity;
impl Activity<f64> for MyActivity {
    type Action = f64;
    type ActivityError = NoError;
    type Choices<'a>
        = std::vec::IntoIter<Result<(f64, f64), NoError>>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        f64: 'a;

    fn choices<'a>(&'a self, from_state: f64) -> Self::Choices<'a>
    where
        Self: 'a,
        Self::Action: 'a,
        Self::ActivityError: 'a,
        f64: 'a,
    {
        vec![Ok((1.0, from_state + 1.0))].into_iter()
    }
}

struct MyWeighted;
impl Weighted<f64, f64> for MyWeighted {
    type Cost = f64;
    type WeightedError = NoError;
    fn cost(&self, _: &f64, _: &f64, _: &f64) -> Result<Option<f64>, NoError> {
        Ok(Some(1.0))
    }
    fn initial_cost(&self, _: &f64) -> Result<Option<f64>, NoError> {
        Ok(Some(0.0))
    }
}

struct MyInformed;
impl Informed<f64, f64> for MyInformed {
    type CostEstimate = f64;
    type InformedError = NoError;
    fn estimate_remaining_cost(
        &self,
        from_state: &f64,
        to_goal: &f64,
    ) -> Result<Option<f64>, NoError> {
        Ok(Some((to_goal - from_state).abs()))
    }
}

#[derive(Clone, Default)]
struct MyRing;
impl Keyed for MyRing {
    type Key = u64;
}
impl Keyring<f64> for MyRing {
    type KeyRef<'a>
        = u64
    where
        Self: 'a,
        f64: 'a;
    fn key_for<'a>(&'a self, state: &'a f64) -> Self::KeyRef<'a>
    where
        Self: 'a,
        f64: 'a,
    {
        *state as u64
    }
}

fn main() {
    let domain = RobotDomain {
        motion: MyActivity,
        cost: MyWeighted,
        heuristic: MyInformed,
        closer: KeyedCloser(MyRing),
    };

    // Test delegation
    let choices: Vec<_> = domain.choices(0.0).collect();
    assert_eq!(choices[0].as_ref().unwrap().1, 1.0);

    let cost = domain.cost(&0.0, &1.0, &1.0).unwrap().unwrap();
    assert_eq!(cost, 1.0);

    let estimate = domain
        .estimate_remaining_cost(&0.0, &10.0)
        .unwrap()
        .unwrap();
    assert_eq!(estimate, 10.0);

    println!("Delegation worked!");
}
