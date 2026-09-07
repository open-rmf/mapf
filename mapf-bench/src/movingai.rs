/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

//! This module contains a simple parser for the movingai benchmarks.
//! This benchmark is widely seen as the de-facto mapf benchmark.
//!
//! The benchmarks consist of *.map files and *.scen files. The *.map
//! files are occupancy grids while the *.scen files contain
//! exact mapf scenarios.

use anyhow::Result;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use mapf::negotiation::{Agent, Scenario};
use std::collections::BTreeMap;

pub struct Map {
    pub width: usize,
    pub height: usize,
    /// The grid representation of the map.
    ///
    /// The characters in the grid represent different terrain types based on the Moving AI format:
    /// - `.` : Passable terrain
    /// - `@` : Obstacle
    /// Full format: https://www.movingai.com/benchmarks/formats.html
    pub grid: Vec<Vec<char>>,
}

impl Map {
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let mut lines = reader.lines();

        let mut width = 0;
        let mut height = 0;

        // Parse header
        while let Some(line) = lines.next() {
            let line = line?;
            if line.starts_with("type") {
                continue;
            } else if line.starts_with("height") {
                height = line.split_whitespace().nth(1).unwrap().parse()?;
            } else if line.starts_with("width") {
                width = line.split_whitespace().nth(1).unwrap().parse()?;
            } else if line.starts_with("map") {
                break;
            }
        }

        let mut grid = Vec::with_capacity(height);
        for _ in 0..height {
            if let Some(line) = lines.next() {
                let line = line?;
                grid.push(line.chars().collect());
            }
        }

        Ok(Map {
            width,
            height,
            grid,
        })
    }

    pub fn to_occupancy_map(&self) -> HashMap<i64, Vec<i64>> {
        let mut occupancy = HashMap::new();
        for y in 0..self.height {
            let mut row = Vec::new();
            for x in 0..self.width {
                let c = self.grid[y][x];
                if c != '.' && c != 'G' && c != 'S' {
                    row.push(x as i64);
                }
            }
            if !row.is_empty() {
                occupancy.insert(y as i64, row);
            }
        }
        occupancy
    }
}

pub struct ScenarioEntry {
    pub _bucket: usize,
    pub _map_file: String,
    pub _map_width: usize,
    pub _map_height: usize,
    pub start: [i64; 2],
    pub goal: [i64; 2],
    pub _optimal_length: f64,
}

pub struct MovingAIScenario {
    pub entries: Vec<ScenarioEntry>,
}

impl MovingAIScenario {
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let mut lines = reader.lines();

        // Skip version line
        lines.next();

        let mut entries = Vec::new();
        for line in lines {
            let line = line?;
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 9 {
                continue;
            }

            let start: [i64; 2] = [parts[4].parse()?, parts[5].parse()?];
            let goal: [i64; 2] = [parts[6].parse()?, parts[7].parse()?];

            entries.push(ScenarioEntry {
                _bucket: parts[0].parse()?,
                _map_file: parts[1].to_string(),
                _map_width: parts[2].parse()?,
                _map_height: parts[3].parse()?,
                start,
                goal,
                _optimal_length: parts[8].parse()?,
            });
        }

        Ok(MovingAIScenario { entries })
    }

    pub fn to_negotiation_scenario(
        &self,
        map: &Map,
        num_agents: usize,
        radius: f64,
        speed: f64,
        spin: f64,
    ) -> Scenario {
        let mut agents = BTreeMap::new();
        for i in 0..num_agents.min(self.entries.len()) {
            let entry = &self.entries[i];
            agents.insert(
                format!("agent_{}", i),
                Agent {
                    start: entry.start,
                    yaw: 0.0,
                    goal: entry.goal,
                    radius,
                    speed,
                    spin,
                },
            );
        }

        Scenario {
            agents,
            obstacles: Vec::new(),
            occupancy: map.to_occupancy_map(),
            cell_size: 1.0,
            camera_bounds: None,
        }
    }
}
