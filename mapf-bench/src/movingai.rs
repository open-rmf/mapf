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
    pub bucket: usize,
    pub map_file: String,
    pub map_width: usize,
    pub map_height: usize,
    pub start_x: usize,
    pub start_y: usize,
    pub goal_x: usize,
    pub goal_y: usize,
    pub optimal_length: f64,
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

            entries.push(ScenarioEntry {
                bucket: parts[0].parse()?,
                map_file: parts[1].to_string(),
                map_width: parts[2].parse()?,
                map_height: parts[3].parse()?,
                start_x: parts[4].parse()?,
                start_y: parts[5].parse()?,
                goal_x: parts[6].parse()?,
                goal_y: parts[7].parse()?,
                optimal_length: parts[8].parse()?,
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
                    start: [entry.start_x as i64, entry.start_y as i64],
                    yaw: 0.0,
                    goal: [entry.goal_x as i64, entry.goal_y as i64],
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
