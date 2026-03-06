use std::collections::{HashMap, HashSet};

use bresenham::Bresenham;

use super::{
    IntersectionType, LeaderFollowerZones, MapfResult, SemanticPlan, SemanticWaypoint, mapf_post,
};

#[derive(Clone)]
pub struct CurrentPosition {
    pub semantic_position: SemanticWaypoint,
    pub real_position: (f64, f64),
}
#[derive(Clone)]
pub struct Grid2D {
    static_obstacles: Vec<Vec<i8>>,
    cell_size: f64,
    width: usize,
    height: usize,
}

impl Grid2D {
    pub fn new(static_obstacles: Vec<Vec<i8>>, cell_size: f64) -> Self {
        let width = static_obstacles.len();
        let height = if width > 0 {
            static_obstacles[0].len()
        } else {
            0
        };
        Self {
            static_obstacles,
            cell_size,
            width,
            height,
        }
    }

    pub fn to_world_coords(&self, x: isize, y: isize) -> (f64, f64) {
        (
            self.cell_size * x as f64,
            self.cell_size * (self.height as isize - 1 - y) as f64,
        )
    }

    pub fn from_world_coords(&self, x: f64, y: f64) -> (isize, isize) {
        (
            (x / self.cell_size) as isize,
            (self.height as isize - 1 - (y / self.cell_size) as isize),
        )
    }

    pub fn allocate_trajectory(
        &self,
        trajectories: &MapfResult,
        positions: &[CurrentPosition],
    ) -> AllocationField {
        let semantic_plan = mapf_post(trajectories);
        let semantic_positions: Vec<SemanticWaypoint> = positions.iter().map(|p| p.semantic_position).collect();
        let assignment = semantic_plan.get_claim_dict(&semantic_positions);

        let mut allocation_field = AllocationField::create(semantic_plan, 1000, 1000);

        for (agent, traj) in trajectories.trajectories.iter().enumerate() {
            let Some(region) = assignment.get(&agent) else {
                panic!("Unknown Agent - something went wrong in get_claim_dict");
            };
            for i in region.start_id..region.end_id + 1 {
                if i >= traj.poses.len() {
                    continue;
                }

                //TODO(arjoc) Add a function parameter getting robot's current real position when i = start_id
                let start_pose = traj.poses[i];
                let start_coords =
                    self.from_world_coords(start_pose.translation.x, start_pose.translation.y);
                allocation_field.mark(
                    start_coords.0,
                    start_coords.1,
                    &TrajectoryAllocation {
                        agent,
                        priority: i,
                        dist_from_center: 0.0,
                    },
                );
                /*self.blur(&TrajectoryAllocation {
                    agent,
                    priority: i,
                    dist_from_center: 0.0
                }, &start_coords, &mut allocation_field, 1.0);*/
                if i + 1 >= traj.poses.len() {
                    continue;
                }

                let end_pose = traj.poses[i + 1];
                let end_coords =
                    self.from_world_coords(end_pose.translation.x, end_pose.translation.y);

                for (x, y) in Bresenham::new(start_coords, end_coords) {
                    allocation_field.mark(
                        x,
                        y,
                        &TrajectoryAllocation {
                            agent,
                            priority: i,
                            dist_from_center: 0.0,
                        },
                    );
                    self.blur(
                        &TrajectoryAllocation {
                            agent,
                            priority: i,
                            dist_from_center: 0.0,
                        },
                        &(x, y),
                        &mut allocation_field,
                        1.0,
                    );
                }
            }
        }
        allocation_field
    }

    fn blur(
        &self,
        trajectory_alloc: &TrajectoryAllocation,
        start_cell: &(isize, isize),
        grid_space: &mut AllocationField,
        max_dist: f64,
    ) {
        let mut stack = vec![(*start_cell, 0.0f64)];
        let mut visited = HashSet::new();
        grid_space.mark(start_cell.0, start_cell.1, trajectory_alloc);
        while let Some(((x, y), dist)) = stack.pop() {
            for i in -1..=1 {
                for j in -1..=1 {
                    if x + i < 0 || y + j < 0 {
                        continue;
                    }
                    let nx = (x + i) as usize;
                    let ny = (y + j) as usize;

                    if visited.contains(&(nx, ny)) {
                        continue;
                    }

                    if nx >= self.static_obstacles.len() {
                        continue;
                    }

                    if ny >= self.static_obstacles[nx].len() {
                        continue;
                    }

                    if self.static_obstacles[nx][ny] == 100 {
                        continue;
                    }

                    if dist + 1.0 > max_dist {
                        continue;
                    }
                    stack.push(((nx as isize, ny as isize), dist + 1.0));
                    let alloc = TrajectoryAllocation {
                        agent: trajectory_alloc.agent,
                        priority: trajectory_alloc.priority,
                        dist_from_center: dist + 1.0,
                    };
                    grid_space.mark(nx as isize, ny as isize, &alloc);
                    visited.insert((nx, ny));
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrajectoryAllocation {
    agent: usize,
    priority: usize,
    dist_from_center: f64,
}

impl TrajectoryAllocation {
    fn to_wp(&self) -> SemanticWaypoint {
        SemanticWaypoint {
            agent: self.agent,
            trajectory_index: self.priority,
        }
    }
}

/// An allocation field. Each point on the grid is allocated to a specific robot
/// based on its original MAPF plan
pub struct AllocationField {
    grid_space: Vec<Vec<Option<TrajectoryAllocation>>>,
    plan: SemanticPlan,
    leader_follower: LeaderFollowerZones,
    cell_by_agent: HashMap<usize, HashSet<(usize, usize)>>,
}

impl AllocationField {
    fn create(plan: SemanticPlan, width: usize, height: usize) -> Self {
        let leader_follower = plan.figure_out_leader_follower_zone();
        Self {
            grid_space: vec![vec![None; height]; width],
            plan,
            leader_follower,
            cell_by_agent: HashMap::default(),
        }
    }

    fn width(&self) -> usize {
        self.grid_space.len()
    }

    fn height(&self) -> usize {
        if self.width() == 0 {
            0
        } else {
            self.grid_space[0].len()
        }
    }

    fn update_spot(&mut self, alloc: &TrajectoryAllocation, x: usize, y: usize) {
        if let Some(prev_alloc) = &self.grid_space[x][y]
            && let Some(p) = self.cell_by_agent.get_mut(&prev_alloc.agent) {
                p.remove(&(x, y));
            }
        self.grid_space[x][y] = Some(alloc.clone());
        if let Some(allocated_list) = self.cell_by_agent.get_mut(&alloc.agent) {
            allocated_list.insert((x, y));
        } else {
            self.cell_by_agent
                .insert(alloc.agent, HashSet::from_iter([(x, y)].iter().cloned()));
        }
    }

    fn mark(&mut self, x: isize, y: isize, alloc: &TrajectoryAllocation) {
        let x = x as usize;
        let y = y as usize;

        if x >= self.width() {
            return;
        }

        if y >= self.height() {
            return;
        }

        let Some(previous_alloc) = self.grid_space[x][y].clone() else {
            self.update_spot(alloc, x, y);
            return;
        };

        // We keep reservations from earlier times first
        if previous_alloc.priority < alloc.priority {
            // Existing (earlier) reservation wins; leave it unchanged.
            return;
        }
        // In the event that a reservation is of the same priority/time step
        // we need to apply rules for allocation.
        // In the event the two agents have no relationship at that time step
        // partition the space by who ever is nearer.
        // In the event the relationship is a leader follower one give priority
        // to the leader.
        // In the event the relationship is an intersection based relationship,
        // give priority to the agent that is leaving the junction.
        else if previous_alloc.priority == alloc.priority {
            // Follower logic for tie-breaking.
            if let Some((_mode, priority, cluster_id)) = self
                .leader_follower
                .allocation_strategy
                .get(&previous_alloc.to_wp())
                && let Some((_mode2, priority2, cluster_id2)) =
                    self.leader_follower.allocation_strategy.get(&alloc.to_wp())
                    && cluster_id == cluster_id2 {
                        if priority <= priority2 {
                            self.grid_space[x][y] = Some(previous_alloc.clone());
                            return;
                        } else {
                            self.update_spot(alloc, x, y);
                            return;
                        }
                    }

            // Logic for intersections
            if let Some(intersection_type) = self
                .plan
                .is_intersection_participant(&previous_alloc.to_wp())
            {
                if let IntersectionType::Lead(followers) = intersection_type {
                    if followers.contains(&alloc.to_wp()) {
                        self.grid_space[x][y] = Some(alloc.clone());
                        return;
                    }
                } else if let IntersectionType::Next(leaders) = intersection_type
                    && leaders.contains(&alloc.to_wp()) {
                        self.update_spot(&previous_alloc, x, y);
                        return;
                    }
            }
            // Vicinity rule.
            if previous_alloc.dist_from_center < alloc.dist_from_center {
                self.grid_space[x][y] = Some(previous_alloc.clone());
            } else {
                self.update_spot(alloc, x, y);
            }
        } else {
            self.update_spot(alloc, x, y);
        }
    }

    pub fn get_allocation(&self, x: isize, y: isize) -> Option<usize> {
        if x < 0 || y < 0 {
            return None;
        }
        let x = x as usize;
        let y = y as usize;
        
        if x >= self.width() || y >= self.height() {
            return None;
        }
        
        let Some(p) = self.grid_space[x][y].clone() else {
            return None;
        };
        Some(p.agent)
    }

    pub fn get_alloc_for_agent(&self, agent: usize) -> Option<Vec<(usize, usize)>> {
        self.cell_by_agent
            .get(&agent)
            .map(|p| Vec::from_iter(p.iter().cloned()))
    }

    pub fn get_alloc_priority(&self, x: isize, y: isize) -> Option<usize> {
        if x < 0 || y < 0 {
            return None;
        }
        let x = x as usize;
        let y = y as usize;
        
        if x >= self.width() || y >= self.height() {
            return None;
        }
        
        let Some(p) = self.grid_space[x][y].clone() else {
            return None;
        };
        Some(p.priority)
    }
}
