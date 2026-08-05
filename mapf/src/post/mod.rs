use core::panic;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use parry2d::bounding_volume::{Aabb, BoundingVolume};
use parry2d::na::{Isometry2, Point2};
use parry2d::query::cast_shapes_nonlinear;
use parry2d::query::{NonlinearRigidMotion, ShapeCastStatus};
use parry2d::shape::Shape;

pub use parry2d::na;
pub use parry2d::shape;
use petgraph::algo::toposort;
use petgraph::graph::DiGraph;
use petgraph::visit::EdgeRef;

pub mod spatial_allocation;

/// A simple class to estimate a Robot's Semantic position based on its location
/// on a trajectory. It assumes you start near the start of the robot trajectory.
pub struct WaypointFollower {
    trajectory: Trajectory,
    current_pose_on_trajectory: usize,
    agent_id: usize,
}

impl WaypointFollower {
    /// Construct the follower for `agent_id` based on a trajectory
    pub fn from_trajectory(agent_id: usize, trajectory: Trajectory) -> Self {
        Self {
            trajectory,
            current_pose_on_trajectory: 0,
            agent_id,
        }
    }

    /// Update position of waypoint follower by finding the closest point on the trajectory.
    ///
    /// This avoids getting stuck if a waypoint is missed, by projecting the robot's position
    /// onto the trajectory segments.
    pub fn update_position_estimate(&mut self, pose: &Isometry2<f64>, _uncertainty: f64) {
        if self.current_pose_on_trajectory >= self.trajectory.poses.len().saturating_sub(1) {
            return;
        }

        let robot_pos = Point2::from(pose.translation.vector);

        let mut closest_dist_sq = f64::MAX;
        let mut best_segment_start_idx = self.current_pose_on_trajectory;

        // Iterate from the current segment onwards to find the closest segment.
        // This prevents the follower from going backwards along the trajectory.
        for i in self.current_pose_on_trajectory..self.trajectory.poses.len().saturating_sub(1) {
            let p1 = Point2::from(self.trajectory.poses[i].translation.vector);
            let p2 = Point2::from(self.trajectory.poses[i + 1].translation.vector);
            let segment_vec = p2 - p1;
            let robot_to_p1 = robot_pos - p1;

            let t = if segment_vec.norm_squared() > 1e-6 {
                (robot_to_p1.dot(&segment_vec)) / segment_vec.norm_squared()
            } else {
                0.0 // Handle zero-length segments
            };

            let closest_point_on_segment = if t < 0.0 {
                p1
            } else if t > 1.0 {
                p2
            } else {
                p1 + t * segment_vec
            };

            let dist_sq = na::distance_squared(&robot_pos, &closest_point_on_segment);

            if dist_sq < closest_dist_sq {
                closest_dist_sq = dist_sq;

                // If the robot is at or past the end of the segment, it has passed this segment's start waypoint.
                if t >= 1.0 {
                    best_segment_start_idx = i + 1;
                } else {
                    best_segment_start_idx = i;
                }
            }
        }
        self.current_pose_on_trajectory = best_segment_start_idx;
    }

    /// Gives the next waypoint we should consider.
    pub fn next_waypoint(&mut self) -> Isometry2<f64> {
        if self.current_pose_on_trajectory + 1 >= self.trajectory.poses.len() {
            return *self.trajectory.poses.last().unwrap();
        }
        self.trajectory.poses[self.current_pose_on_trajectory + 1]
    }

    /// Returns a list of (x,y)  coordinates left for the robot to go through
    pub fn remaining_trajectory(&mut self) -> Vec<(f64, f64)> {
        let mut v = vec![];
        for p in self.current_pose_on_trajectory + 1..self.trajectory.poses.len() {
            v.push((
                self.trajectory.poses[p].translation.x,
                self.trajectory.poses[p].translation.y,
            ));
        }
        v
    }

    /// Returns the list of (x,y) coordinates that are currently safe for the robot to traverse
    pub fn remaining_safe_trajectory_segment(
        &mut self,
        seg: &CurrentlyAllocatedTrajSegment,
    ) -> Vec<(f64, f64)> {
        let mut v = vec![];
        for p in self.current_pose_on_trajectory + 1..seg.end_id {
            v.push((
                self.trajectory.poses[p].translation.x,
                self.trajectory.poses[p].translation.y,
            ));
        }
        v
    }

    /// Returns the current semantic waypoint
    pub fn get_semantic_waypoint(&mut self) -> SemanticWaypoint {
        SemanticWaypoint {
            agent: self.agent_id,
            trajectory_index: self.current_pose_on_trajectory,
        }
    }
}

#[cfg(test)]
#[test]
fn test_waypoint_follower() {
    use parry2d::na::Vector2;

    let example_trajectory = Trajectory {
        poses: vec![
            Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
        ],
    };

    let mut follower = WaypointFollower::from_trajectory(0, example_trajectory.clone());
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[1].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 0);
    // Now lets move closer to the target but not reach it.
    follower.update_position_estimate(&Isometry2::new(Vector2::new(0.1, 0.0), 0.0), 0.1);

    // We should still be at the previous location
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[1].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 0);

    // Lets update our position to be close to the next waypoint, but not past it.
    follower.update_position_estimate(&Isometry2::new(Vector2::new(0.95, 0.0), 0.0), 0.1);
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[1].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 0);

    // Let's reach the next waypoint.
    follower.update_position_estimate(&Isometry2::new(Vector2::new(1.0, 0.0), 0.0), 0.1);
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[2].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 1);

    // Lets update our position to be close to the final pose.
    follower.update_position_estimate(&Isometry2::new(Vector2::new(1.95, 0.0), 0.0), 0.1);
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[2].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 1);

    // Let's reach the final pose
    follower.update_position_estimate(&Isometry2::new(na::Vector2::new(2.0, 0.0), 0.0), 0.1);
    let next_wp = follower.next_waypoint();
    assert!((next_wp.translation.x - example_trajectory.poses[2].translation.x).abs() < 0.01);
    let semantic_pose = follower.get_semantic_waypoint();
    assert_eq!(semantic_pose.trajectory_index, 2);
}

#[cfg(test)]
#[test]
fn test_trajectory_conversion() {
    use crate::motion::se2::WaypointSE2;
    use crate::motion::Trajectory as MapfTrajectory;
    use time_point::TimePoint;

    let w1 = WaypointSE2::new(TimePoint::from_secs_f64(0.0), 0.0, 0.0, 0.0);
    let w2 = WaypointSE2::new(TimePoint::from_secs_f64(1.0), 1.0, 0.0, 0.0);
    let mapf_traj = MapfTrajectory::new(w1, w2).unwrap();

    let post_traj = Trajectory::from(&mapf_traj);
    assert_eq!(post_traj.len(), 2);
    assert!((post_traj.poses[0].translation.x - 0.0).abs() < 1e-6);
    assert!((post_traj.poses[1].translation.x - 1.0).abs() < 1e-6);
}

/// Semantic waypoint: A point on the trajectory that needs to be passed.

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct SemanticWaypoint {
    pub agent: usize,
    pub trajectory_index: usize,
}

/// These are error codes for safe next state retrieval
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
#[non_exhaustive]
pub enum SafeNextStatesError {
    /// An incorreect number of agents were added
    NumberOfAgentsNotMatching,
    /// The semantic state is incorrect. Probably the same agent is reporting state twice.
    InvalidSemanticState,
    /// Agent was not found. Probably a different agent was reporting stete at different times.
    AgentNotFound,
    /// Report a bug if this ever ever crops up
    InternalStateMisMatch,
}

/// Semantic Plan represents the plan in terms of SemanticWaypoints
#[derive(Clone, Debug, Default)]
pub struct SemanticPlan {
    /// Number of agents
    pub num_agents: usize,
    /// These serve as graph nodes, describing each waypoint
    pub waypoints: Vec<SemanticWaypoint>,
    /// For book keeping
    agent_time_to_wp_id: HashMap<SemanticWaypoint, usize>,
    /// Key comes after all of Values
    depends_on_all_of: HashMap<usize, Vec<usize>>,
    /// Next states. Key is the state
    potential_successors: HashMap<usize, Vec<usize>>,
    /// Mapping from agent name to ID
    pub agent_name_to_id: HashMap<String, usize>,
}

impl SemanticPlan {
    /// Get the agent ID from its name
    pub fn get_agent_id(&self, name: &str) -> Option<usize> {
        self.agent_name_to_id.get(name).copied()
    }

    fn add_waypoint(&mut self, waypoint: &SemanticWaypoint) {
        self.agent_time_to_wp_id
            .insert(*waypoint, self.waypoints.len());
        self.waypoints.push(*waypoint);
    }

    fn last_time(&self) -> usize {
        self.waypoints
            .iter()
            .map(|wp| wp.trajectory_index)
            .max()
            .unwrap_or(0)
    }

    /// Use this to add an edge
    fn requires_comes_after(&mut self, after: &SemanticWaypoint, before: &SemanticWaypoint) {
        let Some(after_id) = self.agent_time_to_wp_id.get(before) else {
            return;
        };
        let Some(before_id) = self.agent_time_to_wp_id.get(after) else {
            return;
        };

        let Some(to_vals) = self.depends_on_all_of.get_mut(after_id) else {
            self.depends_on_all_of.insert(*after_id, vec![*before_id]);
            return;
        };
        to_vals.push(*before_id);

        let Some(to_vals) = self.potential_successors.get_mut(before_id) else {
            self.potential_successors
                .insert(*before_id, vec![*after_id]);
            return;
        };
        to_vals.push(*after_id);
    }

    /// Given a Semantic State Estimate, determine if its safe for the current agent to proceed
    /// The state estimate should consist of each agents semantic waypoint
    /// If there is a mismatch between the number of agents and number of
    pub fn is_safe_to_proceed(
        &self,
        current_state: &[SemanticWaypoint],
        agent: usize,
    ) -> Result<bool, SafeNextStatesError> {
        // Lots of O(n^2) behaviour that should really be O(n) or O(1) because our function signature takes current semantic state as a vec
        // TODO(arjo): add a new CurrentSemanticState struct. Require that it is a sorted vec or structure with correct
        if current_state.len() != self.num_agents {
            return Err(SafeNextStatesError::NumberOfAgentsNotMatching);
        }

        // Get the agents current waypoint
        let agent_curr_waypoint: Vec<_> =
            current_state.iter().filter(|x| x.agent == agent).collect();
        if agent_curr_waypoint.len() > 1 {
            return Err(SafeNextStatesError::InvalidSemanticState);
        }
        if agent_curr_waypoint.is_empty() {
            return Err(SafeNextStatesError::AgentNotFound);
        }

        // Try to get the next waypoint for the agent
        let Some(agent_next_waypoint) = self.get_next_for_agent(agent_curr_waypoint[0]) else {
            return Ok(false);
        };

        // Find out what dependencies the waypoint should come after
        let Some(waypoints_that_should_have_been_crossed) = self.comes_before(&agent_next_waypoint)
        else {
            return Err(SafeNextStatesError::InternalStateMisMatch);
        };

        let waypoints_that_should_have_been_crossed = waypoints_that_should_have_been_crossed
            .iter()
            .map(|wp_id| self.waypoints[*wp_id]);

        // Now for each waypoint find out if the other robot has crossed it
        let res = waypoints_that_should_have_been_crossed
            .filter(|wp| {
                let agent_to_check = wp.agent;
                let agent_curr_loc: Vec<_> = current_state
                    .iter()
                    .filter(|x| x.agent == agent_to_check)
                    .collect();
                if agent_to_check == agent {
                    return false;
                }
                if agent_curr_loc.len() > 1 {
                    panic!("malformed")
                }
                if agent_curr_loc.is_empty() {
                    panic!("Made-up agent")
                }
                if agent_curr_loc[0].trajectory_index <= wp.trajectory_index {
                    return true;
                }
                false
            })
            .count();

        Ok(res == 0)
    }

    fn get_next_for_agent(&self, waypoint: &SemanticWaypoint) -> Option<SemanticWaypoint> {
        let mut waypoint = *waypoint;
        waypoint.trajectory_index += 1;
        self.agent_time_to_wp_id
            .get(&waypoint)
            .map(|p| self.waypoints[*p])
    }

    /// Check which waypoints should come before the given waypoint
    pub fn comes_before(&self, waypoint: &SemanticWaypoint) -> Option<&Vec<usize>> {
        let Some(waypoint_id) = self.agent_time_to_wp_id.get(waypoint) else {
            return None;
        };

        self.depends_on_all_of.get(waypoint_id)
    }

    /// Returns a Pet-graph directed graph for further manipulation of the entire
    /// semantic plan
    pub fn to_petgraph(&self) -> DiGraph<SemanticWaypoint, ()> {
        let mut digraph = DiGraph::new();
        let mut node_index_map = HashMap::new();
        for waypoint in &self.waypoints {
            let node_index = digraph.add_node(*waypoint);
            node_index_map.insert(waypoint, node_index);
        }
        for (after, befores) in &self.depends_on_all_of {
            for before in befores {
                let Some(after) = self.waypoints.get(*after) else {
                    continue;
                };
                let Some(before) = self.waypoints.get(*before) else {
                    continue;
                };
                let Some(&after) = node_index_map.get(after) else {
                    continue;
                };
                let Some(&before) = node_index_map.get(before) else {
                    continue;
                };
                digraph.add_edge(before, after, ());
            }
        }
        digraph
    }

    /// Returns the traffic dependencies at the current time.
    pub fn get_current_cut(&self, current_state: &[SemanticWaypoint]) -> Self {
        let mut max_time_stamp = 0;

        // the minimum hashmap contains the lower time bound based on the
        let mut minimum = HashMap::new();
        for agent in current_state {
            max_time_stamp = agent.trajectory_index.max(max_time_stamp);
            minimum.insert(agent.agent, agent.trajectory_index);
        }

        let mut semantic_graph = Self::default();
        for waypoint in &self.waypoints {
            let Some(&minimum_wp) = minimum.get(&waypoint.agent) else {
                continue;
            };
            if waypoint.trajectory_index < minimum_wp {
                continue;
            }
            if waypoint.trajectory_index > max_time_stamp {
                continue;
            }
            semantic_graph.add_waypoint(waypoint);
        }
        for (after, befores) in &self.depends_on_all_of {
            for before in befores {
                let Some(after) = self.waypoints.get(*after) else {
                    continue;
                };
                let Some(before) = self.waypoints.get(*before) else {
                    continue;
                };

                if !semantic_graph.agent_time_to_wp_id.contains_key(after) {
                    continue;
                }

                if !semantic_graph.agent_time_to_wp_id.contains_key(before) {
                    continue;
                }

                semantic_graph.requires_comes_after(after, before);
            }
        }
        semantic_graph
    }

    /// Returns the traffic dependencies at the current time.
    pub fn current_traffic_deps(
        &self,
        current_state: &[SemanticWaypoint],
    ) -> DiGraph<SemanticWaypoint, ()> {
        let mut max_time_stamp = 0;
        let mut minimum = HashMap::new();
        for agent in current_state {
            max_time_stamp = agent.trajectory_index.max(max_time_stamp);
            minimum.insert(agent.agent, agent.trajectory_index);
        }

        let mut digraph = DiGraph::new();
        let mut node_index_map = HashMap::new();
        for waypoint in &self.waypoints {
            let Some(&minimum_wp) = minimum.get(&waypoint.agent) else {
                continue;
            };
            if waypoint.trajectory_index < minimum_wp {
                continue;
            }
            if waypoint.trajectory_index > max_time_stamp {
                continue;
            }
            let node_index = digraph.add_node(*waypoint);
            node_index_map.insert(waypoint, node_index);
        }
        for (after, befores) in &self.depends_on_all_of {
            for before in befores {
                let Some(after) = self.waypoints.get(*after) else {
                    continue;
                };
                let Some(before) = self.waypoints.get(*before) else {
                    continue;
                };
                let Some(&after) = node_index_map.get(after) else {
                    continue;
                };
                let Some(&before) = node_index_map.get(before) else {
                    continue;
                };
                digraph.add_edge(before, after, ());
            }
        }
        digraph
    }

    /// Generate a DOT representation of the graph for visualization
    /// Also colors the current state of the world.
    pub fn to_dot_with_results(&self, waypoints: &[SemanticWaypoint]) -> String {
        let mut dot = String::from("digraph SemanticPlan {\n");
        let mut color = "white";
        for (id, waypoint) in self.waypoints.iter().enumerate() {
            for wp in waypoints {
                if wp.agent == waypoint.agent {
                    if wp.trajectory_index < waypoint.trajectory_index {
                        color = "lightyellow";
                    } else if wp.trajectory_index == waypoint.trajectory_index {
                        color = "lightgreen";
                    }
                }
            }
            dot.push_str(&format!(
                "    {} [label=\"Agent: {}, Index: {}\", color = \"{}\"];\n",
                id, waypoint.agent, waypoint.trajectory_index, color
            ));
        }
        for (after, befores) in &self.depends_on_all_of {
            for before in befores {
                dot.push_str(&format!("    {} -> {};\n", before, after));
            }
        }
        dot.push_str("}\n");
        dot
    }
    /// Generate a DOT representation of the graph for visualization
    /// This is more for debugging than for actual use
    pub fn to_dot(&self) -> String {
        let mut dot = String::from("digraph SemanticPlan {\n");
        for (id, waypoint) in self.waypoints.iter().enumerate() {
            dot.push_str(&format!(
                "    {} [label=\"Agent: {}, Index: {}\"];\n",
                id, waypoint.agent, waypoint.trajectory_index
            ));
        }
        for (after, befores) in &self.depends_on_all_of {
            for before in befores {
                dot.push_str(&format!("    {} -> {};\n", before, after));
            }
        }
        dot.push_str("}\n");
        dot
    }

    /// Check if node participates in intersection.
    pub fn is_intersection_participant(
        &self,
        waypoint: &SemanticWaypoint,
    ) -> Option<IntersectionType> {
        let Some(wp_id) = self.agent_time_to_wp_id.get(waypoint) else {
            return None;
        };

        let Some(p) = self.depends_on_all_of.get(wp_id) else {
            return None;
        };

        let Some(q) = self.potential_successors.get(wp_id) else {
            return None;
        };
        if self.is_follower(waypoint).is_empty() {
            if p.len() > 1 {
                let mut children = HashSet::new();
                for &wp in p {
                    let wp = self.waypoints[wp];
                    if wp.agent == waypoint.agent {
                        continue;
                    }
                    children.insert(wp);
                }
                return Some(IntersectionType::Next(children));
            }
            if q.len() > 1 {
                let mut children = HashSet::new();
                for &wp in p {
                    let wp = self.waypoints[wp];
                    if wp.agent == waypoint.agent {
                        continue;
                    }
                    children.insert(wp);
                }
                return Some(IntersectionType::Lead(children));
            }
        }
        None
    }

    /// Returns if the waypoint is participating in a follower behaviour
    pub fn is_follower(&self, waypoint: &SemanticWaypoint) -> Vec<SemanticWaypoint> {
        if waypoint.trajectory_index + 1 > self.last_time() {
            // TODO(arjoc) We cant lead when weve reached the end. But we could
            // be a follower. We need to update the check in a way that supports this behaviour
            return vec![];
        }

        let mut next_waypoint = *waypoint;
        next_waypoint.trajectory_index += 1;

        let Some(wp_id) = self.agent_time_to_wp_id.get(waypoint) else {
            return vec![];
        };
        let Some(next_wp_id) = self.agent_time_to_wp_id.get(&next_waypoint) else {
            return vec![];
        };

        let Some(t_deps) = self.depends_on_all_of.get(wp_id) else {
            return vec![];
        };

        let Some(t_deps_next) = self.depends_on_all_of.get(next_wp_id) else {
            return vec![];
        };

        let mut depends_on = HashMap::new();
        for potential_follower in t_deps {
            if self.waypoints[*potential_follower].agent == waypoint.agent {
                continue;
            }

            let Some(followers) = depends_on.get_mut(&self.waypoints[*potential_follower].agent)
            else {
                depends_on.insert(
                    self.waypoints[*potential_follower].agent,
                    vec![self.waypoints[*potential_follower].trajectory_index],
                );
                continue;
            };

            followers.push(self.waypoints[*potential_follower].trajectory_index);
        }

        let mut next_depends_on = HashMap::new();
        for potential_follower in t_deps_next {
            if self.waypoints[*potential_follower].agent == waypoint.agent {
                continue;
            }

            let Some(followers) =
                next_depends_on.get_mut(&self.waypoints[*potential_follower].agent)
            else {
                next_depends_on.insert(
                    self.waypoints[*potential_follower].agent,
                    vec![self.waypoints[*potential_follower].trajectory_index],
                );
                continue;
            };
            followers.push(self.waypoints[*potential_follower].trajectory_index);
        }

        let mut followers = vec![];
        for (agent, times) in depends_on {
            let Some(prev_times) = next_depends_on.get(&agent) else {
                continue;
            };

            if times.iter().max() <= prev_times.clone().iter().max() {
                followers.push(SemanticWaypoint {
                    agent,
                    trajectory_index: *times.iter().max().unwrap(),
                });
            }
        }

        followers
    }

    pub fn figure_out_leader_follower_zone(&self) -> LeaderFollowerZones {
        let mut follow_graph = DiGraph::new();

        let mut wp_to_nodeid = HashMap::new();
        for wp in &self.waypoints {
            let node_id = follow_graph.add_node(*wp);
            wp_to_nodeid.insert(*wp, node_id);
        }

        // Determine follower relationship
        for wp in &self.waypoints {
            let vec = self.is_follower(wp);
            for leader in vec {
                let Some(leader) = wp_to_nodeid.get(&leader) else {
                    continue;
                };
                let Some(follower) = wp_to_nodeid.get(wp) else {
                    continue;
                };
                follow_graph.add_edge(*leader, *follower, ());
            }
        }

        // At each timestep cluster the follower graphs
        let p = cluster(&follow_graph);
        let p: Vec<_> = p
            .iter()
            .map(|(k, v)| {
                (
                    *k,
                    v.iter()
                        .map(|node_index| follow_graph[*node_index])
                        .collect::<Vec<_>>(),
                )
            })
            .collect();

        let mut wp_to_cluster: HashMap<SemanticWaypoint, usize> = HashMap::new();
        for (cluster_id, agent_waypoints) in &p {
            for agent_waypoint in agent_waypoints {
                wp_to_cluster.insert(*agent_waypoint, *cluster_id);
            }
        }

        // Then for each cluster perform a topo-sort and figure out who is the leader
        let mut cluster_to_leader = HashMap::new();
        let mut leader_to_cluster = HashMap::new();

        let mut allocation_strategy: HashMap<SemanticWaypoint, (AllocationStrategy, usize, usize)> =
            HashMap::new();

        for (cluster_id, agent_waypoint) in &p {
            let mut sub_graph = DiGraph::new();
            let mut wp_to_nodeid = HashMap::new();
            for agent_waypoint in agent_waypoint {
                let node_id = sub_graph.add_node(*agent_waypoint);
                wp_to_nodeid.insert(*agent_waypoint, node_id);
            }

            for edge in follow_graph.edge_references() {
                let target = follow_graph[edge.target()];
                let source = follow_graph[edge.source()];

                let Some(source_id) = wp_to_nodeid.get(&source) else {
                    continue;
                };

                let Some(target_id) = wp_to_nodeid.get(&target) else {
                    continue;
                };

                sub_graph.add_edge(*source_id, *target_id, ());
            }

            // Extract prime leader
            let ts = toposort(&sub_graph, None);
            let ts: Vec<_> = ts.unwrap().iter().map(|v| sub_graph[*v]).collect();
            if let Some(&leader) = ts.first() {
                if ts.len() > 1 {
                    cluster_to_leader.insert(cluster_id, leader);
                    leader_to_cluster.insert(leader, cluster_id);
                    allocation_strategy.insert(
                        ts[0],
                        (AllocationStrategy::Leader(0.0, 0.0), 0, *cluster_id),
                    );
                    for i in 1..ts.len() {
                        allocation_strategy.insert(
                            ts[i],
                            (
                                AllocationStrategy::Follower(ts[i - 1].agent),
                                i,
                                *cluster_id,
                            ),
                        );
                    }
                }
            }
        }

        // Now that we have prime leaders, extract the exact follow groups and determine follow
        // sections in the plan
        let mut leaders: Vec<_> = leader_to_cluster.keys().cloned().collect();

        leaders.sort_by(|&a, &b| b.trajectory_index.cmp(&a.trajectory_index));

        // Get a new "leader segment".
        let mut leader_to_leader_segments: HashMap<SemanticWaypoint, usize> = HashMap::new();
        let mut leader_segment_to_leader: HashMap<usize, HashSet<SemanticWaypoint>> =
            HashMap::new();
        let mut last_lead_segment = 1usize;

        for leader in leaders {
            let mut hypothetical_leader = leader;
            hypothetical_leader.trajectory_index += 1;
            if let Some(leader_segment) = leader_to_leader_segments.get(&hypothetical_leader) {
                if let Some(leaders) = leader_segment_to_leader.get_mut(leader_segment) {
                    leaders.insert(leader);
                    leader_to_leader_segments.insert(leader, *leader_segment);
                    continue;
                }
            }
            leader_segment_to_leader.insert(
                last_lead_segment,
                HashSet::from_iter([leader].iter().cloned()),
            );
            leader_to_leader_segments.insert(leader, last_lead_segment);
            last_lead_segment += 1;
        }

        // This table contains the lead segment that we will allocate free space up to for the leader.
        let mut allocate_till: HashMap<SemanticWaypoint, SemanticWaypoint> = HashMap::new();
        for (_, lead_members) in leader_segment_to_leader.iter() {
            let mut lead_member_buckets: HashMap<usize, Vec<&SemanticWaypoint>> = HashMap::new();

            for lead in lead_members {
                if let Some(agents) = lead_member_buckets.get_mut(&lead.agent) {
                    agents.push(lead);
                }
                lead_member_buckets.insert(lead.agent, vec![lead]);
            }

            // TODO(arjoc): Dont need a hashmap
            for (_, mut p) in lead_member_buckets {
                p.sort_by(|&a, &b| a.trajectory_index.cmp(&b.trajectory_index));

                // TODO(arjoc): verify if the lead segment actually needs sorting or a max is sufficient.
                let Some(&sem_wp) = p.first() else {
                    continue;
                };

                if p.len() <= 1 {
                    allocate_till.insert(*sem_wp, *sem_wp);
                    continue;
                }

                let mut last_upd = 0;
                for next_wp in 1..p.len() {
                    if p[next_wp].trajectory_index - p[next_wp - 1].trajectory_index > 1
                        || next_wp == p.len() - 1
                    {
                        let lead_segment = *p[next_wp];
                        for id in last_upd..next_wp {
                            allocate_till.insert(*p[id], lead_segment);
                        }
                        last_upd = next_wp;
                    }
                }
            }
        }

        LeaderFollowerZones {
            allocation_strategy,
        }
    }

    pub fn get_claim_dict(
        &self,
        current_positions: &[SemanticWaypoint],
    ) -> HashMap<usize, CurrentlyAllocatedTrajSegment> {
        let mut allocate_till: HashMap<SemanticWaypoint, SemanticWaypoint> = HashMap::new();

        let mut agent_to_pos = HashMap::new();
        for &wp in current_positions {
            // Keep track of where other agents are
            agent_to_pos.insert(wp.agent, wp);
        }

        // Claim every waypoint that can be claimed.
        for wp in current_positions {
            let mut new_wp = *wp;
            loop {
                new_wp.trajectory_index += 1;
                if !self.agent_time_to_wp_id.contains_key(&new_wp) {
                    break;
                }
                let Some(deps) = self.comes_before(&new_wp) else {
                    break;
                };
                let mut continue_to_mark = true;
                for dep in deps {
                    let dep_wp = self.waypoints[*dep];
                    let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) else {
                        continue;
                    };

                    // HACK FOR TYPE I DEP
                    if other_agent.agent == wp.agent {
                        continue;
                    }

                    if other_agent.trajectory_index <= dep_wp.trajectory_index {
                        // stop blocking
                        continue_to_mark = false;
                    }
                }
                if !continue_to_mark {
                    break;
                }
            }
            allocate_till.insert(*wp, new_wp);
        }

        // Prepare output
        let mut final_reservation_table = HashMap::new();
        for (start, end) in allocate_till.iter() {
            final_reservation_table.insert(
                start.agent,
                CurrentlyAllocatedTrajSegment {
                    start_id: start.trajectory_index,
                    end_id: end.trajectory_index,
                },
            );
        }
        final_reservation_table
    }

    pub fn check_for_violation(&self, current_pos: &[SemanticWaypoint]) -> bool {
        let mut agent_to_pos = HashMap::new();
        for &wp in current_pos {
            agent_to_pos.insert(wp.agent, wp);
        }

        for wp in current_pos {
            if let Some(deps) = self.comes_before(wp) {
                for &dep_idx in deps {
                    let dep_wp = self.waypoints[dep_idx];

                    if dep_wp.agent == wp.agent {
                        continue;
                    }

                    if let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) {
                        if other_agent.trajectory_index <= dep_wp.trajectory_index {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

/// The currently allocated trajectory segment based on the overall
/// progress of all other robots.
pub struct CurrentlyAllocatedTrajSegment {
    pub start_id: usize,
    pub end_id: usize,
}

/// Representation of the type of dependency. Is it a leader, a follower, an
/// intersection or in the vicinity.
pub enum AllocationStrategy {
    Leader(f64, f64), // Clears space till the end of the leader
    Follower(usize),  // Follows x
    IntersectionLead(usize),
    IntersectionWait(SemanticWaypoint),
    Vicinity,
}

/// Leader-follower zones describe space time regions in which leader-follower relationships hold
pub(crate) struct LeaderFollowerZones {
    pub(crate) allocation_strategy: HashMap<SemanticWaypoint, (AllocationStrategy, usize, usize)>,
}

#[derive(Debug, Clone)]
pub enum IntersectionType {
    Lead(HashSet<SemanticWaypoint>),
    Next(HashSet<SemanticWaypoint>),
}

fn cluster(g: &DiGraph<SemanticWaypoint, ()>) -> HashMap<usize, Vec<petgraph::prelude::NodeIndex>> {
    let mut node_sets = petgraph::unionfind::UnionFind::new(g.node_count());
    for edge in g.edge_references() {
        let (a, b) = (edge.source(), edge.target());

        // union the two nodes of the edge
        node_sets.union(a.index(), b.index());
    }

    // 4. Organize nodes into clusters (groups) using a HashMap.
    // The keys will be the canonical set IDs, and values will be lists of nodes in that cluster.
    let mut clusters = HashMap::new();

    for node_idx in g.node_indices() {
        let raw_idx = node_idx.index();
        // Find the canonical root (set ID) for this node
        let set_id = node_sets.find(raw_idx);

        // Add the node to the corresponding cluster list
        clusters
            .entry(set_id)
            .or_insert_with(Vec::new)
            .push(node_idx);
    }

    clusters
}

/// Pose Trajectory
#[derive(Debug, Clone)]
pub struct Trajectory {
    pub poses: Vec<Isometry2<f64>>,
}

impl Trajectory {
    pub fn len(&self) -> usize {
        self.poses.len()
    }
}

impl<W> From<&crate::motion::Trajectory<W>> for Trajectory
where
    W: crate::motion::r2::Positioned + crate::motion::se2::MaybeOriented + crate::motion::Waypoint,
{
    fn from(traj: &crate::motion::Trajectory<W>) -> Self {
        let mut poses = Vec::new();
        for wp in traj.iter() {
            let p = wp.point();
            let yaw = wp.maybe_oriented().map(|o| o.angle()).unwrap_or(0.0);
            poses.push(Isometry2::new(na::Vector2::new(p.x, p.y), yaw));
        }
        Self { poses }
    }
}

/// Input of time discretized MAPF result
/// The trajectories are time discretized, and the
/// footprints are the shapes of the agents.
/// We expect all trajectories to be of the same length.
#[derive(Clone)]
pub struct MapfResult {
    /// The trajectories of the agents
    pub trajectories: Vec<Trajectory>,
    /// The shapes of the agents
    pub footprints: Vec<Arc<dyn Shape>>,
    /// The time discretization of the trajectories
    pub discretization_timestep: f64,
    /// Mapping from agent name to ID
    pub agent_name_to_id: HashMap<String, usize>,
}

impl std::fmt::Debug for MapfResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MapfResult")
            .field("trajectories", &self.trajectories)
            .field("discretization_timestep", &self.discretization_timestep)
            .finish()
    }
}

/// Sweep the objects to check their motion
fn calculate_nonlinear_rigid_motion(
    isometry1: &Isometry2<f64>,
    isometry2: &Isometry2<f64>,
    delta_time: f64,           // Time elapsed between the two isometries
    local_center: Point2<f64>, // Local center of rotation
) -> NonlinearRigidMotion {
    // 1. Calculate Linear Velocity:
    let linear_velocity =
        (isometry2.translation.vector - isometry1.translation.vector) / delta_time;

    // 2. Calculate Angular Velocity:
    //    This is more complex and involves extracting the rotation difference.
    let rotation1 = isometry1.rotation;
    let rotation2 = isometry2.rotation;

    // Calculate the relative rotation (rotation that transforms rotation1 to rotation2)
    let relative_rotation = rotation2 * rotation1.inverse();

    // Convert the relative rotation to an axis-angle representation
    let angle = relative_rotation.angle();

    // Angular velocity is the axis scaled by the angle and divided by the time step.
    let angular_velocity = angle / delta_time;

    NonlinearRigidMotion {
        start: *isometry1, // Or *isometry2, depending on your starting point
        local_center,
        linvel: linear_velocity,
        angvel: angular_velocity,
    }
}

/// Check if the robots collide while executing a motion.
fn collides(
    ti1: &Isometry2<f64>,
    ti2: &Isometry2<f64>,
    shape_i: &dyn Shape,
    tj1: &Isometry2<f64>,
    tj2: &Isometry2<f64>,
    shape_j: &dyn Shape,
    delta_time: f64,
) -> bool {
    let motion_i = calculate_nonlinear_rigid_motion(ti1, ti2, delta_time, Point2::origin());
    let motion_j = calculate_nonlinear_rigid_motion(tj1, tj2, delta_time, Point2::origin());
    let time_of_impact = cast_shapes_nonlinear(
        &motion_i, shape_i, &motion_j, shape_j, 0.0, delta_time, true,
    );
    if let Ok(Some(toi)) = time_of_impact {
        // Check if the time of impact is within the delta_time
        toi.status == ShapeCastStatus::Converged && toi.time_of_impact <= delta_time
            || toi.status == ShapeCastStatus::PenetratingOrWithinTargetDist
    } else {
        false
    }
}

/// Simple implementation of mapf_post: A MapF -> semantic plan
/// Method.
/// TODO(arjoc): This can be improved.
///
/// Based on https://whoenig.github.io/publications/2019_RA-L_Hoenig.pdf
pub fn mapf_post(mapf_result: &MapfResult) -> SemanticPlan {
    let mut semantic_plan = SemanticPlan {
        agent_name_to_id: mapf_result.agent_name_to_id.clone(),
        ..SemanticPlan::default()
    };
    semantic_plan.num_agents = mapf_result.trajectories.len();
    // Type 1 edges
    for agent in 0..mapf_result.trajectories.len() {
        for trajectory_index in 0..mapf_result.trajectories[agent].len() {
            semantic_plan.add_waypoint(&SemanticWaypoint {
                agent,
                trajectory_index,
            });

            if trajectory_index < 1 {
                semantic_plan
                    .depends_on_all_of
                    .insert(semantic_plan.waypoints.len() - 1, vec![]);
                continue;
            }
            semantic_plan.depends_on_all_of.insert(
                semantic_plan.waypoints.len() - 1,
                vec![semantic_plan.waypoints.len() - 2],
            );
            semantic_plan.potential_successors.insert(
                semantic_plan.waypoints.len() - 2,
                vec![semantic_plan.waypoints.len() - 1],
            );
        }
    }

    // Type 2 edges.
    //
    // Broad phase: build a merged (start-pose, end-pose) AABB per trajectory
    // segment, then sweep-and-prune along whichever axis (X or Y) has the
    // larger spread. Segments are sorted by their AABB min on that axis, and
    // for each segment we only scan forward while later segments could still
    // overlap on that axis, stopping early once they can't. Only AABB-overlapping
    // pairs go through the expensive exact `collides` check.
    struct SegmentAabb {
        agent: usize,
        index: usize, // index in trajectory (represents motion from index-1 to index)
        aabb: Aabb,
    }

    let mut all_segments = Vec::new();
    let mut min_bound = Point2::new(f64::MAX, f64::MAX);
    let mut max_bound = Point2::new(f64::MIN, f64::MIN);

    for (agent_idx, trajectory) in mapf_result.trajectories.iter().enumerate() {
        let footprint = &*mapf_result.footprints[agent_idx];
        for i in 1..trajectory.len() {
            let aabb1 = footprint.compute_aabb(&trajectory.poses[i - 1]);
            let aabb2 = footprint.compute_aabb(&trajectory.poses[i]);
            let merged_aabb = aabb1.merged(&aabb2);

            min_bound.x = min_bound.x.min(merged_aabb.mins.x);
            min_bound.y = min_bound.y.min(merged_aabb.mins.y);
            max_bound.x = max_bound.x.max(merged_aabb.maxs.x);
            max_bound.y = max_bound.y.max(merged_aabb.maxs.y);

            all_segments.push(SegmentAabb {
                agent: agent_idx,
                index: i,
                aabb: merged_aabb,
            });
        }
    }

    let x_spread = max_bound.x - min_bound.x;
    let y_spread = max_bound.y - min_bound.y;
    let sort_by_x = x_spread >= y_spread;

    if sort_by_x {
        all_segments.sort_by(|a, b| a.aabb.mins.x.partial_cmp(&b.aabb.mins.x).unwrap());
    } else {
        all_segments.sort_by(|a, b| a.aabb.mins.y.partial_cmp(&b.aabb.mins.y).unwrap());
    }

    for i in 0..all_segments.len() {
        let seg1 = &all_segments[i];
        for j in i + 1..all_segments.len() {
            let seg2 = &all_segments[j];

            if sort_by_x {
                if seg2.aabb.mins.x > seg1.aabb.maxs.x {
                    break;
                }
            } else if seg2.aabb.mins.y > seg1.aabb.maxs.y {
                break;
            }

            if seg1.agent == seg2.agent {
                continue;
            }

            // We sorted by X or Y, so seg1 and seg2 can be in any temporal
            // order; normalize them to (earlier, later) by trajectory index
            // before running the exact collision check.
            let (earlier, later) = if seg1.index < seg2.index {
                (seg1, seg2)
            } else if seg2.index < seg1.index {
                (seg2, seg1)
            } else {
                // Same trajectory index on both agents: neither can depend on the other.
                continue;
            };

            if earlier.aabb.intersects(&later.aabb)
                && collides(
                    &mapf_result.trajectories[earlier.agent].poses[earlier.index - 1],
                    &mapf_result.trajectories[earlier.agent].poses[earlier.index],
                    &*mapf_result.footprints[earlier.agent],
                    &mapf_result.trajectories[later.agent].poses[later.index - 1],
                    &mapf_result.trajectories[later.agent].poses[later.index],
                    &*mapf_result.footprints[later.agent],
                    mapf_result.discretization_timestep,
                )
            {
                semantic_plan.requires_comes_after(
                    &SemanticWaypoint {
                        agent: earlier.agent,
                        trajectory_index: earlier.index - 1,
                    },
                    &SemanticWaypoint {
                        agent: later.agent,
                        trajectory_index: later.index - 1,
                    },
                );
            }
        }
    }
    semantic_plan
}

// ---
// Unit Tests
// ---

#[cfg(test)]
mod tests {
    use parry2d::{na::Vector2, utils::hashset::HashSet};

    use super::*;
    use std::collections::HashMap; // Make sure HashMap is in scope

    #[test]
    fn test_add_waypoint() {
        let mut plan = SemanticPlan::default();
        let wp1 = SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        };
        let wp2 = SemanticWaypoint {
            agent: 0,
            trajectory_index: 1,
        };
        let wp3 = SemanticWaypoint {
            agent: 1,
            trajectory_index: 0,
        };

        plan.add_waypoint(&wp1);
        assert_eq!(plan.waypoints.len(), 1);
        assert_eq!(plan.waypoints[0], wp1);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp1), Some(&0));

        plan.add_waypoint(&wp2);
        assert_eq!(plan.waypoints.len(), 2);
        assert_eq!(plan.waypoints[1], wp2);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp2), Some(&1));

        plan.add_waypoint(&wp3);
        assert_eq!(plan.waypoints.len(), 3);
        assert_eq!(plan.waypoints[2], wp3);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp3), Some(&2));

        // Adding an existing waypoint (by value) should still add it again with a new ID
        // This behavior is derived from how `add_waypoint` is implemented.
        let wp1_again = SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        };
        plan.add_waypoint(&wp1_again);
        assert_eq!(plan.waypoints.len(), 4);
        assert_eq!(plan.waypoints[3], wp1_again);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp1_again), Some(&3)); // Now maps to the new ID
    }

    #[test]
    fn test_add_comes_after_basic() {
        let mut plan = SemanticPlan::default();
        let wp0_0 = SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        }; // ID 0
        let wp0_1 = SemanticWaypoint {
            agent: 0,
            trajectory_index: 1,
        }; // ID 1
        let wp1_0 = SemanticWaypoint {
            agent: 1,
            trajectory_index: 0,
        }; // ID 2

        plan.add_waypoint(&wp0_0);
        plan.add_waypoint(&wp0_1);
        plan.add_waypoint(&wp1_0);

        // wp1_0 (ID 2) comes after wp0_0 (ID 0)
        plan.requires_comes_after(&wp0_0, &wp1_0);
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(2, vec![0]);
        assert_eq!(plan.depends_on_all_of, expected_comes_after);

        // wp1_0 (ID 2) also comes after wp0_1 (ID 1)
        plan.requires_comes_after(&wp0_1, &wp1_0);
        let mut expected_comes_after_2: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after_2.insert(2, vec![0, 1]);

        // Retrieve the actual vector and sort it for consistent comparison
        let mut actual_vec = plan.depends_on_all_of.get(&2).unwrap().clone();
        actual_vec.sort_unstable();

        // Retrieve the expected vector and sort it
        let mut expected_vec = expected_comes_after_2.get(&2).unwrap().clone();
        expected_vec.sort_unstable();

        assert_eq!(actual_vec, expected_vec);
    }

    #[test]
    fn test_add_comes_after_non_existent_waypoints() {
        let mut plan = SemanticPlan::default();
        let wp_a = SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        };
        let wp_b = SemanticWaypoint {
            agent: 0,
            trajectory_index: 1,
        };
        let wp_c = SemanticWaypoint {
            agent: 0,
            trajectory_index: 2,
        };

        plan.add_waypoint(&wp_a); // ID 0

        // Attempt to add a dependency where 'before' waypoint (wp_b) is not in the plan
        plan.requires_comes_after(&wp_b, &wp_a);
        assert!(plan.depends_on_all_of.is_empty());

        // Attempt to add a dependency where 'after' waypoint (wp_c) is not in the plan
        plan.requires_comes_after(&wp_a, &wp_c);
        assert!(plan.depends_on_all_of.is_empty());

        // Add wp_c to the plan
        plan.add_waypoint(&wp_c); // ID 1

        // Now, this dependency should be added successfully: wp_c (ID 1) comes after wp_a (ID 0)
        plan.requires_comes_after(&wp_a, &wp_c);
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(1, vec![0]);
        assert_eq!(plan.depends_on_all_of, expected_comes_after);
    }

    #[test]
    fn test_add_comes_after_self_dependency() {
        let mut plan = SemanticPlan::default();
        let wp1 = SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        };

        plan.add_waypoint(&wp1); // ID 0

        // Test self-dependency: wp1 (ID 0) comes after wp1 (ID 0)
        plan.requires_comes_after(&wp1, &wp1);
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(0, vec![0]);
        assert_eq!(plan.depends_on_all_of, expected_comes_after);
    }

    #[test]
    fn test_collision_detection() {
        let isometry1 = Isometry2::new(Vector2::new(0.0, 0.0), 0.0);
        let isometry2 = Isometry2::new(Vector2::new(1.0, 1.0), 0.0);
        let shape1 = Arc::new(parry2d::shape::Ball::new(0.5));
        let shape2 = Arc::new(parry2d::shape::Ball::new(0.5));
        let delta_time = 1.0;

        assert!(collides(
            &isometry1, &isometry2, &*shape1, &isometry1, &isometry2, &*shape2, delta_time
        ));
    }

    #[test]
    fn test_post_processing() {
        // Create a cross junction MapfResult
        let mapf_result = MapfResult {
            trajectories: vec![
                vec![
                    Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                ], // Trajectory for agent1 (horizontal path)
                vec![
                    Isometry2::new(Vector2::new(1.0, -1.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, -1.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 1.0), 0.0),
                ], // Trajectory for agent2 (vertical path)
            ]
            .into_iter()
            .map(|poses| Trajectory { poses })
            .collect(),
            footprints: vec![
                Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent1
                Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
            ],
            discretization_timestep: 1.0,
            agent_name_to_id: HashMap::default(),
        };

        // Generate the semantic plan
        let semantic_plan = mapf_post(&mapf_result);
        // Check the number of waypoints
        assert_eq!(semantic_plan.waypoints.len(), 8); // 4 for each agent
        let res = semantic_plan
            .comes_before(&SemanticWaypoint {
                agent: 1,
                trajectory_index: 2,
            })
            .unwrap();
        assert_eq!(res.len(), 3); // 3 waypoints should come before this one
        let t1 = semantic_plan.waypoints[res[0]];
        let t2 = semantic_plan.waypoints[res[1]];
        let t3 = semantic_plan.waypoints[res[2]];

        let desired = HashSet::from_iter(vec![t1, t2, t3]);
        assert!(desired.contains(&SemanticWaypoint {
            agent: 0,
            trajectory_index: 1
        }));
        assert!(desired.contains(&SemanticWaypoint {
            agent: 1,
            trajectory_index: 1
        }));
    }

    #[test]
    fn test_get_claims_dict_follow() {
        use std::sync::Arc;

        let mapf_result = MapfResult {
            trajectories: vec![
                vec![
                    Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                ], // Trajectory for agent1
                vec![
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                ], // Trajectory for agent2
            ]
            .into_iter()
            .map(|poses| Trajectory { poses })
            .collect(),
            footprints: vec![
                Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent1
                Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
            ],
            discretization_timestep: 1.0,
            agent_name_to_id: HashMap::default(),
        };

        let semantic_plan = mapf_post(&mapf_result);

        let reserved_parts = semantic_plan.get_claim_dict(&vec![
            SemanticWaypoint {
                agent: 0,
                trajectory_index: 0,
            },
            SemanticWaypoint {
                agent: 1,
                trajectory_index: 3,
            },
        ]);

        assert_eq!(reserved_parts.len(), 2);
        assert_eq!(reserved_parts.get(&0).unwrap().end_id, 4);
        assert_eq!(reserved_parts.get(&1).unwrap().end_id, 6);

        let reserved_parts = semantic_plan.get_claim_dict(&vec![
            SemanticWaypoint {
                agent: 0,
                trajectory_index: 3,
            },
            SemanticWaypoint {
                agent: 1,
                trajectory_index: 3,
            },
        ]);

        assert_eq!(reserved_parts.len(), 2);
        assert_eq!(reserved_parts.get(&0).unwrap().end_id, 4);
        assert_eq!(reserved_parts.get(&1).unwrap().end_id, 6);
    }

    #[test]
    fn test_check_for_violation() {
        use std::sync::Arc;

        let mapf_result = MapfResult {
            trajectories: vec![
                vec![
                    Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                ], // Trajectory for agent1
                vec![
                    Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                ], // Trajectory for agent2
            ]
            .into_iter()
            .map(|poses| Trajectory { poses })
            .collect(),
            footprints: vec![
                Arc::new(parry2d::shape::Ball::new(0.49)),
                Arc::new(parry2d::shape::Ball::new(0.49)),
            ],
            discretization_timestep: 1.0,
            agent_name_to_id: HashMap::default(),
        };

        let semantic_plan = mapf_post(&mapf_result);

        // Case 1: Safe state
        let safe_state = vec![
            SemanticWaypoint {
                agent: 0,
                trajectory_index: 0,
            },
            SemanticWaypoint {
                agent: 1,
                trajectory_index: 0,
            },
        ];
        assert!(!semantic_plan.check_for_violation(&safe_state));

        // Case 2: Violation
        let violation_state = vec![
            SemanticWaypoint {
                agent: 0,
                trajectory_index: 1,
            },
            SemanticWaypoint {
                agent: 1,
                trajectory_index: 0,
            },
        ];
        assert!(semantic_plan.check_for_violation(&violation_state));

        // Case 3: Safe again
        let safe_state_2 = vec![
            SemanticWaypoint {
                agent: 0,
                trajectory_index: 1,
            },
            SemanticWaypoint {
                agent: 1,
                trajectory_index: 1,
            },
        ];
        assert!(!semantic_plan.check_for_violation(&safe_state_2));
    }
}

/// Cross-checks the AABB sweep-and-prune broad phase used by `mapf_post`'s
/// Type 2 edge detection against a brute-force O((N*T)^2) reference
/// implementation, to make sure the pruning never drops a true collision.
#[cfg(test)]
mod sweep_line_tests {
    use super::*;
    use parry2d::na::Vector2;
    use parry2d::shape::Ball;
    use std::collections::HashSet;

    fn mapf_result_from(paths: &[Vec<(f64, f64)>], r: f64) -> MapfResult {
        MapfResult {
            trajectories: paths
                .iter()
                .map(|path| Trajectory {
                    poses: path
                        .iter()
                        .map(|&(x, y)| Isometry2::new(Vector2::new(x, y), 0.0))
                        .collect(),
                })
                .collect(),
            footprints: paths
                .iter()
                .map(|_| Arc::new(Ball::new(r)) as Arc<dyn Shape>)
                .collect(),
            discretization_timestep: 1.0,
            agent_name_to_id: HashMap::default(),
        }
    }

    fn type2_edges_of(plan: &SemanticPlan) -> HashSet<(SemanticWaypoint, SemanticWaypoint)> {
        let mut edges = HashSet::new();
        for successor in &plan.waypoints {
            let Some(predecessor_ids) = plan.comes_before(successor) else {
                continue;
            };
            for &pred_id in predecessor_ids {
                let predecessor = plan.waypoints[pred_id];
                if predecessor.agent != successor.agent {
                    edges.insert((predecessor, *successor));
                }
            }
        }
        edges
    }

    /// The brute-force O((NT)^2) collision-dependency algorithm that the
    /// sweep-and-prune broad phase replaced.
    fn brute_force_type2_edges(
        mapf_result: &MapfResult,
    ) -> HashSet<(SemanticWaypoint, SemanticWaypoint)> {
        let mut edges = HashSet::new();
        let trajectories = &mapf_result.trajectories;
        for agent1 in 0..trajectories.len() {
            for ti1 in 1..trajectories[agent1].len() {
                for agent2 in 0..trajectories.len() {
                    if agent1 == agent2 {
                        continue;
                    }
                    for ti2 in ti1 + 1..trajectories[agent2].len() {
                        if collides(
                            &trajectories[agent1].poses[ti1 - 1],
                            &trajectories[agent1].poses[ti1],
                            &*mapf_result.footprints[agent1],
                            &trajectories[agent2].poses[ti2 - 1],
                            &trajectories[agent2].poses[ti2],
                            &*mapf_result.footprints[agent2],
                            mapf_result.discretization_timestep,
                        ) {
                            edges.insert((
                                SemanticWaypoint {
                                    agent: agent1,
                                    trajectory_index: ti1 - 1,
                                },
                                SemanticWaypoint {
                                    agent: agent2,
                                    trajectory_index: ti2 - 1,
                                },
                            ));
                        }
                    }
                }
            }
        }
        edges
    }

    /// For ball footprints, straight-line motion scenarios the sweep-line
    /// must produce the identical dependency graph as the brute-force
    /// implementation: the merged endpoint AABB contains each segment's swept
    /// volume, so AABB-disjoint segments cannot collide, and both algorithms
    /// then apply the same exact collision check.
    #[test]
    fn test_sweep_line_matches_bruteforce() {
        let scenarios: Vec<(&str, Vec<Vec<(f64, f64)>>)> = vec![
            (
                "horizontal_cross",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 0.0)],
                    vec![(1.0, -1.0), (1.0, -1.0), (1.0, 0.0), (1.0, 1.0)],
                ],
            ),
            (
                "horizontal_follow",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                    vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
                ],
            ),
            (
                "vertical_follow",
                vec![
                    vec![(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)],
                    vec![(0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.0, 4.0)],
                ],
            ),
            (
                "head_on_swap",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                    vec![(2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
                ],
            ),
            (
                "diagonal_cross",
                vec![
                    vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)],
                    vec![(0.0, 2.0), (1.0, 1.0), (2.0, 0.0)],
                ],
            ),
            (
                "parallel_far_apart",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                    vec![(0.0, 50.0), (1.0, 50.0), (2.0, 50.0)],
                ],
            ),
            (
                "three_agents",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                    vec![(1.5, -1.5), (1.5, 0.0), (1.5, 1.5), (1.5, 1.5)],
                    vec![(3.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
                ],
            ),
            (
                "unequal_lengths_crossing",
                vec![
                    vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                    vec![(1.0, -2.0), (1.0, -1.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)],
                ],
            ),
        ];

        for (name, paths) in scenarios {
            let mapf_result = mapf_result_from(&paths, 0.49);
            let actual = type2_edges_of(&mapf_post(&mapf_result));
            let expected = brute_force_type2_edges(&mapf_result);
            assert_eq!(
                actual, expected,
                "sweep-line and brute-force disagree on scenario '{name}'"
            );
        }
    }

    /// A scene and its 90-degree rotation must yield identical edges, since
    /// ball footprints make collisions rotation-invariant. Tests that the
    /// sort-by-Y branch agrees with the sort-by-X branch.
    #[test]
    fn test_sort_axis_invariance() {
        // Spread is larger along X -> sweep sorts by X.
        let horizontal = mapf_result_from(
            &[
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
            ],
            0.49,
        );
        // Same agent/index structure rotated 90 degrees onto Y -> sweep sorts by Y.
        let vertical = mapf_result_from(
            &[
                vec![(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)],
                vec![(0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.0, 4.0)],
            ],
            0.49,
        );

        let horizontal_edges = type2_edges_of(&mapf_post(&horizontal));
        let vertical_edges = type2_edges_of(&mapf_post(&vertical));

        assert!(
            !horizontal_edges.is_empty(),
            "overlapping follow paths must produce collision dependencies"
        );
        assert_eq!(
            horizontal_edges, vertical_edges,
            "sort-by-X and sort-by-Y branches must produce identical edges"
        );
    }
}
