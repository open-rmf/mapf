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

pub mod scenario;
pub use scenario::*;

use crate::{
    algorithm::{
        path::{DecisionRange, MetaTrajectory},
        AStarConnect, QueueLength, SearchStatus,
    },
    domain::{ClosedStatus, Configurable, Cost},
    error::ThisError,
    graph::{occupancy::*, SharedGraph},
    motion::{
        have_conflict,
        r2::{Positioned, WaypointR2},
        se2::{DifferentialDriveLineFollow, WaypointSE2},
        trajectory::TrajectoryIter,
        BoundingBox, CcbsConstraint, CcbsEnvironment, CircularProfile, Duration,
        DynamicCircularObstacle, DynamicEnvironment, TimePoint, Timed, TravelEffortCost,
    },
    planner::{halt::QueueLengthLimit, Planner},
    premade::{SippSE2, StateSippSE2},
    util::triangular_for,
};
use kdtree::{distance::squared_euclidean, KdTree};
use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashMap, HashSet},
    sync::Arc,
};

#[derive(Debug, ThisError)]
pub enum NegotiationError {
    #[error("Some endpoints have a conflict:\n{0:?}")]
    ConflictingEndpoints(HashMap<String, String>),
    #[error("It was impossible to find a basic plan for {0}")]
    PlanningImpossible(String),
    #[error("A solution might have been possible, but we failed to find it")]
    PlanningFailed((Vec<NegotiationNode>, HashMap<usize, String>)),
}

pub fn negotiate(
    scenario: &Scenario,
    queue_length_limit: Option<usize>,
    algorithm: ConflictDetectionAlgorithm,
) -> Result<
    (
        NegotiationNode,
        Vec<NegotiationNode>,
        HashMap<usize, String>,
    ),
    NegotiationError,
> {
    let cs = scenario.cell_size;
    let mut conflicts = HashMap::new();
    triangular_for(scenario.agents.iter(), |(n_a, a), (n_b, b)| {
        let min_dist = a.radius + b.radius;
        let min_dist_squared = min_dist * min_dist;

        for (cell_a, cell_b) in [
            (a.start_cell(), b.start_cell()),
            (a.goal_cell(), b.goal_cell()),
        ] {
            let pa = cell_a.center_point(cs);
            let pb = cell_b.center_point(cs);
            let dist_squared = (pa - pb).norm_squared();
            if dist_squared < min_dist_squared {
                conflicts.insert(
                    (**n_a).clone().min((*n_b).clone()),
                    (**n_a).clone().max((*n_b).clone()),
                );
            }
        }
    });
    if !conflicts.is_empty() {
        return Err(NegotiationError::ConflictingEndpoints(conflicts));
    }

    let (name_map, agents) = {
        let mut name_map = HashMap::new();
        let mut agents = Vec::new();
        for (name, agent) in &scenario.agents {
            name_map.insert(agents.len(), name.clone());
            agents.push(agent.clone());
        }

        (name_map, agents)
    };

    let grid = {
        let mut grid = SparseGrid::new(scenario.cell_size);
        let changes: HashMap<_, _> = scenario
            .occupancy
            .iter()
            .flat_map(|(y, row)| row.iter().map(|x| (Cell::new(*x, *y), true)))
            .collect();
        grid.change_cells(&changes);
        grid
    };

    let profiles: Vec<_> = agents
        .iter()
        .map(|a| CircularProfile::new(a.radius, 0.0, 0.0).unwrap())
        .collect();

    let planners = agents
        .iter()
        .map(|a| {
            let profile = CircularProfile::new(a.radius, 0.0, 0.0).unwrap();
            let accessibility = Arc::new(Accessibility::new(grid.clone(), a.radius));
            // let visibility = Arc::new(Visibility::new(grid.clone(), a.radius));
            // let heuristic = SharedGraph::new(VisibilityGraph::new(visibility.clone(), []));
            // let activity = SharedGraph::new(NeighborhoodGraph::new(visibility.clone(), []));
            let activity = SharedGraph::new(AccessibilityGraph::new(accessibility));
            let heuristic = activity.clone();
            let environment = Arc::new(CcbsEnvironment::new(Arc::new(DynamicEnvironment::new(
                profile,
            ))));
            let extrapolator = DifferentialDriveLineFollow::new(a.speed, a.spin).unwrap();
            let weight = TravelEffortCost::default();

            Planner::new(AStarConnect(
                SippSE2::new_sipp_se2(activity, heuristic, extrapolator, environment, weight)
                    .unwrap(),
            ))
            .with_halting(QueueLengthLimit(queue_length_limit))
        })
        .collect::<Vec<_>>();

    let mut ideal: Vec<Proposal> = Vec::new();
    for (i, (agent, planner)) in agents.iter().zip(planners.iter()).enumerate() {
        let start = agent.make_start();
        let goal = agent.make_goal();
        let s = match match planner.plan(start, goal) {
            Ok(search) => search,
            Err(err) => {
                return Err(NegotiationError::PlanningImpossible(
                    format!("{err:?}").to_owned(),
                ))
            }
        }
        .solve()
        .unwrap()
        .solution()
        {
            Some(s) => s,
            None => {
                return Err(NegotiationError::PlanningImpossible(
                    name_map.get(&i).unwrap().clone(),
                ));
            }
        };
        let mt = s
            .make_trajectory_or_hold(Duration::from_secs(1))
            .unwrap()
            .with_indefinite_finish_time(true);
        ideal.push(Proposal {
            meta: mt,
            cost: s.total_cost,
        });
    }

    let (mut negotiation_of_agent, mut negotiations) =
        organize_negotiations(&ideal, &profiles, algorithm);

    let mut closer = NegotiationCloser::new();
    let mut culled = 0;
    let mut count = 0;
    let mut solution_node: Option<NegotiationNode> = None;
    let mut arena = Vec::new();
    while !negotiations.is_empty() {
        dbg!(count);
        count += 1;

        let base_env = {
            let mut base_env =
                DynamicEnvironment::new(CircularProfile::new(0.0, 0.0, 0.0).unwrap());
            for i in 0..agents.len() {
                if !negotiation_of_agent.contains_key(&i) {
                    base_env.obstacles.push(
                        DynamicCircularObstacle::new(profiles[i])
                            .with_trajectory(Some(ideal[i].meta.trajectory.clone())),
                    );
                }
            }
            Arc::new(base_env)
        };

        for root in negotiations.values() {
            let mut queue: BinaryHeap<QueueEntry> = BinaryHeap::new();
            let root = NegotiationNode::from_root(root, &ideal, base_env.clone(), arena.len());
            arena.push(root.clone());
            queue.push(QueueEntry::new(root));

            let mut solution = None;
            let mut iters = 0;
            while let Some(mut top) = queue.pop() {
                iters += 1;
                if iters % 10 == 0 {
                    dbg!(iters);
                }
                if iters > 1000 {
                    println!("Too many iterations");

                    // Dump the remaining queue into the node history
                    println!("Queue begins at {}", arena.len() + 1);
                    while let Some(remainder) = queue.pop() {
                        arena.push(remainder.node);
                    }

                    break;
                }

                if !closer.close(&top.node) {
                    culled += 1;
                    // println!("REDUNDANT NODE: {:?}", top.node.keys);
                    continue;
                }

                // Sort the conflicts such that we pop the the earliest conflict.
                // Using Reverse will put the conflicts in descending order, and
                // then popping the last element will grab the lowest time.
                top.node
                    .negotiation
                    .conflicts
                    .sort_unstable_by_key(|c| Reverse(c.time));
                let next_conflict = match top.node.negotiation.conflicts.pop() {
                    Some(c) => c,
                    None => {
                        // There are no conflicts left, so we have found the
                        // solution for this negotiation.
                        // solution = Some(top.node.proposals);
                        solution = Some(top.node);

                        // Dump the remaining queue into the node history
                        // println!("Queue begins at {}", arena.len() + 1);
                        // while let Some(remainder) = queue.pop() {
                        //     arena.push(remainder.node);
                        // }

                        break;
                    }
                };

                let finish_time = top
                    .node
                    .proposals
                    .values()
                    .max_by_key(|t| t.meta.trajectory.finish_motion_time())
                    .unwrap()
                    .meta
                    .trajectory
                    .finish_motion_time();

                let segments = next_conflict.segments;
                for (concede, constraint) in
                    [(segments[0], segments[1]), (segments[1], segments[0])]
                {
                    let mut environment = top.node.environment.clone();
                    // Insert the new constraint on top of the previous
                    // environment
                    let env_constraint = CcbsConstraint {
                        obstacle: DynamicCircularObstacle::new(profiles[constraint.agent])
                            .with_trajectory(Some(
                                top.node
                                    .proposals
                                    .get(&constraint.agent)
                                    .unwrap()
                                    .meta
                                    .get_trajectory_segment(&constraint.range),
                            )),
                        mask: constraint.agent,
                    };

                    match concede.range {
                        DecisionRange::Before(s, _) | DecisionRange::After(s, _) => {
                            // dbg!((concede, constraint));
                            environment.insert_constraint(
                                (s.key.vertex, s.key.vertex),
                                env_constraint.clone(),
                            );

                            // let mut test_env = DynamicEnvironment::new(profiles[concede.agent]);
                            // test_env.obstacles.push(env_constraint.obstacle);
                            // let arrivals = compute_safe_arrival_times(s.waypoint.into(), &test_env);
                            // assert!(arrivals.len() > 1, "The obstacle should have given us an altered arrival time");
                            // let t0 = s.waypoint.time;
                            // let tf = arrivals.last().cloned();
                            // NegotiationKey::new((s.key.vertex, s.key.vertex), (t0, tf), constraint.agent)
                        }
                        DecisionRange::Between(range) => {
                            environment.insert_constraint(
                                (range[0].state.key.vertex, range[1].state.key.vertex),
                                env_constraint.clone(),
                            );
                            // let mut test_env = DynamicEnvironment::new(profiles[concede.agent]);
                            // test_env.obstacles.push(env_constraint.obstacle);
                            // let paths = compute_safe_linear_paths(
                            //     range[0].state.waypoint.into(),
                            //     range[1].state.waypoint.into(),
                            //     &test_env,
                            // );
                            // assert!(paths.len() <= 1, "We should only get one safe path against a conflict constraint");
                            // let t0 = range[0].state.waypoint.time;
                            // let tf = paths.first().map(|p| p.last().unwrap().movement().unwrap().time);
                            // NegotiationKey::new((range[0].state.key.vertex, range[1].state.key.vertex), (t0, tf), constraint.agent)
                        }
                    };

                    let key =
                        NegotiationKey::new(&concede.range, &constraint.range, constraint.agent);

                    // Set the environment to be suitable for the conceding agent
                    environment.overlay_profile(profiles[concede.agent]);
                    environment.set_mask(Some(concede.agent));

                    let agent = &agents[concede.agent];
                    // Replan for the conceding agent with this constraint added
                    let mut search = planners[concede.agent]
                        .clone()
                        .configure(|config| config.modify_environment(|_| Ok(environment.clone())))
                        .unwrap()
                        .plan(
                            agent.make_start(),
                            agent.make_goal().with_minimum_time(Some(finish_time)),
                        )
                        .unwrap();

                    let solution = match search.solve().unwrap() {
                        SearchStatus::Solved(solution) => solution,
                        SearchStatus::Impossible => {
                            println!(
                                "The search is IMPOSSIBLE for {}! node: {}:{}, queue: {}, arena: {}",
                                name_map[&concede.agent],
                                top.node.id,
                                concede.agent,
                                search.memory().0.queue.len(),
                                search.memory().0.arena.len(),
                            );
                            let mut failed_node = top.node.clone();
                            failed_node.conceded = Some(concede.agent);
                            failed_node.environment = environment;
                            failed_node.outcome = NodeOutcome::Impossible;
                            arena.push(failed_node);
                            continue;
                        }
                        SearchStatus::Incomplete => {
                            println!(
                                "The search is INCOMPLETE for {}! node: {}:{} measure: {}, queue: {}, arena: {}",
                                name_map[&concede.agent],
                                top.node.id,
                                concede.agent,
                                search.memory().queue_length(),
                                search.memory().0.queue.len(),
                                search.memory().0.arena.len(),
                            );
                            let mut failed_node = top.node.clone();
                            failed_node.conceded = Some(concede.agent);
                            failed_node.environment = environment;
                            failed_node.outcome = NodeOutcome::Incomplete;
                            arena.push(failed_node);
                            continue;
                        }
                    };

                    let mt = solution
                        .make_trajectory_or_hold::<WaypointSE2>(Duration::from_secs(1))
                        .unwrap()
                        .with_indefinite_finish_time(true);
                    let cost = solution.total_cost;

                    let mut proposals = top.node.proposals.clone();
                    proposals.insert(concede.agent, Proposal { meta: mt, cost });
                    let conflicts = reasses_conflicts(&proposals, &profiles, algorithm);

                    let node = top.node.fork(
                        conflicts,
                        proposals,
                        environment,
                        key,
                        Some(concede.agent),
                        arena.len(),
                    );
                    arena.push(node.clone());
                    queue.push(QueueEntry::new(node));
                }
            }

            if let Some(solution) = solution {
                for (i, mt) in &solution.proposals {
                    ideal[*i] = mt.clone();
                }
                solution_node = Some(solution);
            } else {
                // TODO(@mxgrey): Consider re-running the negotiation but
                // without the base environment in order to identify which other
                // agents need to be pulled into the negotiation.
                // Even better would be to queue up those nodes as backup nodes
                // in a lower priority queue running in parallel, then use the
                // outcome if a solution cannot be found.
                println!("Culled {culled}");
                return Err(NegotiationError::PlanningFailed((arena, name_map)));
            }
        }

        (negotiation_of_agent, negotiations) =
            reconsider_negotiations(&ideal, &profiles, negotiation_of_agent, negotiations, algorithm);
    }

    if solution_node.is_none() {
        let fake = NegotiationNode::from_root(
            &Negotiation {
                conflicts: vec![],
                participants: vec![],
            },
            &ideal,
            Arc::new(DynamicEnvironment::new(
                CircularProfile::new(0.0, 0.0, 0.0).unwrap(),
            )),
            0,
        );

        solution_node = Some(fake);
    }

    if let Some(node) = &mut solution_node {
        for (i, proposal) in ideal.iter().enumerate() {
            match node.proposals.entry(i) {
                std::collections::hash_map::Entry::Vacant(vacant) => {
                    vacant.insert(proposal.clone());
                }
                _ => {}
            }
        }
    }

    // dbg!(closer);

    println!("Culled {culled}");
    Ok((solution_node.unwrap(), arena, name_map))
    // Ok(ideal.into_iter().enumerate().map(|(i, proposal)|
    //     (name_map.get(&i).unwrap().clone(), proposal)
    // ).collect())
}

#[derive(Debug, Clone)]
pub struct Proposal {
    pub meta: MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    pub cost: Cost<f64>,
}

#[derive(Debug, Clone, Copy, Hash, Eq, PartialEq)]
pub struct NegotiationKey {
    pub concede: (Cell, Cell, i64),
    pub constraint: (Cell, Cell, i64),
    pub mask: usize,
}

impl NegotiationKey {
    pub fn new(
        // concede: (Cell, Cell, TimePoint),
        // constraint: (Cell, Cell, TimePoint),
        concede: &DecisionRange<StateSippSE2<Cell>>,
        constraint: &DecisionRange<StateSippSE2<Cell>>,
        mask: usize,
    ) -> Self {
        let res = 1e3 as i64;
        // let res = 1e7 as i64;
        let concede = (
            concede.initial_state().key.vertex,
            concede.final_state().key.vertex,
            concede.initial_state().waypoint.time,
        );
        let constraint = (
            constraint.initial_state().key.vertex,
            constraint.final_state().key.vertex,
            constraint.initial_state().waypoint.time,
        );
        Self {
            concede: (concede.0, concede.1, concede.2.nanos_since_zero / res),
            constraint: (
                constraint.0,
                constraint.1,
                constraint.2.nanos_since_zero / res,
            ),
            mask,
        }
    }
}

#[derive(Debug, Clone)]
pub struct NegotiationCloser {
    pub closed_set: HashMap<NegotiationKey, HashSet<usize>>,
}

impl NegotiationCloser {
    pub fn new() -> Self {
        Self {
            closed_set: Default::default(),
        }
    }

    pub fn status<'a>(&'a self, node: &NegotiationNode) -> ClosedStatus<'a, ()> {
        let mut key_iter = node.keys.iter();
        let Some(first_key) = key_iter.next() else {
            return ClosedStatus::Open;
        };

        let mut candidates: HashSet<usize> = HashSet::from_iter(
            self.closed_set
                .get(first_key)
                .iter()
                .flat_map(|x| *x)
                .cloned(),
        );

        while let Some(next_key) = key_iter.next() {
            if candidates.is_empty() {
                break;
            }

            let Some(new_candidates) = self.closed_set.get(next_key) else {
                return ClosedStatus::Open;
            };
            candidates.retain(|c| new_candidates.contains(c));
        }

        if candidates.is_empty() {
            ClosedStatus::Open
        } else {
            ClosedStatus::Closed(&())
        }
    }

    pub fn close<'a>(&'a mut self, node: &NegotiationNode) -> bool {
        if self.status(node).is_closed() {
            return false;
        }

        for key in &node.keys {
            self.closed_set.entry(*key).or_default().insert(node.id);
        }

        true
    }
}

#[derive(Debug, Clone)]
pub struct NegotiationNode {
    /// Basic info about this negotiation
    pub negotiation: Negotiation,
    /// What participant trajectories have been completed. When these are all
    /// conflict-free then we have found a solution.
    pub proposals: HashMap<usize, Proposal>,
    /// The environment that was used to reach this node. The overlay contains
    /// the accumulated constraints for this node, inserted as obstacles in the
    /// overlay.
    pub environment: CcbsEnvironment<WaypointSE2, Cell>,
    pub keys: HashSet<NegotiationKey>,
    pub conceded: Option<usize>,
    pub cost: Cost<f64>,
    pub depth: usize,
    pub outcome: NodeOutcome,
    pub id: usize,
    pub parent: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
pub enum NodeOutcome {
    Success,
    Impossible,
    Incomplete,
}

impl NegotiationNode {
    fn from_root(
        root: &Negotiation,
        ideal: &Vec<Proposal>,
        base_env: Arc<DynamicEnvironment<WaypointSE2>>,
        id: usize,
    ) -> Self {
        let cost = ideal
            .iter()
            .fold(Cost(0.0), |cost, proposal| cost + proposal.cost);
        Self {
            negotiation: root.clone(),
            proposals: root
                .participants
                .iter()
                .map(|i| (*i, ideal[*i].clone()))
                .collect(),
            environment: {
                let mut env = CcbsEnvironment::new(base_env);
                for i in &root.participants {
                    env.overlay_trajectory(*i, None).ok();
                }
                env
            },
            conceded: None,
            keys: HashSet::new(),
            cost,
            depth: 0,
            outcome: NodeOutcome::Success,
            id,
            parent: None,
        }
    }

    fn fork(
        &self,
        conflicts: Vec<Conflict>,
        proposals: HashMap<usize, Proposal>,
        environment: CcbsEnvironment<WaypointSE2, Cell>,
        key: NegotiationKey,
        conceded: Option<usize>,
        id: usize,
    ) -> Self {
        let cost = proposals
            .values()
            .fold(Cost(0.0), |cost, proposal| cost + proposal.cost);
        let mut keys = self.keys.clone();
        if !keys.insert(key) {
            println!("REDUNDANT KEY IN {}: {key:?}", self.id);
        }
        NegotiationNode {
            negotiation: Negotiation {
                conflicts,
                participants: self.negotiation.participants.clone(),
            },
            proposals,
            environment,
            conceded,
            cost,
            keys,
            depth: self.depth + 1,
            outcome: self.outcome,
            id,
            parent: Some(self.id),
        }
    }
}

#[derive(Clone)]
struct QueueEntry {
    node: NegotiationNode,
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        if f64::abs(self.node.cost.0 - other.node.cost.0) < 0.1 {
            Reverse(self.node.depth).partial_cmp(&Reverse(other.node.depth))
        } else {
            Reverse(self.node.cost).partial_cmp(&Reverse(other.node.cost))
        }
    }
}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.node.cost.eq(&other.node.cost)
    }
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if f64::abs(self.node.cost.0 - other.node.cost.0) < 0.1 {
            self.node.depth.cmp(&self.node.depth)
        } else {
            Reverse(self.node.cost).cmp(&Reverse(other.node.cost))
        }
    }
}
impl Eq for QueueEntry {}

impl QueueEntry {
    fn new(node: NegotiationNode) -> Self {
        Self { node }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Negotiation {
    /// Conflicts that were identified for this state of the negotiation
    pub conflicts: Vec<Conflict>,
    /// All agents that need to participate in the negotiation
    pub participants: Vec<usize>,
}

#[derive(Clone, Copy)]
pub struct Conflict {
    time: TimePoint,
    segments: [Segment; 2],
}

impl std::fmt::Debug for Conflict {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Conflict")
            .field("time", &self.time.as_secs_f64())
            .field("segments", &self.segments)
            .finish()
    }
}

impl Conflict {
    pub fn time(&self) -> TimePoint {
        self.time
    }

    pub fn agent_pair(&self) -> [usize; 2] {
        let a = self.segments[0].agent;
        let b = self.segments[1].agent;
        [a.min(b), a.max(b)]
    }
}

pub type SippDecisionRange = DecisionRange<StateSippSE2<Cell>>;
pub type DecisionRangePair = (SippDecisionRange, SippDecisionRange);

#[derive(Debug, Clone, Copy)]
pub struct Segment {
    agent: usize,
    range: SippDecisionRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictDetectionAlgorithm {
    Baseline,
    KdTree,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ConflictDetectionStats {
    pub trajectory_count: usize,
    pub pair_enumerations: usize,
    pub bbox_candidate_pairs: usize,
    pub conflicts_found: usize,
}

#[derive(Debug, Clone)]
pub struct ConflictDetectionReport {
    pub conflicts: Vec<Conflict>,
    pub stats: ConflictDetectionStats,
}

#[derive(Clone, Copy)]
struct ProposalEnvelope {
    agent: usize,
    bbox: BoundingBox,
    center: [f64; 2],
    radius: f64,
}

pub fn detect_conflicts_for_proposals(
    proposals: &HashMap<usize, Proposal>,
    profiles: &[CircularProfile],
    algorithm: ConflictDetectionAlgorithm,
) -> ConflictDetectionReport {
    let ordered = collect_ordered_proposals(proposals, profiles);
    match algorithm {
        ConflictDetectionAlgorithm::Baseline => detect_conflicts_baseline(&ordered),
        ConflictDetectionAlgorithm::KdTree => detect_conflicts_kdtree(&ordered),
    }
}

fn reasses_conflicts(
    proposals: &HashMap<usize, Proposal>,
    profiles: &Vec<CircularProfile>,
    algorithm: ConflictDetectionAlgorithm,
) -> Vec<Conflict> {
    detect_conflicts_for_proposals(proposals, profiles, algorithm).conflicts
}

type OrderedProposalRef<'a> = (usize, &'a Proposal, &'a CircularProfile);

fn collect_ordered_proposals<'a>(
    proposals: &'a HashMap<usize, Proposal>,
    profiles: &'a [CircularProfile],
) -> Vec<OrderedProposalRef<'a>> {
    let mut ordered: Vec<_> = proposals
        .iter()
        .map(|(agent, proposal)| (*agent, proposal, profiles.get(*agent).unwrap()))
        .collect();
    ordered.sort_unstable_by_key(|(agent, _, _)| *agent);
    ordered
}

fn conflict_for_pair(
    agent_a: usize,
    mt_a: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_a: &CircularProfile,
    agent_b: usize,
    mt_b: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_b: &CircularProfile,
) -> Option<Conflict> {
    let (range_a, range_b) = find_first_conflict(mt_a, profile_a, mt_b, profile_b)?;
    Some(Conflict {
        time: TimePoint::min(
            mt_a.decision_start_time(&range_a),
            mt_b.decision_start_time(&range_b),
        ),
        segments: [
            Segment {
                agent: agent_a,
                range: range_a,
            },
            Segment {
                agent: agent_b,
                range: range_b,
            },
        ],
    })
}

fn detect_conflicts_baseline(ordered: &[OrderedProposalRef<'_>]) -> ConflictDetectionReport {
    let mut stats = ConflictDetectionStats {
        trajectory_count: ordered.len(),
        ..Default::default()
    };
    let mut conflicts = Vec::new();

    for i in 0..ordered.len() {
        let (agent_a, proposal_a, profile_a) = ordered[i];
        for (agent_b, proposal_b, profile_b) in ordered[i + 1..].iter().copied() {
            stats.pair_enumerations += 1;

            let bbox_a = BoundingBox::for_trajectory(profile_a, &proposal_a.meta.trajectory);
            let bbox_b = BoundingBox::for_trajectory(profile_b, &proposal_b.meta.trajectory);
            if !bbox_a.overlaps(Some(bbox_b)) {
                continue;
            }

            stats.bbox_candidate_pairs += 1;
            if let Some(conflict) = conflict_for_pair(
                agent_a,
                &proposal_a.meta,
                profile_a,
                agent_b,
                &proposal_b.meta,
                profile_b,
            ) {
                conflicts.push(conflict);
            }
        }
    }

    conflicts.sort_by_key(|conflict| {
        let [a, b] = conflict.agent_pair();
        (conflict.time().nanos_since_zero, a, b)
    });
    stats.conflicts_found = conflicts.len();
    ConflictDetectionReport { conflicts, stats }
}

fn detect_conflicts_kdtree(ordered: &[OrderedProposalRef<'_>]) -> ConflictDetectionReport {
    let mut stats = ConflictDetectionStats {
        trajectory_count: ordered.len(),
        ..Default::default()
    };
    let mut conflicts = Vec::new();

    if ordered.len() < 2 {
        return ConflictDetectionReport { conflicts, stats };
    }

    let envelopes: Vec<_> = ordered
        .iter()
        .map(|(agent, proposal, profile)| {
            let bbox = BoundingBox::for_trajectory(profile, &proposal.meta.trajectory);
            let center = bbox.center();
            ProposalEnvelope {
                agent: *agent,
                bbox,
                center: [center.x, center.y],
                radius: bbox.circumscribed_radius(),
            }
        })
        .collect();

    let max_radius = envelopes.iter().fold(0.0_f64, |r, env| r.max(env.radius));
    let mut tree = KdTree::new(2);
    for (index, envelope) in envelopes.iter().enumerate() {
        tree.add(envelope.center, index).unwrap();
    }

    let mut visited_pairs = HashSet::new();
    for (index, envelope_a) in envelopes.iter().enumerate() {
        let query_radius = envelope_a.radius + max_radius;
        let neighbors = tree
            .within(&envelope_a.center, query_radius.powi(2), &squared_euclidean)
            .unwrap();

        for neighbor in neighbors {
            let other_index = *neighbor.1;
            if other_index <= index {
                continue;
            }

            let envelope_b = envelopes[other_index];
            let pair = (
                envelope_a.agent.min(envelope_b.agent),
                envelope_a.agent.max(envelope_b.agent),
            );
            if !visited_pairs.insert(pair) {
                continue;
            }

            stats.pair_enumerations += 1;
            if !envelope_a.bbox.overlaps(Some(envelope_b.bbox)) {
                continue;
            }

            stats.bbox_candidate_pairs += 1;
            let (agent_a, proposal_a, profile_a) = ordered[index];
            let (agent_b, proposal_b, profile_b) = ordered[other_index];
            if let Some(conflict) = conflict_for_pair(
                agent_a,
                &proposal_a.meta,
                profile_a,
                agent_b,
                &proposal_b.meta,
                profile_b,
            ) {
                conflicts.push(conflict);
            }
        }
    }

    conflicts.sort_by_key(|conflict| {
        let [a, b] = conflict.agent_pair();
        (conflict.time().nanos_since_zero, a, b)
    });
    stats.conflicts_found = conflicts.len();
    ConflictDetectionReport { conflicts, stats }
}

fn organize_negotiations(
    ideal: &Vec<Proposal>,
    profiles: &Vec<CircularProfile>,
    algorithm: ConflictDetectionAlgorithm,
) -> (HashMap<usize, usize>, HashMap<usize, Negotiation>) {
    let mut next_conflict_id = 0;
    let mut negotiation_of_agent: HashMap<usize, usize> = HashMap::new();
    let mut negotiations: HashMap<usize, Negotiation> = HashMap::new();

    let proposals_map: HashMap<usize, Proposal> = ideal
        .iter()
        .enumerate()
        .map(|(i, p)| (i, p.clone()))
        .collect();

    let report = detect_conflicts_for_proposals(&proposals_map, profiles, algorithm);

    for conflict in report.conflicts {
        let i_a = conflict.segments[0].agent;
        let i_b = conflict.segments[1].agent;

        // Check if either agent is already in a conflict
        let conflict_id_a = negotiation_of_agent.get(&i_a).cloned();
        let conflict_id_b = negotiation_of_agent.get(&i_b).cloned();
        let conflict_id = if let (Some(conflict_id_a), Some(conflict_id_b)) =
            (conflict_id_a, conflict_id_b)
        {
            if conflict_id_a == conflict_id_b {
                conflict_id_a
            } else {
                // Pick the smaller id to merge both into.
                let conflict_id = usize::min(conflict_id_a, conflict_id_b);
                let acquire_id = usize::max(conflict_id_a, conflict_id_b);
                let info = negotiations.remove(&acquire_id).unwrap();
                for c in &info.conflicts {
                    for w in &c.segments {
                        negotiation_of_agent.insert(w.agent, conflict_id);
                    }
                }
                negotiations
                    .entry(conflict_id)
                    .or_default()
                    .conflicts
                    .extend(info.conflicts);

                conflict_id
            }
        } else if let Some(conflict_id_a) = conflict_id_a {
            conflict_id_a
        } else if let Some(conflict_id_b) = conflict_id_b {
            conflict_id_b
        } else {
            let conflict_id = next_conflict_id;
            next_conflict_id += 1;
            conflict_id
        };

        negotiation_of_agent.insert(i_a, conflict_id);
        negotiation_of_agent.insert(i_b, conflict_id);
        negotiations
            .entry(conflict_id)
            .or_default()
            .conflicts
            .push(conflict);
    }

    for negotiation in negotiations.values_mut() {
        negotiation.participants = negotiation
            .conflicts
            .iter()
            .flat_map(|c| c.segments.iter().map(|s| s.agent))
            .collect();
        negotiation.participants.sort_unstable();
        negotiation.participants.dedup();
    }

    (negotiation_of_agent, negotiations)
}

fn reconsider_negotiations(
    base: &Vec<Proposal>,
    profiles: &Vec<CircularProfile>,
    previous_negotiation_of_agent: HashMap<usize, usize>,
    previous_negotiations: HashMap<usize, Negotiation>,
    algorithm: ConflictDetectionAlgorithm,
) -> (HashMap<usize, usize>, HashMap<usize, Negotiation>) {
    let (mut new_negotiation_of_agent, mut new_negotiations) =
        organize_negotiations(base, profiles, algorithm);

    // Now that we've negotiated away some conflicts, check if any new conflicts
    // have been formed and pull all newly conflicting agents together into a
    // larger negotiation.

    // Key: ID of an old negotiation
    // Value: IDs of the new negotiations that ought to contain the participants
    // of the old negotiations.
    let mut merge_old_negotiation_into_new: HashMap<usize, HashSet<usize>> = HashMap::new();
    for (i, negotiation) in &new_negotiations {
        for agent in &negotiation.participants {
            let Some(n_prev) = previous_negotiation_of_agent.get(agent).cloned() else {
                continue;
            };
            merge_old_negotiation_into_new
                .entry(n_prev)
                .or_default()
                .insert(*i);
        }
    }

    let mut merge_new_negotiation_into: HashMap<usize, usize> = HashMap::new();
    for overlapping in merge_old_negotiation_into_new.values() {
        let merge_all_into = 'merge: {
            // Find if one of them is already supposed to merge into another
            for i in overlapping {
                if let Some(i_into) = merge_new_negotiation_into.get(i) {
                    break 'merge *i_into;
                }
            }

            // None of them is being merged into a new negotiation yet, so pick
            // the smallest value.
            overlapping.iter().min().cloned().unwrap()
        };

        for i in overlapping {
            dbg!(i);
            if let Some(prev_parent) = merge_new_negotiation_into.get(i).copied() {
                // assert!(!merge_new_negotiation_into.contains_key(&prev_parent));
                if merge_new_negotiation_into.contains_key(&prev_parent) {
                    dbg!(prev_parent);
                    dbg!(&merge_new_negotiation_into);
                    assert!(false);
                }

                dbg!((prev_parent, merge_all_into));
                if prev_parent != merge_all_into {
                    merge_new_negotiation_into.insert(prev_parent, merge_all_into);
                }
            }

            if *i == merge_all_into {
                dbg!(merge_all_into);
                merge_new_negotiation_into.remove(&merge_all_into);
            } else {
                dbg!((*i, merge_all_into));
                merge_new_negotiation_into.insert(*i, merge_all_into);
            }
        }
    }

    let mut count = 0;
    let mut inconsistent = true;
    while inconsistent {
        // TODO(@mxgrey): Remove the counting and assertion after testing
        count += 1;
        // assert!(count < 1_000_000);
        if count > 1_000_000 {
            println!("Unable to achieve a consistent renegotiation");
            dbg!(&merge_new_negotiation_into);
            assert!(false);
        }

        inconsistent = false;
        let mut redirect = Vec::new();
        for (from_i, to_i) in &merge_new_negotiation_into {
            if let Some(redirect_i) = merge_new_negotiation_into.get(to_i) {
                inconsistent = true;
                redirect.push((*from_i, *redirect_i));
            }
        }

        for (from_i, redirect_i) in redirect {
            merge_new_negotiation_into.insert(from_i, redirect_i);
        }
    }

    for (merge_from, merge_into) in &merge_new_negotiation_into {
        let merge_from_n = new_negotiations.remove(merge_from).unwrap();
        let merge_into_n = new_negotiations.get_mut(merge_into).unwrap();
        merge_into_n.conflicts.extend(merge_from_n.conflicts);
        merge_into_n.participants.extend(merge_from_n.participants);
        merge_into_n.participants.sort_unstable();
        merge_into_n.participants.dedup();
    }

    for (old_i, into_negotiations) in &merge_old_negotiation_into_new {
        if let Some(new_n) = into_negotiations.iter().next() {
            let merge_into = if let Some(redirect_n) = merge_new_negotiation_into.get(new_n) {
                *redirect_n
            } else {
                *new_n
            };

            let old_n = previous_negotiations.get(old_i).unwrap();
            let merge_into_n = new_negotiations.get_mut(&merge_into).unwrap();
            merge_into_n
                .participants
                .extend(old_n.participants.iter().copied());
            merge_into_n.participants.sort_unstable();
            merge_into_n.participants.dedup();
        }
    }

    for (i, n) in &new_negotiations {
        for p in &n.participants {
            new_negotiation_of_agent.insert(*p, *i);
        }
    }

    (new_negotiation_of_agent, new_negotiations)
}

pub fn find_first_conflict(
    mt_a: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_a: &CircularProfile,
    mt_b: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_b: &CircularProfile,
) -> Option<DecisionRangePair> {
    let (traj_a, traj_b) = (&mt_a.trajectory, &mt_b.trajectory);

    if !BoundingBox::for_trajectory(profile_a, traj_a)
        .overlaps(Some(BoundingBox::for_trajectory(profile_b, traj_b)))
    {
        return None;
    }

    if let Some(r) = find_pre_initial_conflict(mt_a, profile_a, mt_b, profile_b) {
        return Some(r);
    }

    let mut iter_a = traj_a.iter().pairs().enumerate();
    let mut iter_b = traj_b.iter().pairs().enumerate();
    let mut next_a = iter_a.next();
    let mut next_b = iter_b.next();
    let mut bb_a: Option<BoundingBox> = None;
    let mut bb_b: Option<BoundingBox> = None;
    let conflict_distance_squared = profile_a.conflict_distance_squared_for(profile_b);

    while let (Some((i_a, [wp0_a, wp1_a])), Some((i_b, [wp0_b, wp1_b]))) = (next_a, next_b) {
        if wp1_a.time <= wp0_b.time {
            bb_a = None;
            next_a = iter_a.next();
            continue;
        }

        if wp1_b.time <= wp0_a.time {
            bb_b = None;
            next_b = iter_b.next();
            continue;
        }

        let wp0_a: WaypointR2 = wp0_a.into();
        let wp1_a: WaypointR2 = wp1_a.into();
        let wp0_b: WaypointR2 = wp0_b.into();
        let wp1_b: WaypointR2 = wp1_b.into();

        if bb_a.is_none() {
            bb_a = Some(BoundingBox::for_line(profile_a, &wp0_a, &wp1_a));
        }

        if bb_b.is_none() {
            bb_b = Some(BoundingBox::for_line(profile_b, &wp0_b, &wp1_b));
        }

        if have_conflict(
            (&wp0_a, &wp1_a),
            bb_a,
            profile_a,
            (&wp0_b, &wp1_b),
            bb_b,
            profile_b,
            conflict_distance_squared,
        ) {
            return Some((mt_a.get_decision_range(i_a), mt_b.get_decision_range(i_b)));
            // return dbg!(Some((
            //     mt_a.get_decision_range(i_a),
            //     mt_b.get_decision_range(i_b),
            // )));
        }

        if wp1_a.time < wp1_b.time {
            bb_a = None;
            next_a = iter_a.next();
        } else {
            bb_b = None;
            next_b = iter_b.next();
        }
    }

    if let Some(r) = find_post_finish_conflict(mt_a, profile_a, mt_b, profile_b) {
        return Some(r);
    }

    None
}

fn find_pre_initial_conflict(
    mt_a: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_a: &CircularProfile,
    mt_b: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_b: &CircularProfile,
) -> Option<DecisionRangePair> {
    let (mt_a, profile_a, wp_b, profile_b, swapped) = {
        if mt_a.trajectory.initial_motion_time() < mt_b.trajectory.initial_motion_time() {
            (
                mt_a,
                profile_a,
                mt_b.trajectory.initial_motion().clone(),
                profile_b,
                false,
            )
        } else if mt_b.trajectory.initial_motion_time() < mt_a.trajectory.initial_motion_time() {
            (
                mt_b,
                profile_b,
                mt_a.trajectory.initial_motion().clone(),
                profile_a,
                true,
            )
        } else {
            return None;
        }
    };

    let (i_a, t0) =
        match find_spillover_conflict(mt_a.trajectory.iter(), profile_a, wp_b, profile_b, false) {
            Some(i_a) => i_a,
            None => return None,
        };

    let mut range_a = mt_a.get_decision_range(i_a);
    let mut range_b = DecisionRange::Before(mt_b.initial_state.clone(), t0);
    if swapped {
        std::mem::swap(&mut range_a, &mut range_b);
    }

    Some((range_a, range_b))
    // dbg!(Some((range_a, range_b)))
}

fn find_post_finish_conflict(
    mt_a: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_a: &CircularProfile,
    mt_b: &MetaTrajectory<WaypointSE2, StateSippSE2<Cell>>,
    profile_b: &CircularProfile,
) -> Option<DecisionRangePair> {
    let (mt_a, profile_a, mt_b, profile_b, swapped) = {
        if mt_b.trajectory.finish_motion_time() < mt_a.trajectory.finish_motion_time() {
            (mt_a, profile_a, mt_b, profile_b, false)
        } else if mt_a.trajectory.finish_motion_time() < mt_b.trajectory.finish_motion_time() {
            (mt_b, profile_b, mt_a, profile_a, true)
        } else {
            return None;
        }
    };

    let wp_b = mt_b.trajectory.finish_motion().clone();

    let (i_a, tf) =
        match find_spillover_conflict(mt_a.trajectory.iter(), profile_a, wp_b, profile_b, true) {
            Some(i_a) => i_a,
            None => return None,
        };

    // dbg!((relative_i_a, i_a));
    let mut range_a = mt_a.get_decision_range(i_a);
    // let mut range_b = mt_b.get_decision_range(mt_b.trajectory.len());
    let mut range_b = DecisionRange::After(mt_b.final_state, tf);
    if swapped {
        std::mem::swap(&mut range_a, &mut range_b);
    }

    Some((range_a, range_b))
    // dbg!(Some((range_a, range_b)))
}

fn find_spillover_conflict(
    iter_a: TrajectoryIter<WaypointSE2>,
    profile_a: &CircularProfile,
    wp_b: WaypointSE2,
    profile_b: &CircularProfile,
    // If trailing is true that means we're looking at the indefinite ending of a trajectory.
    // If trailing is false that means we're looking at the indefinite beginning of a trajectory.
    trailing: bool,
) -> Option<(usize, TimePoint)> {
    let bb_b = BoundingBox::for_point(wp_b.point()).inflated_by(profile_b.footprint_radius());
    let wp_b: WaypointR2 = wp_b.into();
    let conflict_distance_squared = profile_a.conflict_distance_squared_for(profile_b);

    for (i_a, [wp0_a, wp1_a]) in iter_a.pairs().enumerate() {
        let wp0_a: WaypointR2 = wp0_a.into();
        let wp1_a: WaypointR2 = wp1_a.into();

        if trailing {
            if wp1_a.time < wp_b.time {
                continue;
            }
        } else {
            if wp_b.time < wp0_a.time {
                break;
            }
        }

        let (wp0_b, wp1_b) = if trailing {
            (wp_b, wp_b.with_time(wp1_a.time))
        } else {
            (wp_b.with_time(wp0_a.time), wp_b)
        };

        let t = if trailing { wp1_a.time } else { wp0_a.time };

        if have_conflict(
            (&wp0_a, &wp1_a),
            None,
            profile_a,
            (&wp0_b, &wp1_b),
            Some(bb_b),
            profile_b,
            conflict_distance_squared,
        ) {
            return Some((i_a, t));
        }
        // if have_conflict(
        //     dbg!((&wp0_a, &wp1_a)), None, profile_a,
        //     dbg!((&wp0_b, &wp1_b)), Some(bb_b), profile_b,
        //     conflict_distance_squared,
        // ) {
        //     dbg!();
        //     return Some(i_a);
        // }
        // dbg!();
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        algorithm::path::DecisionPoint,
        motion::{r2::Positioned, se2::WaypointSE2, Trajectory},
    };
    use std::f64::consts::PI;

    #[derive(Clone, Copy)]
    enum SyntheticScenarioFamily {
        LowOverlap,
        MediumOverlap,
        HighOverlap,
    }

    fn build_synthetic_proposals(
        agent_count: usize,
        waypoint_count: usize,
        family: SyntheticScenarioFamily,
    ) -> (HashMap<usize, Proposal>, Vec<CircularProfile>) {
        assert!(agent_count > 1);
        assert!(waypoint_count >= 2);

        let mut proposals = HashMap::new();
        let profiles = vec![CircularProfile::new(0.35, 0.0, 0.0).unwrap(); agent_count];

        for agent in 0..agent_count {
            let control_points = control_points_for_family(agent, agent_count, family);
            let start_time = start_time_for_family(agent, family);
            let waypoints = sample_polyline(
                &control_points,
                waypoint_count,
                start_time,
                start_time + 20.0,
            );
            let trajectory = Trajectory::from_iter(waypoints.iter().copied()).unwrap();

            let states: Vec<_> = waypoints
                .iter()
                .copied()
                .map(|waypoint| {
                    StateSippSE2::new(Cell::from_point(waypoint.point(), 1.0), waypoint)
                })
                .collect();

            let meta = MetaTrajectory {
                trajectory,
                decision_points: states
                    .iter()
                    .cloned()
                    .enumerate()
                    .map(|(index, state)| DecisionPoint { index, state })
                    .collect(),
                initial_state: states.first().cloned().unwrap(),
                final_state: states.last().cloned().unwrap(),
            };

            proposals.insert(
                agent,
                Proposal {
                    meta,
                    cost: Cost(agent as f64),
                },
            );
        }

        (proposals, profiles)
    }

    fn control_points_for_family(
        agent: usize,
        agent_count: usize,
        family: SyntheticScenarioFamily,
    ) -> Vec<(f64, f64)> {
        match family {
            SyntheticScenarioFamily::LowOverlap => {
                let y = (agent as f64 - agent_count as f64 / 2.0) * 4.0;
                vec![(-12.0, y), (12.0, y)]
            }
            SyntheticScenarioFamily::MediumOverlap => {
                let lane = agent % 6;
                let offset = lane as f64 - 2.5;
                if agent % 2 == 0 {
                    let y = offset * 1.5;
                    vec![(-12.0, y), (-2.0, y), (2.0, y), (12.0, y)]
                } else {
                    let x = offset * 1.5;
                    vec![(x, -12.0), (x, -2.0), (x, 2.0), (x, 12.0)]
                }
            }
            SyntheticScenarioFamily::HighOverlap => {
                let theta = 2.0 * PI * agent as f64 / agent_count as f64;
                let (sin_t, cos_t) = theta.sin_cos();
                vec![
                    (10.0 * cos_t, 10.0 * sin_t),
                    (2.0 * cos_t, 2.0 * sin_t),
                    (0.0, 0.0),
                    (-2.0 * cos_t, -2.0 * sin_t),
                    (-10.0 * cos_t, -10.0 * sin_t),
                ]
            }
        }
    }

    fn start_time_for_family(agent: usize, family: SyntheticScenarioFamily) -> f64 {
        match family {
            SyntheticScenarioFamily::LowOverlap => agent as f64 * 0.05,
            SyntheticScenarioFamily::MediumOverlap => (agent % 4) as f64 * 0.15,
            SyntheticScenarioFamily::HighOverlap => (agent % 8) as f64 * 0.05,
        }
    }

    fn sample_polyline(
        control_points: &[(f64, f64)],
        waypoint_count: usize,
        start_time: f64,
        finish_time: f64,
    ) -> Vec<WaypointSE2> {
        let mut lengths = Vec::new();
        let mut cumulative = Vec::new();
        let mut total_length = 0.0;

        for window in control_points.windows(2) {
            let dx = window[1].0 - window[0].0;
            let dy = window[1].1 - window[0].1;
            let length = (dx * dx + dy * dy).sqrt();
            lengths.push(length);
            cumulative.push(total_length);
            total_length += length.max(1e-9);
        }

        let mut waypoints = Vec::with_capacity(waypoint_count);
        for index in 0..waypoint_count {
            let progress = if waypoint_count == 1 {
                0.0
            } else {
                index as f64 / (waypoint_count - 1) as f64
            };
            let target_length = total_length * progress;

            let mut segment = lengths.len() - 1;
            for (candidate, start_length) in cumulative.iter().enumerate() {
                let end_length = *start_length + lengths[candidate];
                if target_length <= end_length || candidate == lengths.len() - 1 {
                    segment = candidate;
                    break;
                }
            }

            let seg_start = control_points[segment];
            let seg_end = control_points[segment + 1];
            let seg_length = lengths[segment].max(1e-9);
            let local_progress = (target_length - cumulative[segment]) / seg_length;
            let x = seg_start.0 + (seg_end.0 - seg_start.0) * local_progress;
            let y = seg_start.1 + (seg_end.1 - seg_start.1) * local_progress;
            let yaw = (seg_end.1 - seg_start.1).atan2(seg_end.0 - seg_start.0);
            let time = start_time + (finish_time - start_time) * progress;
            waypoints.push(WaypointSE2::new_f64(time, x, y, yaw));
        }

        waypoints
    }

    fn conflict_signature(report: &ConflictDetectionReport) -> Vec<String> {
        report
            .conflicts
            .iter()
            .map(|conflict| format!("{conflict:?}"))
            .collect()
    }

    #[test]
    fn kdtree_matches_baseline_for_low_overlap() {
        let (proposals, profiles) =
            build_synthetic_proposals(24, 16, SyntheticScenarioFamily::LowOverlap);
        let baseline = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::Baseline,
        );
        let kdtree = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::KdTree,
        );

        assert_eq!(conflict_signature(&baseline), conflict_signature(&kdtree));
        assert!(kdtree.stats.pair_enumerations < baseline.stats.pair_enumerations);
    }

    #[test]
    fn kdtree_matches_baseline_for_medium_overlap() {
        let (proposals, profiles) =
            build_synthetic_proposals(32, 24, SyntheticScenarioFamily::MediumOverlap);
        let baseline = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::Baseline,
        );
        let kdtree = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::KdTree,
        );

        assert_eq!(conflict_signature(&baseline), conflict_signature(&kdtree));
        assert!(kdtree.stats.bbox_candidate_pairs <= baseline.stats.bbox_candidate_pairs);
    }

    #[test]
    fn kdtree_matches_baseline_for_high_overlap() {
        let (proposals, profiles) =
            build_synthetic_proposals(20, 24, SyntheticScenarioFamily::HighOverlap);
        let baseline = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::Baseline,
        );
        let kdtree = detect_conflicts_for_proposals(
            &proposals,
            &profiles,
            ConflictDetectionAlgorithm::KdTree,
        );

        assert_eq!(conflict_signature(&baseline), conflict_signature(&kdtree));
        assert_eq!(baseline.stats.conflicts_found, kdtree.stats.conflicts_found);
    }
}
