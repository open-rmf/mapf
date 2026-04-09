use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use mapf::{
    algorithm::path::{DecisionPoint, MetaTrajectory},
    domain::Cost,
    graph::occupancy::Cell,
    motion::CircularProfile,
    motion::{r2::Positioned, se2::WaypointSE2, Trajectory},
    negotiation::{detect_conflicts_for_proposals, ConflictDetectionAlgorithm, Proposal},
    premade::StateSippSE2,
};
use std::{collections::HashMap, f64::consts::PI};

#[derive(Clone, Copy)]
enum SyntheticScenarioFamily {
    LowOverlap,
    MediumOverlap,
    HighOverlap,
}

fn bench_conflict_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("conflict_detection");

    for family in [
        SyntheticScenarioFamily::LowOverlap,
        SyntheticScenarioFamily::MediumOverlap,
        SyntheticScenarioFamily::HighOverlap,
    ] {
        for (agents, waypoints) in [(16, 16), (32, 24), (64, 32)] {
            let case = build_synthetic_proposals(agents, waypoints, family);
            let label = format!("{}-a{}-w{}", family_name(family), agents, waypoints);

            group.bench_with_input(
                BenchmarkId::new("baseline", &label),
                &case,
                |b, (proposals, profiles)| {
                    b.iter(|| {
                        black_box(detect_conflicts_for_proposals(
                            proposals,
                            profiles,
                            ConflictDetectionAlgorithm::Baseline,
                        ))
                    });
                },
            );

            group.bench_with_input(
                BenchmarkId::new("kdtree", &label),
                &case,
                |b, (proposals, profiles)| {
                    b.iter(|| {
                        black_box(detect_conflicts_for_proposals(
                            proposals,
                            profiles,
                            ConflictDetectionAlgorithm::KdTree,
                        ))
                    });
                },
            );
        }
    }

    group.finish();
}

criterion_group!(benches, bench_conflict_detection);
criterion_main!(benches);

fn family_name(family: SyntheticScenarioFamily) -> &'static str {
    match family {
        SyntheticScenarioFamily::LowOverlap => "low",
        SyntheticScenarioFamily::MediumOverlap => "medium",
        SyntheticScenarioFamily::HighOverlap => "high",
    }
}

fn build_synthetic_proposals(
    agent_count: usize,
    waypoint_count: usize,
    family: SyntheticScenarioFamily,
) -> (HashMap<usize, Proposal>, Vec<CircularProfile>) {
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
            .map(|waypoint| StateSippSE2::new(Cell::from_point(waypoint.point(), 1.0), waypoint))
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
