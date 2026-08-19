//! Plain `Instant` timing, no benchmarking crate. See `post::timing::grid_scene`
//! in `src/post/mod.rs` for why this scene is adversarial to the sweep.

use mapf::post::{
    mapf_post, mapf_post_sweep,
    na::{Isometry2, Vector2},
    shape::{Ball, Shape},
    MapfResult, SemanticPlan, Trajectory,
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

fn grid_scene(n: usize, num_waypoints: usize) -> MapfResult {
    let side = (n as f64).sqrt().ceil() as usize;
    let cell_spacing = 3.0;
    let wiggle = 0.4;
    let denom = (num_waypoints.max(2) - 1) as f64;

    let mut trajectories = Vec::with_capacity(n);
    let mut footprints = Vec::with_capacity(n);
    for idx in 0..n {
        let col = (idx % side) as f64;
        let row = (idx / side) as f64;
        let cx = col * cell_spacing;
        let cy = row * cell_spacing;
        let poses = (0..num_waypoints)
            .map(|wp_idx| {
                let t = wp_idx as f64 / denom * std::f64::consts::TAU;
                Isometry2::new(
                    Vector2::new(cx + t.sin() * wiggle, cy + t.cos() * wiggle),
                    0.0,
                )
            })
            .collect();
        trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.49)) as Arc<dyn Shape>);
    }
    MapfResult {
        trajectories,
        footprints,
        discretization_timestep: 1.0,
        agent_name_to_id: HashMap::default(),
    }
}

fn time<F: FnMut()>(mut f: F, iters: u32) -> Duration {
    let start = Instant::now();
    for _ in 0..iters {
        f();
    }
    start.elapsed() / iters
}

fn main() {
    let agent_counts = [
        1usize, 10, 50, 100, 500, 1000, 2000, 4000, 8000, 16000,
    ];
    let traj_lengths = [1usize, 10, 50];

    for (strategy_name, strategy) in [
        ("sweep", mapf_post_sweep as fn(&MapfResult) -> SemanticPlan),
        ("aabb_tree", mapf_post as fn(&MapfResult) -> SemanticPlan),
    ] {
        println!("\n== {strategy_name}: agents (rows) x trajectory length (columns) ==");
        print!("{:>10}", "agents");
        for &tl in &traj_lengths {
            print!(" {:>14}", format!("len={tl}"));
        }
        println!();

        for &n in &agent_counts {
            print!("{n:>10}");
            for &tl in &traj_lengths {
                let scene = grid_scene(n, tl);
                let segments = n * tl.saturating_sub(1);
                let iters = if segments > 5_000 { 1 } else { 3 };
                let elapsed = time(
                    || {
                        std::hint::black_box(strategy(&scene));
                    },
                    iters,
                );
                print!(" {elapsed:>14?}");
            }
            println!();
        }
    }
}
