mod movingai;

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;
use std::time::Instant;

/// Run a single MAPF benchmark scenario using the negotiation solver.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the Moving AI map file (.map)
    #[arg(short, long)]
    map: PathBuf,

    /// Path to the Moving AI scenario file (.scen)
    #[arg(short, long)]
    scen: PathBuf,

    /// Radius of the agents
    #[arg(short, long, default_value_t = 0.45)]
    radius: f64,

    /// Speed of the agents
    #[arg(long, default_value_t = 1.0)]
    speed: f64,

    /// Number of agents to include in the benchmark
    #[arg(short, long, default_value_t = 10)]
    num_agents: usize,

    /// Timeout in seconds (enforced by parent process)
    #[arg(short, long, default_value_t = 30)]
    timeout: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("Loading map: {:?}", args.map);
    let map = movingai::Map::from_file(&args.map)?;

    println!("Loading scenario: {:?}", args.scen);
    let scenario = movingai::MovingAIScenario::from_file(&args.scen)?;

    let negotiation_scenario = scenario.to_negotiation_scenario(
        &map,
        args.num_agents,
        args.radius,
        args.speed,
        60_f64.to_radians(),
    );

    println!("Running negotiation with {} agents", args.num_agents);
    let start_time = Instant::now();
    // We don't have a clean way to pass wall-clock timeout into negotiate yet,
    // so we'll rely on the parent script to enforce strict timeouts for now,
    // or we could add a QueueLengthLimit as a proxy.
    let result = mapf::negotiation::negotiate(&negotiation_scenario, None);
    let duration = start_time.elapsed();

    match result {
        Ok((_solution, _arena, _name_map)) => {
            println!("Negotiation successful in {:?}", duration);
        }
        Err(e) => {
            println!("Negotiation failed: {:?}", e);
        }
    }

    Ok(())
}
