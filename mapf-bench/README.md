# `mapf-bench`

`mapf-bench` is a binary crate designed to run single Multi-Agent Path Finding (MAPF) benchmark scenarios using the `mapf` library's negotiation solver.

Currently, it supports benchmarks from the [Moving AI MAPF benchmark collection](https://www.movingai.com/benchmarks/mapf.html).

## Relationship with `scripts/benchmark.py`

While `mapf-bench` can be run directly to test a single scenario, it is intended to be used in conjunction with the Python orchestrator script located at `scripts/benchmark.py`.

*   **`scripts/benchmark.py`**: Automates downloading the Moving AI benchmark files (maps and scenarios), unzipping them, building `mapf-bench` in release mode, and running it across multiple configurations (different maps, scenarios, and agent counts) to generate a comprehensive performance report.
*   **`mapf-bench`**: The low-level runner that loads a specific map and scenario file, sets up the negotiation scenario, and runs the solver for a single run.

## Intended Workflow

To run a full benchmark suite:

1.  Use the Python script from the root directory:
    ```bash
    python3 scripts/benchmark.py --timeout 30 --max-scenarios 1 --maps empty-32-32.map
    ```
    This script will handle building this crate and running it.

To run a single scenario manually for debugging:

1.  Build the crate:
    ```bash
    cargo build -p mapf-bench --release
    ```
2.  Run the binary, providing paths to the map and scenario files:
    ```bash
    cargo run -p mapf-bench --release -- --map cache/maps/empty-32-32.map --scen cache/scenarios/scen-random/empty-32-32-random-1.scen --num-agents 10
    ```

## Command Line Arguments

`mapf-bench` accepts the following arguments:

*   `-m`, `--map <MAP>`: Path to the Moving AI map file (`.map`).
*   `-s`, `--scen <SCEN>`: Path to the Moving AI scenario file (`.scen`).
*   `-r`, `--radius <RADIUS>`: Radius of the agents (default: 0.45).
*   `--speed <SPEED>`: Speed of the agents (default: 1.0). (Note: no short option to avoid conflict with `--scen`).
*   `-n`, `--num-agents <NUM_AGENTS>`: Number of agents to include in the scenario (default: 10).
*   `-t`, `--timeout <TIMEOUT>`: Timeout in seconds (default: 30). Note: This timeout is currently enforced by the parent process (`benchmark.py`) rather than the binary itself.

## Map Format

The benchmark loader parses Moving AI `.map` files. The characters in the grid are interpreted as follows:
*   `.` : Passable terrain.
*   Any other character : Obstacle (impassable).
