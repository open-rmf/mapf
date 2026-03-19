# Conflict Detection First Slice

This first slice keeps the production negotiation path on the baseline algorithm and adds an experimental KD-tree broad phase for comparison.

## What changed

- `mapf::negotiation::detect_conflicts_for_proposals(...)` can now run:
  - `ConflictDetectionAlgorithm::Baseline`
  - `ConflictDetectionAlgorithm::KdTree`
- The KD-tree path prunes proposal pairs using trajectory bounding boxes before calling the exact `find_first_conflict(...)` logic.
- Existing negotiation behavior still uses the baseline detector.

## Why this slice is useful

- It isolates the performance-sensitive conflict-detection pass without changing planner semantics.
- It gives a reproducible benchmark path for a mentor conversation.
- It proves whether a KD-tree-style broad phase is promising before attempting deeper integration.

## Commands

Build and test:

```bash
CARGO_HOME=$PWD/.cargo_home cargo test -p mapf
```

Print a baseline vs KD-tree report:

```bash
CARGO_HOME=$PWD/.cargo_home cargo run --release -p mapf --example conflict_detection_report
```

Run criterion benchmarks:

```bash
CARGO_HOME=$PWD/.cargo_home cargo bench -p mapf --bench conflict_detection
```

## Interpretation

- `pair_enumerations`: how many proposal pairs the algorithm examined.
- `bbox_candidate_pairs`: how many pairs survived broad-phase pruning and needed exact checking.
- `conflicts`: how many exact conflicts were found.

The KD-tree path is expected to reduce `pair_enumerations` and usually reduce `bbox_candidate_pairs`, while returning the same conflict set as the baseline.
