# FOCAL Search Benchmark Results

Benchmark conducted on `arjoc/temp-benchmark-focal` branch.

## Configuration
- **Maps:** `room-32-32-4.map`, `maze-32-32-2.map`
- **Scenarios:** 2 per map (random-1, random-10)
- **Agents:** 5, 10
- **Timeout:** 15 seconds
- **Focal Weight:** 1.1
- **Focal Heuristic:** Number of conflicts

## Results

| Scenario | Agents | Algorithm | Status | Time |
|----------|--------|-----------|--------|------|
| room-32-32-4:random-1 | 5 | baseline | SUCCESS | 0.41s |
| room-32-32-4:random-1 | 10 | baseline | SUCCESS | 0.39s |
| room-32-32-4:random-10 | 5 | baseline | SUCCESS | 0.36s |
| room-32-32-4:random-10 | 10 | baseline | SUCCESS | 0.40s |
| maze-32-32-2:random-1 | 5 | baseline | SUCCESS | 0.39s |
| maze-32-32-2:random-1 | 10 | baseline | SUCCESS | 0.46s |
| maze-32-32-2:random-10 | 5 | baseline | SUCCESS | 0.45s |
| maze-32-32-2:random-10 | 10 | baseline | Timeout | 15.00s |

## Observations
- FOCAL search with conflict count heuristic is integrated into the negotiation loop.
- Performance is comparable to baseline for simpler scenarios.
- Timeouts still occur for 10 agents in the more complex maze scenario.
