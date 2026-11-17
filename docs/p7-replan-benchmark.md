# P7-03 planner comparison decision

## Decision

Keep the current weighted A* planner. Do not integrate D* Lite: it did not
meet the predeclared replan-latency gate on this matched route set. The
benchmark-only implementation and full trial records are retained so the
comparison can be repeated if map size or update patterns change.

## Frozen comparison

The benchmark reuses the three P4-03 occupancy routes. Each route receives one
fixed obstacle-map update that changes its feasible route while keeping its
start and goal valid. Both algorithms plan on the initial map and then produce
a route after that same update. Trial order alternates between algorithms.
There are three paired trials per route, nine matched pairs, and 18 retained
algorithm attempts.

The adoption gate was recorded in
`config/p7_replan_benchmark.json` before the timing run. D* Lite would be
adopted only if all attempts succeeded, its overall median replan time improved
by at least 20%, it was faster on at least two of the three route medians, and
its worst route cost regression was at most 1%.

## Results

All 18 attempts succeeded in both the original run and a fresh 2026-11-16
retry. The retry returned equal weighted path costs on all three scenarios.
Weighted A* had a 3.105 ms overall median replan time; D* Lite had a 20.659 ms
median. D* Lite was faster on one route only, by 3.5%, and slower on the other
two. Its overall speedup was -565.4% (about 6.65 times slower), so the frozen
adoption gate failed. The measured cost regression was 0%.

The existing production planner remains weighted A*. `scripts/dstar_lite.py`
is an isolated benchmark candidate and is not connected to ROS or motion
commands.

## Scope

These timings use Python `perf_counter()` on deterministic occupancy grids.
They cover planning and incremental map repair only; they exclude ROS message
transport, sensor acquisition, map fusion, and rover motion. The route set is
small (20–40 cells per axis), and each route has only three trials. This is
enough to reject D* Lite for the current bounded workload, not to claim that it
is slower for larger maps or different update density. Rebenchmark before any
future planner migration.

Evidence: `evidence/runs/20261116-p703-retry/benchmark.json` and the original
`evidence/runs/20261112-p703/P7-03/benchmark.json` / `summary.json`.
