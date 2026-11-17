# P10-02 — Reproducible scenario suite

`config/evaluation/scenarios.json` freezes five evaluation cases, the 40x30
grid geometry, terrain-cost weight, seed, and three trials per case. Run them
with:

```bash
python3 tools/run_scenarios.py --output evidence/runs/<run-id>/P10-02/scenario-results.json
python3 -m unittest tests/test_evaluation_scenarios.py -v
```

The runner executes the existing weighted A* helper on matched occupancy and
terrain grids. It records every trial, both path lengths and costs, planning
times, seeded roughness, scenario acceptance checks, and obstacle-triggered
replan results. The unexpected-obstacle case requires the original route to be
intersected, a changed route after the map update, and a traversable result.
Crater and uncertain-terrain cases require the terrain-cost route to avoid the
cost region while recording the occupancy-only route against the same map.

## Scope and limits

These 15 trials are deterministic **grid-planner fixtures**, not 15 Gazebo
mission trials. They verify scenario setup and planner behavior, not rover
motion, physical collision response, sensor-based obstacle detection, or
live-node latency. The report marks physical collision counts and localization
error unavailable, with reasons; it also lists motion/resource metrics that
this runner cannot measure. Do not interpret unavailable values as zero or as
passing results.

The full outbound delivery and return mission has separate accepted runtime
evidence from P8-03: three independent Habitat-Science-Habitat round trips, nine
completed legs, and three durable delivery events. That evidence is linked in
the P10-02 output but was not rerun or rescored by this scenario harness. The
other four scenarios still need live mission trials before broad runtime claims
are made.

The runner stores the configuration hash, runner hash, Python version, suite
seed, per-attempt seed, timestamps, all individual attempts, and the separate
P8-03 runtime-evidence reference. The cross-trial report and metric
availability accounting are in `evidence/runs/20261113-p1003/P10-03/`.
