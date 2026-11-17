# P10-01 — Evaluation contract and truth boundary

## Scope

`config/evaluation/metrics.json` is the versioned metric catalog for P10. Each
metric declares its unit, reference source, aggregation, gate state, and
evidence source. It covers localization, path length and error, travel time,
mission success, collisions, clearance, terrain exposure and unsafe-terrain
entry, perception IoU, replanning, recovery, and resource use.

The established localization qualification limits are position error <= 0.25 m
and yaw error <= 0.15 rad, with reference samples aligned within 0.02 s. These
are existing short-route qualification limits, not a claim that broader routes
or all missions have passed. The perception gate applies only to the active
three-class held-out checkpoint. Scenario-specific acceptance gates are frozen
for P10-02's planner fixtures; live-mission gates remain unverified. Resource
limits belong to P11-01. Missing reference instrumentation is reported as
unavailable, never as a zero count or a pass.

## Truth boundary

Gazebo reference pose is captured over Gazebo Transport from
`/world/lunar_world/pose/info` and `/clock` by the standalone
`tools/capture_gazebo_truth.cc` recorder. `tools/analyze_odom_truth.py` and
`tools/run_odom_qualification.py` are evaluation-only consumers. Truth is not
bridged into ROS and no operational node, planner, controller, mission manager,
or launcher may subscribe to or read the simulator pose interface. Operational
decisions remain based on sensor-derived ROS interfaces.

`tools/validate_evaluation_contract.py` checks the catalog and scans declared
operational source paths for simulator-truth interface markers. Add new
operational source roots to `operational_paths` when the architecture expands.
An evaluator may join truth with recorded operational outputs by simulation
time, but must keep the truth data and comparison outside the live decision
path.

## Verification

```bash
python3 tools/validate_evaluation_contract.py
python3 -m unittest tests/test_evaluation_contract.py -v
python3 -m py_compile tools/validate_evaluation_contract.py tests/test_evaluation_contract.py
```

P10-01 does not implement scenario runners or claim five-scenario results;
those are P10-02 and P10-03.
