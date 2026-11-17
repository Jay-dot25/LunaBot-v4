# P3-01 wheel odometry qualification

## Outcome

**P3-01 passes its three-trial simulated accuracy gate after effective-track
calibration.** The WSL qualification profile uses wheel separation 1.03 m,
selected from separate calibration trials and then held fixed for three fresh
trials. Straight errors were 0.0019–0.0039 m; turn errors were 0.0879–0.0965
rad. The estimator remains wheel-position dead reckoning and remains slip
sensitive; these results qualify the recorded Gazebo profile, not physical
hardware or long-duration drift.

## Source and frames

The Gazebo Sim 6.18 DiffDrive plugin is configured with three wheels per side.
Its SDF lists the front left/right pair first, radius 0.17 m, and calibrated
effective separation 1.03 m. The chassis wheel centerlines remain at their
modeled geometric spacing; this is the effective track used by DiffDrive to
account for measured skid-steer turn response.
The plugin sends velocity commands to all configured wheels, but integrates the
joint positions of leftJoints[0] and rightJoints[0]. It publishes
/lunabot/odom and a TF message copied from the same integrated pose:
odom -> chassis. This is wheel motion, not simulator world pose or GPS.

Pinned primary source:
https://raw.githubusercontent.com/gazebosim/gz-sim/ignition-gazebo6_6.18.0/src/systems/diff_drive/DiffDrive.cc
See PreUpdate lines 460–475 and UpdateOdometry lines 490–575.
Runtime TF lookup passed in all three normal trials. The SDF has one DiffDrive
plugin, and launch inspection found no second odom -> chassis broadcaster.
Fresh launches returned odometry to origin and actual Odometry message stamps
were monotonic in all three trials. This verifies fresh-world/process reset;
in-process simulation-time rewind remains outside this task.

## Frozen accuracy criteria

- Truth and odometry align within 0.02 s simulation time.
- Straight endpoint position error <= max(0.10 m, 10% of truth distance).
- A 1.5 rad in-place turn has yaw error <= 0.10 rad.
- Fresh run odometry origin <= 0.02 m and 0.02 rad; message stamps increase.
- Controlled low-friction change yields a measurable, explicitly reported wheel
  versus ground-truth discrepancy.
- Runtime TF lookup succeeds and source ownership of odom -> chassis is unique.

## Historical synchronized trials using 0.80 m track (2026-11-03)

The evaluator-only C++ recorder pairs Gazebo model poses with /clock. It writes
truth to an evidence CSV and publishes no operational ROS topics. The analyzer
matches each Phase B command/odometry sample to the nearest simulation stamp.
Measured nearest-sample error was <=0.009 s in these trials.

| Run | Straight odom / truth (m) | Straight error | Turn odom / truth (rad) | Turn error | Reset, stamps, TF | Result |
|---|---:|---:|---:|---:|---|---|
| 1 | 0.837 / 0.844 | 0.007 m (pass) | 1.558 / 0.260 | 1.298 rad (fail) | pass | Accuracy fail |
| 2 | 1.089 / 0.892 | 0.197 m (fail) | 1.566 / 0.003 | 1.563 rad (fail) | pass | Accuracy fail |
| 3 | 1.089 / 0.892 | 0.197 m (fail) | 1.563 / 0.047 | 1.516 rad (fail) | pass | Accuracy fail |

All three historical turn trials failed the yaw criterion. Two of three straight trials exceeded
the 0.10 m tolerance. This makes the current DiffDrive pose unsuitable as the
sole localization source. Straight/turn repeatability also varies by trial.

## Effective-track calibration and frozen retry (2026-11-16)

The 0.80 m setting repeatedly over-reported the 1.5 rad wheel turn against
Gazebo truth. Under the qualified software-rendered WSL profile, two independent
0.6 rad/s trials measured truth/odom turn ratios of 0.799 and 0.771. Copied
SDFs were used for calibration; 1.02 m missed the yaw gate by 0.0015 rad, so
the candidate was adjusted to 1.03 m. The canonical model was changed only
after that candidate passed the frozen three-trial qualification below.

| Qualification run | Straight odom / truth (m) | Straight error | Turn odom / truth (rad) | Turn error | Alignment / reset / stamps | Result |
|---|---:|---:|---:|---:|---|---|
| 1 | 1.092 / 1.094 | 0.0019 m | 1.570 / 1.478 | 0.0920 rad | pass; <=0.008 s alignment | PASS |
| 2 | 1.082 / 1.086 | 0.0038 m | 1.563 / 1.475 | 0.0879 rad | pass; <=0.008 s alignment | PASS |
| 3 | 1.089 / 1.092 | 0.0027 m | 1.570 / 1.474 | 0.0965 rad | pass; <=0.007 s alignment | PASS |

All three fresh trials passed the unchanged straight, turn, reset, stamp, and
truth-alignment gates. Candidate and trial evidence are in
`evidence/runs/20261116-p301-retry/calibration/` and
`evidence/runs/20261116-p301-retry/qualification-fixed/`. The standard
actuator watchdog remains 0.5 s. The derived WSL profile changes only the
health-report scheduling timeout to 2.0 s and uses software rendering.

## Slip and cause checks

A copied low-friction rover SDF reduced wheel collision mu/mu2 from 4.0 to 0.02
and increased configured slip1/slip2 from 0.01 to 0.2. It was loaded only via
the optional Phase B fixture path; the canonical SDF was not edited. The
wheel-derived straight distance was 1.077 m while ground truth moved 0.130 m,
a 0.947 m discrepancy. This confirms the odometry integrates wheel motion and
does not compensate for wheel-ground slip.

A copied world with terrain mu/mu2=4.0 improved straight error in its single
diagnostic trial (0.006 m) but still failed turn yaw by 1.380 rad. A separate
0.2 rad/s turn trial still failed yaw by 0.570 rad. These checks do not justify
editing canonical friction, steering, geometry, or estimator settings. No live
slip alarm is claimed; truth remains evaluator-only. The health monitor checks
freshness and validity, not wheel-ground traction.

## Decision and next use

Keep DiffDrive as a local prior with the measured 1.03 m effective track.
Wheel odometry remains slip-sensitive, so use qualified scan SLAM for the
GPS-independent global correction path. The controlled low-friction trial
remains the explicit limit on this qualification; no live traction alarm or
physical-hardware accuracy claim is made.

## Reproduction

From /home/jay/lunabot-v4, run one fresh headless synchronized trial:
python3 tools/run_odom_qualification.py --run-dir evidence/runs/20261103-p301/trial-N

For the controlled low-friction trial, pass the copied fixture:
python3 tools/run_odom_qualification.py --run-dir evidence/runs/20261103-p301/slip-repeat --model evidence/runs/20261103-p301/slip-fixture/model-low-friction.sdf

The runner builds the evaluator recorder when absent, launches it before
launch-b.sh, saves run_command.json and logs, then invokes
tools/analyze_odom_truth.py. Each run directory must be new.
The runner itself was exercised end-to-end as trial-4; truth synchronization
and reset passed, straight error was 0.003 m, and the turn accuracy gate failed.

Full commands, source findings, per-run metrics, generated fixtures, and logs:
evidence/runs/20261103-p301/summary.json. Individual data and qualification
JSON files remain under trial-1 through trial-4, slip-trial,
high-friction-trial, and slow-turn-trial.

P3-01 passes for the recorded three-trial WSL Gazebo profile. This does not
qualify physical hardware or long-duration drift.
