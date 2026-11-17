# P2-03 sensor/state health, actuator timeout and restart handling

## Operational architecture
Raw onboard streams -> health_monitor.py -> /lunabot/safety/health ->
control_odometry.py -> ROS /cmd_vel -> bridge remapping ->
Ignition /lunabot/actuator_request -> native CommandWatchdog ->
Ignition /cmd_vel -> existing DiffDrive.

The ROS controller remains the one configured ROS command publisher. The native
guard is the sole configured Ignition actuator-command publisher. These are
different transports despite the shared final topic spelling. Simulator world
pose is not used by either layer.

Raw RGB-D and high-rate joint validation run in a separate process. This prevents
their Python callbacks from blocking the controller's small command heartbeat
executor. Raw queues are bounded to one message. The actuator guard runs inside
Gazebo and survives termination of the ROS controller and bridge.

## Input and freshness contract
Existing input frames, rates and calibration are described in sensor-contracts.md.
The monitor validates RGB/depth payload structure, depth observations, CameraInfo,
LiDAR ranges, IMU/odometry finite values and orientation, named mast joints, and
advancing acquisition stamps. JointState uses an empty header frame (as observed);
the joint names define its coordinates.

Defaults in config/safety/control.json:
- Clock progress: 0.5s wall time.
- Raw input receipt: 1s wall time; raw acquisition age: 0.25s simulation time.
- Future timestamp tolerance: 0.1s simulation time.
- Health report receipt/source age: 0.5s wall time.
- Path validity: 3s, when a configured path is needed for nonzero motion.
- SLAM TF stamp offset: 0.2s, matching the existing transform_timeout setting.

A-B use onboard odometry health. C-L additionally require map->odom; D-H use
/plan, I-L /lunabot/terrain/plan for motion freshness. Pose/TF health here is
structural and temporal; localization accuracy/covariance qualification remains P3.

slam_toolbox 2.6.10 publishes TF at scan_stamp + transform_timeout and repeats
the stamp between scans. The offset is removed for health-age checks; repeated
TF publication never refreshes measurement age. Reference:
[slam_toolbox 2.6.10 TF publisher](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.6.10/src/slam_toolbox_common.cpp)
The async callback updates scan_header before scan-processing selection:
[async scan callback](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.6.10/src/slam_toolbox_async.cpp)
Measured processed-TF age sample: median 0.097s, p95 0.170s, maximum 0.200s.
See localization-age.json; this measures timing, not localization error.

## Health report protocol
/lunabot/safety/health is std_msgs/String containing JSON schema 1:
instance (boot UUID), sequence, epoch (clock-reset generation), created_monotonic,
require_map, path_topic, reason, motion_reason and sim_time.
The controller rejects stale/replayed/malformed reports, wrong map/path profiles and
multiple health publishers. Both processes use the same WSL host monotonic clock;
this source-age protocol is local-host, not a distributed clock-sync solution.

A monitor restart or clock epoch change latches a controller fault. Missing,
stale or invalid required data also latches a fault. Recovery does not resume
motion. A healthy profile, fresh zero request and explicit reset are required.

## Startup and actuator behavior
A new controller starts STARTUP_INHIBIT. A launcher performs a one-shot
/lunabot/safety/arm_startup Trigger after required startup work. The service
only arms a new startup state; it never clears a runtime fault. Duplicate success
acknowledgment is idempotent and changes no state, covering a delayed/lost RPC
response. A held old motion request cannot resume a restarted controller.

The native watchdog in src/lunabot_gazebo/src/command_watchdog.cc:
- receives only guarded Ignition requests and forwards to the existing DiffDrive;
- uses steady wall-clock receipt age, with a 0.25s timeout;
- publishes zero and latches after timeout, invalid command or simulation rewind;
- requires a fresh zero handshake before permitting later nonzero requests.

The controller publishes zero while inhibited, so a restarted native guard can
be ready while controller motion permission remains inhibited. Native READY does
not mean the mission/controller is armed. All legacy launchers map the ROS command
to the request channel, and require this native build. Missing guard/feedback
therefore prevents normal startup arming.

scripts/prepare_safety_plugin.sh builds the native target into ignored .build/.
ROS package builds also install it; the reusable launch supplies its installed
library path. No ROS/Gazebo distribution was changed or packages installed.

## Failure and cleanup evidence
Live tests use actual sensors and commands, not synthetic demonstration data.
tests/run_safety_faults.py starts a held Phase B stack and runs
tests/test_safety_faults_live.py, with bounded teardown.
The suite suspends the health monitor and bridge, kills/restarts the controller,
pauses/resumes the simulation, and starts a matching map-required profile without
SLAM. It verifies stopping, latched recovery, explicit rearm, and no stale-command
resume. Wheel odometry is used for settled-stop observations, not world truth.

tests/test_safety_health.py uses explicitly synthetic unit fixtures for invalid,
missing, old/future/replayed inputs, clock jumps, path/localization loss and
health-report protocol failures. These fixtures are not production sensor data.

Process teardown is centralized in scripts/process_helpers.sh. The odometry
monitor signal handler only requests exit; ROS teardown runs in normal flow.
A TERM-ignoring test group was forcibly stopped in about 2.05s by the bounded
helper. Fault-test launcher exit 130 is expected after the harness's requested
SIGINT; a forced launcher kill is not an accepted clean result.

Evidence root: evidence/runs/20261103-p203/P2-03/.
- final-qualified/faults.json and traces.json: final live outcomes and raw timing.
- final-qualified/exit.json: test and launcher exit codes.
- health-fixtures-final.log: unit fixtures.
- bounded-cleanup.json: TERM-ignoring group test.
- slam-split/: real SLAM-enabled drive regression.
- static-final.json and static-*-final.log: all A-L static gates.
- package-build-final.log / package-launch-final.log / dependencies-final.log.

Static gates ran in an unchanged temporary copy. Each gate executed once;
recursive prerequisites reused only actual recorded results from that same
snapshot. No checks or expected counts were removed. Historical evidence was
preserved. Earlier failed attempts remain in their original run directories.

## Remaining boundaries
This is simulation actuator behavior, not real hardware safety certification.
ROS/Gazebo topic access control against arbitrary publishers is not implemented.
Health validates estimated-state structure/timing, not correctness against truth.
Actual SLAM loss while navigating and every A-L runtime combination still need
broader integration coverage; current live loss test requires missing SLAM and
blocks arming. Backward/forward clock jumps are fixture-tested; pause/resume is
live-tested. Full missions, collision avoidance, ML inference-health integration,
and quantitative localization accuracy belong to later tasks.
