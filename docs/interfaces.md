# LunaBot V4 interface inventory

For each interface record endpoints, units, frame, clock/stamp, measured rate, QoS, owner and stale/failure behavior. Configured rates do not prove runtime.

| Name | Type and flow | Contract |
|---|---|---|
| /lunabot/camera/image_raw | sensor_msgs/Image bridge→perception | RGB optical frame, sensor stamp |
| /lunabot/depth/image_raw | sensor_msgs/Image bridge→projection | Encoding units; synchronize RGB/prediction |
| /lunabot/lidar/scan | sensor_msgs/LaserScan bridge→SLAM/detector | LiDAR frame, metres/radians |
| /lunabot/imu | sensor_msgs/Imu bridge→qualified fusion/evaluator | IMU frame, SI, covariance |
| /lunabot/joint_states | sensor_msgs/JointState bridge→diagnostics | Positions/rates; estimator use needs evidence |
| /lunabot/odom | nav_msgs/Odometry DiffDrive→navigation | odom→chassis; qualify wheel source P3-01 |
| /tf, /tf_static | tf2_msgs/TFMessage owners→consumers | One publisher per edge |
| /map | nav_msgs/OccupancyGrid SLAM→planners | Map identity/revision/resolution |
| /initialpose | geometry_msgs/PoseWithCovarianceStamped operator/relocalization→slam_toolbox | Map frame, covariance, deliberate seed; never evaluator truth |
| /goal_pose | geometry_msgs/PoseStamped operator/mission→planner | Validate frame/map/goal identity |
| /cmd_vel_in | geometry_msgs/Twist planner→safety | Request only, chassis m/s/rad/s |
| /cmd_vel | geometry_msgs/Twist safety→DiffDrive | Sole authority; zero on fault |
| /lunabot/terrain/segmentation | sensor_msgs/Image inference→mapper | mono8 labels 0 unknown, 1 terrain, 2 obstacle; image stamp |
| /lunabot/terrain/segmentation_confidence | sensor_msgs/Image perception→mapper | aligned mono8 decision-strength confidence, 0–255; heuristic confidence is not a calibrated probability |
| /lunabot/terrain/semantic_map | nav_msgs/OccupancyGrid mapper→cost | -1 unknown, 25 terrain, 100 obstacle; rolling bounded map |
| /lunabot/terrain/semantic_map/confidence | nav_msgs/OccupancyGrid mapper→diagnostics | aligned effective confidence 0–100, -1 unknown |
| /lunabot/terrain/semantic_map/age | nav_msgs/OccupancyGrid mapper→diagnostics | aligned age in whole seconds, capped at 100; -1 unknown |
| /lunabot/terrain/semantic_map/slope | nav_msgs/OccupancyGrid mapper→cost | aligned elevation-gradient risk, 0–100; -1 insufficient geometry |
| /lunabot/terrain/semantic_map/roughness | nav_msgs/OccupancyGrid mapper→cost | aligned local height-spread risk, 0–100 |
| /lunabot/terrain/semantic_map/crater_risk | nav_msgs/OccupancyGrid mapper→cost | aligned bowl-depression candidate risk, 0–100 |
| /lunabot/terrain/semantic_map/uncertainty | nav_msgs/OccupancyGrid mapper→cost | aligned confidence/age/geometry uncertainty, 0–100 |
| /lunabot/terrain/cost_map | nav_msgs/OccupancyGrid mapper→A* | Shared geometry/revision |
| /lunabot/terrain/plan | nav_msgs/Path A*→follower | Map frame, stamp, goal/revision validity |
| /lunabot/dashboard/telemetry | std_msgs/String adapter→dashboard | Read-only JSON metadata for views 1–5: source stamp/frame when present, source/receive age, freshness reason, ROS type and publisher-node provenance; original source topics are not republished |
| /lunabot/dashboard/command_request | std_msgs/String dashboard→guarded gateway | JSON action request; never a velocity interface. Freshness and confirmation policy is in docs/dashboard-controls.md |
| /lunabot/dashboard/control_status | std_msgs/String guarded gateway→dashboard | Request acceptance/result, gate reason, mission and safety receive ages; a dispatch acknowledgment is not mission-state authority |
| Status topics obstacle/autonomy/replan/evaluation/mission | std_msgs/String monitors→UI | Preserve names; verify QoS/publishers |

No mission action is claimed to exist; define during P8. Runtime-confirm existing contracts.
P2-01 update: corrected camera physical pitch, scoped optical/IMU/LiDAR frames, CameraInfo bridges and JointState type. Sensor-head TF follows measured mast joints. Phase B live acceptance PASS; A/B/C static gates PASS in isolated copy; full A-L runtime remains unverified. Measured contracts, QoS evidence, failure behavior and registration limits: docs/sensor-contracts.md; evidence/runs/20261103-p201/P2-01/summary.json.

P2-02 update: sole configured controller routing for A-L; latched emergency stop/reset services, finite-command validation, steady-clock watchdog. DDS and Phase B live acceptance PASS; Phase A smoke PASS. Interfaces, evidence and remaining process/restart/freshness limits: docs/safety-control.md and evidence/runs/20261103-p202/P2-02/summary.json.

## P2-03 additions and ownership clarification
- /lunabot/safety/health: std_msgs/String JSON schema 1, health_monitor publisher
  at 20Hz, controller subscriber, bounded queue depth 1. Includes local monotonic
  creation time, boot instance, sequence, clock epoch, map/path profile and
  ordinary/motion health reasons. Stale, replayed, mismatched or malformed data
  inhibits motion. This timing protocol assumes both nodes share one WSL host.
- /lunabot/safety/arm_startup: std_srvs/Trigger. One-shot startup authorization;
  repeated success acknowledgment changes no state and runtime faults are refused.
- ROS /cmd_vel is the controller request output, not the native watchdog output.
  The bridge remaps that ROS input to Ignition /lunabot/actuator_request.
- Ignition /cmd_vel is published by the native watchdog to existing DiffDrive.
- /lunabot/actuator_watchdog/state: std_msgs/String bridged from native state,
  approximately 50Hz wall time. READY is a guard zero-handshake state, not
  controller/mission motion permission. Health requires fresh READY feedback.
- Clock progress, raw sensor age and state/TF health are checked by the monitor.
  C-L require map->odom; D-H use /plan, I-L /lunabot/terrain/plan when moving.
Full details and actual verification: safety-faults.md.

P3-02 localization mode loads a saved slam_toolbox pose graph with match_type=3, then publishes /map and map-to-odom; /initialpose supports seed correction. Evaluator truth remains on a separate Gazebo transport capture path and is not bridged into runtime interfaces. See docs/slam-qualification.md.
