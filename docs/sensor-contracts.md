# Sensor and TF contracts (P2-01)

Verified live on Phase B in evidence/runs/20261103-p201/P2-01/final/sensors.json.
The A/B/C static gates pass in an isolated copy; all 12 launchers pass Bash syntax.
Full A-L runtime regression is not claimed.

| Stream | Measured simulation Hz | Units/encoding | Actual header frame |
|---|---|---|---|
| RGB | 19.71 | rgb8, 640x480 | lunabot_v4/sensor_head/rgb_camera |
| Depth | 14.48 | 32FC1 metres, 640x480 | lunabot_v4/sensor_head/depth_camera |
| RGB CameraInfo | 20.00 | K fx/fy about 554.38 pixels | same RGB frame |
| Depth CameraInfo | 15.15 | K fx/fy about 554.38 pixels | same depth frame |
| LiDAR | 10.00 | 720 beams, ranges metres, angles radians | lunabot_v4/sensor_head/lidar |
| IMU | 100.00 | SI angular velocity/acceleration | lunabot_v4/imu_link/imu |
| Odometry | 29.41 | metres/radians, velocities SI | odom, child chassis |

Each observed stream has one ROS graph publisher and monotonically advancing
simulation timestamps. Capture-specific QoS and endpoint node names are in the JSON.
The audit uses best-effort sensor subscriptions; TF static uses transient-local
durability. Queue depths are bounded. Rates are simulation-time rates, not wall-clock
performance. The configured 20% rate tolerance and 0.25s simulation freshness bound
are initial diagnostic contracts; safety stopping deadlines are P2-02/P2-03.

## Geometry and ownership
- Gazebo bridge owns sensor data and wheel-derived odom->chassis.
- scripts/sensor_mount_tf.py owns chassis->sensor_head using measured mast pan/tilt
  from JointState at its acquisition timestamp. It publishes nothing if either
  joint is missing or nonfinite; stale-transform handling belongs to safety.
- Existing static publishers own mechanical sensor mounts (RGB/depth pitch 0.25rad,
  LiDAR pitch 0.5rad) plus scoped header aliases. Each static edge has one process.
- Scoped camera header frames are assigned ROS optical axes: z forward, x right,
  y down. They are children of the physical camera mounts with fixed optical
  rotation (roll=-pi/2, yaw=-pi/2). LiDAR and IMU scoped frames use mechanical axes.
- IMU mount is 0.10m above chassis. Sensor-head rest offset is (0.18,0,0.85)m.
  Pan and tilt are measured; no fixed-neutral TF is published for this edge.
- Live mast test: pan about 0.2004rad, tilt about 0.1002rad, quaternion matched
  measured joint angles to the acceptance tolerance.
- Both CameraInfo intrinsics agree with SDF FOV to <1 pixel. This is simulated
  source calibration, not physical-camera calibration. P6-01 consumes these
  intrinsics for calibrated RGB/depth registration; live ROS projection
  tolerance remains to be measured.
- RGB and depth centres differ vertically by 0.04m. Equal image size/FOV does not
  imply registered pixels. Timestamp matching, optical ray projection and
  depth-to-RGB registration are implemented in P6-01 and covered by deterministic
  projection fixtures. Live ROS/Gazebo projection tolerance is not yet measured.

## Reproduction
Start an isolated Phase B headless demo using EVIDENCE_DIR. While it is live:
source /opt/ros/humble/setup.bash
python3 tools/audit_sensor_contracts.py --duration 12 --output <new-sample.json>
python3 tools/check_sensor_contracts.py --sample <sample.json> --output <report.json>
The articulation acceptance additionally expects nonzero mast commands. Exact
timed run orchestration is preserved with this run's evidence.

Known limits: other launchers inherit corrected contract source but have not had
full runtime regression; process-group cleanup fix currently applies to Phase B.
SLAM map->odom ownership remains P3-02 runtime acceptance. IMU zero covariance is
not evidence of perfect accuracy; qualify covariance/estimation in P3.
