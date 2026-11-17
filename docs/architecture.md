# LunaBot V4 architecture

Gazebo provides sensors and physical interaction. Headless ROS uses RGB/depth, LiDAR, IMU and qualified wheel-derived odometry. SLAM owns map→odom; qualified odometry owns odom→chassis; robot description owns static sensor frames. RGB-D inference and calibrated projection feed semantic/geometry layers fused with occupancy. Preserve occupancy A* baseline and weighted A*. Local avoidance/follower requests motion through a single safety authority, sole /cmd_vel publisher to DiffDrive.

Mission manager owns sequence and durable collection/delivery events. Dashboard observes telemetry and calls mission/safety interfaces. Isolated evaluator receives simulator truth and publishes no operational commands; test access isolation.

Add package metadata/install rules around existing simulation assets and a small bringup package in Phase 1. Add interfaces/package boundaries when justified. Preserve locations and A-L commands; migrate incrementally.

P9-01 compares custom browser + rosbridge_suite, Foxglove Bridge/client, and native Qt/RViz/rqt. ADR-006 selects Foxglove only as the first prototype candidate; P9-03 owns measured latency/resource and release gates. The six view topic/write contract is config/dashboard/contracts.json. Dashboard/RViz remain optional for headless operation. Keep simulator truth isolated, preserve source stamps/age/provenance, and gate controls on fresh mission/safety state.

P9-02 adds an optional read-only ROS metadata adapter for dashboard views 1–5. It subscribes only to the allowlisted topic set, preserves direct source streams, and publishes per-topic source/receive age, frame, freshness, type and publisher provenance on `/lunabot/dashboard/telemetry`. See docs/dashboard-adapter.md; live panel rendering and performance qualification remain P9-03.
## P2-03 refinement
Raw validation now runs in health_monitor.py, separate from control_odometry.py.
The monitor reports timestamped, sequenced health; the controller owns ROS
/cmd_vel, the bridge maps it to Ignition /lunabot/actuator_request, and the native
CommandWatchdog alone forwards to Ignition /cmd_vel for DiffDrive. The guard times
out independently of ROS controller/bridge loss. New controllers start inhibited,
runtime faults stay latched, and startup arming cannot clear them. Report age is
a local-WSL-host monotonic-time contract. See safety-faults.md for the complete
protocol, profile requirements, tests and remaining boundaries.

## P3-01 qualification status

DiffDrive is confirmed to integrate the first configured left/right wheel
positions. Three synchronized trials show the current wheel-derived pose fails
straight and turn accuracy gates; low-friction testing exposes substantial
wheel-ground discrepancy. Keep it as a local prior only. P3-02 must qualify
LiDAR SLAM as GPS-independent global correction. No EKF or canonical model
replacement is introduced from this test. See docs/odometry-qualification.md.

## P3-02 qualified SLAM path

Phase C supports headless mapping and saved-graph localization as separate SLAM_MODE launch modes. slam_toolbox owns map-to-odom; DiffDrive remains the sole odom-to-chassis source. Evaluator-only Gazebo truth is collected outside ROS. The verified WSL profile uses OGRE2/software OpenGL, and saved map YAML preserves PGM unknown space on independent map_server reload. RViz is optional. The short-route localization error is 0.230 m within the 0.25 m qualification bound; wider-route accuracy and full perception resolution remain open.
