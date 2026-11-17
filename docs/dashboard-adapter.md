# Dashboard telemetry adapter (P9-02)

The optional `lunabot_bringup` dashboard adapter subscribes to the exact source
topic set used by views 1–5 in `config/dashboard/contracts.json`. It publishes
one JSON metadata stream on `/lunabot/dashboard/telemetry`; the original ROS
messages remain directly available to Foxglove for images, grids, paths, TF,
plots and status payloads. The adapter has no control-write interface and does
not subscribe to velocity or simulator truth or rewrite source data.

## Start

Build/source the ROS workspace as usual, then run the adapter alongside a
running LunaBot stack:

```bash
colcon build --packages-select lunabot_bringup
source install/setup.bash
ros2 launch lunabot_bringup dashboard_telemetry.launch.py
```

The launch defaults to ROS simulation time. Set `use_sim_time:=false` only when
the source headers and adapter use the same wall ROS clock. The adapter is
separate from A–L launches and is optional for headless operation. Add the
allowlisted source topics plus `/lunabot/dashboard/telemetry` to the selected
Foxglove layout/bridge configuration.

## Metadata behavior

Each source row contains the expected topic/type/view, current publisher node
names, original header stamp and frame when available, source age, monotonic
receive age, configured thresholds, expected frame, freshness state/reason,
and a subscription error when the ROS graph reports an unexpected type. Camera,
depth, perception images, LiDAR, IMU and odometry use the P2-01 source-age bound
of 0.25 simulation seconds plus per-stream receive timeouts. Other streams use
topic-specific receive timeouts in `config/dashboard/adapter.json`. Static TF
and mission event history have explicit latched/event semantics.

For status topics typed as `std_msgs/msg/String`, the adapter extracts a source
stamp only when the JSON payload declares `source_stamp_seconds`, `stamp_sec`,
or `sim_time`; it keeps `created_monotonic` distinct as host monotonic time. If
the payload has no source stamp, the row reports
`stamp_origin=not_provided_by_message` and uses measured receive age. It does
not invent a source stamp or imply that receive time is acquisition time.

The adapter discovers only the explicit topics in `adapter.json`; a graph type
mismatch is reported as unknown and that topic is not subscribed. Its own JSON
output is metadata for freshness/provenance indicators, not a replacement for
the original visual payloads. P9-03 verified a live six-panel Foxglove session
and camera display, and captured app-level performance-sidebar diagnostics.
Camera source-to-render age, per-camera dropped frames, full ROS/Gazebo client
resource coverage and frozen resource budgets remain open.
