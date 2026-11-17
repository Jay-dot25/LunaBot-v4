# ADR-006 — Dashboard prototype choice (P9-01)

**Status:** Prototype Foxglove Desktop/Web plus the Foxglove ROS 2 Bridge first.
This is a candidate for P9-02/03 trials, not production approval.
**Date:** 2026-11-13
**Decision owner:** LunaBot V4 project

## Context

The dashboard needs six operator views over existing ROS 2 Humble topics:
system/mission health, sensors and perception, map/localization, terrain risk,
planning/evaluation, and mission/safety controls. The main map is an occupancy
grid in the local `map` frame; the UI must not consume Gazebo truth or publish
velocity. Sensor stamps, receive age, stale state, frame, and provenance need to
remain visible. Headless A-L operation must stay independent from any GUI.

## Compared stacks

| Stack | ROS and streaming | Maps and six views | Controls | Latency/resources | WSL and licensing | Maintenance |
|---|---|---|---|---|---|---|
| Custom browser UI + `rosbridge_suite` | ROS 2 Humble gateway; JSON WebSocket topics/services. | Full control of six views, but occupancy rendering, TF, image synchronization, timestamp age, and terrain layers are all custom. | Full flexibility; can implement explicit status gates and role-aware controls. | Not measured. JSON conversion and front-end rendering need CPU/RAM/GPU and render-age trials. | ROS gateway in WSL, Windows browser via localhost. `rosbridge_suite` is BSD-3-Clause. | Highest UI implementation cost; greatest layout flexibility. |
| **Foxglove Desktop/Web + Foxglove Bridge** | ROS 2 bridge over Foxglove WebSocket; topic publishing and service calls supported; actions are not supported by the documented live-connection matrix. | Existing 3D panel renders `nav_msgs/OccupancyGrid`, TF, images/depth, and markers; Image and Plot cover camera and temporal data. A separate Map panel is geospatial, so use 3D for this rover's local SLAM map. Most views use standard panels; health freshness and mission controls need a guarded request panel. | Publish JSON requests only to `/lunabot/dashboard/command_request`; display `/lunabot/dashboard/control_status`. A local gateway checks freshness/confirmation and forwards only allowed mission commands or existing safety `Trigger` services. | Not measured. Vendor describes its C++ bridge as high performance/low overhead. No six-view workload numbers exist here. WebGL/WebGPU and WSL GPU path require measurement. | Bridge runs in WSL and the client connects over localhost. Bridge is MIT. Foxglove direct live connections require a Developer seat; verify the current plan before release. | Lowest initial panel effort; product account/seat terms and custom operator interlocks remain. |
| Native Qt/Python + RViz2 + rqt | Direct ROS 2 DDS in the existing WSL Humble runtime; no WebSocket gateway. | RViz2 already has LunaBot configs and native map, TF, image, point-cloud, and marker displays. rqt provides plugins; six integrated views still need a custom Qt panel. | rqt service/topic tools exist; a custom panel can gate commands on health and mission freshness. | Not measured for six simultaneous views. Existing WSL RViz runs prove only that selected GUI profiles work. | WSLg supports Linux GUI applications. RViz2/rqt ROS packages are BSD-licensed. | Medium-high custom UI work; direct ROS integration and no separate bridge. |

The structured candidate records, including comparison fields and benchmark
gaps, are in `config/dashboard/options.json`. These are architecture comparisons,
not measured latency or resource results.

## Decision

Prototype Foxglove Bridge with a local Foxglove client first because its built-in
image, 3D grid, plot, raw-message, and service-call panels cover more of the
six-view requirement with less custom UI work. Keep the bridge on localhost,
restrict topics/services to the dashboard contract, and never expose velocity
or simulator truth interfaces. Use the 3D panel for local `OccupancyGrid`
visualization; do not confuse the geospatial Map panel with the local SLAM map.

If a Developer seat is unavailable, or if freshness/latency/resource tests fail,
the fallback candidate is custom browser + BSD `rosbridge_suite`. Native Qt is
the fallback when WSL networking/rendering proves unreliable or when no gateway
is acceptable. P9-03 retains the final decision gate.

## Six-view data and safety contract

`config/dashboard/contracts.json` defines the six view topic sets and their
write boundary. Every view must show source stamp, receive age, and one of
fresh/stale/unknown. Sensor age uses simulation time; render age uses local
monotonic time. Map, cost, and path data use frame `map` and preserve source
stamps and provenance.

The dashboard client writes only `/lunabot/dashboard/command_request`. The
gateway forwards permitted mission commands to `/lunabot/mission/command` and
calls the three existing `std_srvs/Trigger` safety services. It reports results
on `/lunabot/dashboard/control_status`. The dashboard must never publish
`/cmd_vel`, `/cmd_vel_in`, or the actuator request. Mission controls require
mission status fresher than 1.5 s; reset/arming require source-fresh safety
health within 0.5 s. E-stop remains available during telemetry loss. Confirm
before `CANCEL`, `ESTOP`, or `RESET_ESTOP`. These are UI guards; ROS safety
authority continues to enforce stop/reset rules.

## Measurement plan and limits

No browser/native dashboard, bridge, ROS node, or six-panel session was launched
for P9-01, so there are no measured UI latency, CPU, RSS, VRAM, or dropped-frame
results yet. Do not reuse P5-03 model inference or P7-03 planner timings as
dashboard measurements. P9-02/03 will measure:

- source-stamp to render age p50/p95 for camera, map, plan, and status;
- camera render FPS and stale/dropped frame counts under the actual six-view set;
- total CPU, RSS, and VRAM with normal and doubled image/map load;
- emergency-stop button to controller safe-stop acknowledgment, separate from
  sensor-to-actuator stop timing;
- WSL localhost connection, hardware acceleration, and clean disconnect/reconnect.

The UI remains optional for headless acceptance.

## Primary references (checked 2026-11-13)

- Foxglove ROS 2 live setup and Bridge recommendation:
  [ROS 2 connection guide](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2),
  [Bridge overview](https://docs.foxglove.dev/docs/fleet/bridge).
- Live feature matrix and Developer seat requirement:
  [Live connections](https://docs.foxglove.dev/docs/visualization/connecting/live),
  [seat types](https://docs.foxglove.dev/docs/security/seat-types).
- Foxglove local-grid/image/service features:
  [3D panel, including ROS OccupancyGrid](https://docs.foxglove.dev/docs/visualization/panels/3d),
  [Image panel](https://docs.foxglove.dev/docs/visualization/panels/image),
  [Service Call panel](https://docs.foxglove.dev/docs/visualization/panels/service-call).
- Bridge license:
  [ROS 2 bridge source and license](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md).
- ROS bridge and native package licenses/support:
  [rosbridge_suite on ROS Index](https://index.ros.org/p/rosbridge_suite/),
  [RViz2 on ROS Index](https://index.ros.org/p/rviz2/),
  [rqt_gui on ROS Index](https://index.ros.org/p/rqt_gui/).
- WSL graphical applications:
  [Microsoft WSL GUI guide](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

P9-03 refines view 6 with a guarded command gateway; see
`docs/dashboard-controls.md`. No six-view runtime performance result is claimed.
