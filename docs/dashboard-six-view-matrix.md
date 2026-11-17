# Six-view dashboard build matrix

Source of truth: `config/dashboard/contracts.json` (dashboard `lunabot-six-view-v1`).

**Status: specified, not yet verified in a live dashboard.** The current
Foxglove layout is a smoke layout and the latest reopened session had no active
Rosbridge connection. Use this matrix when assembling the native target layout;
record the actual panel names and screenshot in the P9-03 run evidence.

All six view groups use `/lunabot/dashboard/telemetry` for source stamp,
receive-age, and fresh/stale/unknown metadata. The controls group also shows
`/lunabot/dashboard/control_status` and writes requests only to
`/lunabot/dashboard/command_request`.

| View | Required read topics | Additional setup |
|---|---|---|
| System and mission health | `/lunabot/safety/health`, `/lunabot/safety/stopped`, `/lunabot/control/status`, `/lunabot/mission/status`, `/lunabot/autonomy/status`, `/lunabot/evaluation/status` | Show source stamp and freshness metadata. |
| Sensors and perception | `/lunabot/camera/image_raw`, `/lunabot/depth/image_raw`, `/lunabot/terrain/segmentation`, `/lunabot/terrain/segmentation_confidence`, `/lunabot/terrain/overlay`, `/lunabot/lidar/scan`, `/lunabot/imu` | Show camera and depth images plus perception layers. |
| Map and localization | `/map`, `/tf`, `/tf_static`, `/lunabot/odom`, `/goal_pose`, `/plan` | Use `map` frame; include transform and localization status. |
| Terrain and risk layers | `/lunabot/terrain/semantic_map`, `/lunabot/terrain/semantic_map/confidence`, `/lunabot/terrain/semantic_map/age`, `/lunabot/terrain/semantic_map/slope`, `/lunabot/terrain/semantic_map/roughness`, `/lunabot/terrain/semantic_map/crater_risk`, `/lunabot/terrain/semantic_map/uncertainty`, `/lunabot/terrain/cost_map` | Use `map` frame; expose each layer or provide a layer selector. |
| Planning and evaluation | `/lunabot/terrain/plan`, `/lunabot/terrain/planner/status`, `/lunabot/autonomy/replan_status`, `/lunabot/evaluation/status`, `/lunabot/mission/events` | Use `map` frame for the plan; show current and event status. |
| Mission and safety controls | `/lunabot/mission/status`, `/lunabot/mission/events`, `/lunabot/safety/health`, `/lunabot/safety/stopped`, `/lunabot/control/status`, `/lunabot/dashboard/control_status` | Request-only publish topic: `/lunabot/dashboard/command_request` (`std_msgs/msg/String`). No direct service calls, velocity writes, or simulator-truth access. Show fresh mission and safety state before enabling requests. |

## Live verification checklist

- [ ] Six groups are present and readable at the target native resolution.
- [ ] Every read topic above is actually present in its intended view.
- [ ] Every view shows source stamp, receive age, and fresh/stale/unknown state.
- [ ] Missing and stale publishers display unknown/stale, never fresh by default.
- [ ] Map and plan panels use the `map` frame and align with `/tf`.
- [ ] Controls only publish the guarded request topic and require fresh status.
- [ ] No simulator-truth topic or direct velocity output is subscribed/published.
- [ ] Screenshot and completed topic-to-panel mapping are saved with run evidence.
