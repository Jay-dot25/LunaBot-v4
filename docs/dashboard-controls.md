# Dashboard controls gateway (P9-03)

View 6 writes only JSON requests to `/lunabot/dashboard/command_request` and
reads results from `/lunabot/dashboard/control_status`. Run the guarded gateway
alongside Phase L after building `lunabot_bringup`:

```bash
colcon build --packages-select lunabot_bringup
source install/setup.bash
ros2 launch lunabot_bringup dashboard_controls.launch.py
```

Example requests for a Foxglove Publish panel or future custom control panel:

```json
{"request_id":"op-001","action":"PAUSE"}
{"request_id":"op-002","action":"CANCEL","confirmed":true}
{"request_id":"op-003","action":"ESTOP","confirmed":true}
{"request_id":"op-004","action":"RESET_ESTOP","confirmed":true}
{"request_id":"op-005","action":"ARM_STARTUP"}
{"request_id":"op-006","action":"DELIVER","delivery_id":"sample-01"}
```

The `confirmed` field is the UI confirmation result; a visible confirmation
prompt must precede `CANCEL`, `ESTOP`, and `RESET_ESTOP`. ROS topic access is not
an authorization mechanism, so this guard is an operator-flow check, not an
access-control boundary.

## Action gates

- `PAUSE`, `RESUME`, `CANCEL`, `RECOVER`, and `DELIVER` require mission-status
  receive age at most 1.5 s. Delivery IDs use the mission manager's restricted
  `[A-Za-z0-9._:-]{1,128}` form.
- `ESTOP` requires explicit confirmation but deliberately bypasses freshness
  gates, so stale telemetry cannot disable the emergency-stop action. The
  gateway calls the existing `/lunabot/safety/emergency_stop` service.
- `RESET_ESTOP` requires confirmation, mission status at most 1.5 s old, and a
  schema-1 safety health report at most 0.5 s old by its source
  `created_monotonic` stamp. It calls the existing safety reset service first;
  only a successful controller reset is followed by the mission lifecycle
  `RESET_ESTOP` command. The controller still requires fresh zero input and
  healthy sensor gates.
- `ARM_STARTUP` requires a fresh safety report and delegates startup arming to
  the existing service, which enforces its own zero-input and health checks.

The gateway reports `PENDING`, accepted/rejected outcome, reason, status ages,
and command on `/lunabot/dashboard/control_status`. A published mission command
is only a dispatch acknowledgment; the mission status remains authoritative for
the resulting lifecycle state. No dashboard node publishes velocity.

## Performance gate status

The gateway and telemetry adapter now build and run against a live Phase L ROS
graph. The gateway dispatched PAUSE/RESUME requests, but the mission lifecycle
was already FAILED during the saved probe and rejected those transitions.
Confirmed E-stop/reset reached their services; the recorded request-to-service
acknowledgment times are 96 ms and 10 ms.
P9-03 still requires a loaded six-view ROS/GUI run. Record source-to-render
latency, camera frame rate/drop counts, adapter/client CPU and memory, VRAM,
reconnect behavior, and e-stop UI-request-to-controller acknowledgment under
normal and doubled image and map load. Foxglove Studio is not installed in the
available desktop, and target-machine CPU/RSS/VRAM limits remain for P11-01.
See `evidence/runs/20261114-p903/P9-03/summary.md` and
`evidence/runs/20261114-p903/runtime/phase-l/` for live results and remaining
acceptance.
