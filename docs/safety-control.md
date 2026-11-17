# Command safety contract (P2-02)

## Ownership and configuration
scripts/control_odometry.py is the final operational /cmd_vel publisher for A-L.
Teleop defaults to /cmd_vel_in; Phase A now explicitly routes through this boundary.
B-L retain their controller routing; I-L diagnostic planner output stays isolated.
config/safety/control.json provides defaults, with existing CLI overrides preserved:
0.45m/s linear, 1rad/s angular, acceleration 0.4m/s2 and 0.8rad/s2,
watchdog 0.5s and output 30Hz. Nonpositive/nonfinite settings or identical
input/output topics are rejected at startup. Input/output queues have depth one.
The timer uses a steady clock; receipt freshness uses time.monotonic().

## Interfaces and behavior
- /cmd_vel_in: geometry_msgs/Twist, chassis forward x and yaw z only.
  Nonfinite values in any component or unsupported nonzero components latch
  INVALID_COMMAND and immediately zero controller output. Finite excessive
  forward/yaw commands are clamped; normal commands are acceleration-limited.
- /cmd_vel: geometry_msgs/Twist, final safe request to the bridge.
- /lunabot/safety/emergency_stop: std_srvs/Trigger. Immediately publishes zero,
  clears target/output and latches EMERGENCY_STOP. A continued nonzero request
  cannot release the latch.
- /lunabot/safety/reset: std_srvs/Trigger. Requires a fresh all-zero input
  within the watchdog interval and exactly one output publisher. Clears the
  latch, discards the old command, and requires a new motion request.
- /lunabot/safety/stopped: retained std_msgs/Bool; true when inhibited.
- /lunabot/control/status: std_msgs/String, state/age/limits/counts, at 5Hz.
- Watchdog expiry zeros output immediately, bypassing the normal ramp. A fresh
  valid command may resume after watchdog; emergency/invalid/conflict stops
  require explicit reset.
- An extra output publisher latches OUTPUT_AUTHORITY_CONFLICT. This inhibits
  this controller; ROS graph detection is not an access-control mechanism and
  cannot prevent another process sending directly to the bridge.

Operator commands:
ros2 service call /lunabot/safety/emergency_stop std_srvs/srv/Trigger
ros2 topic pub --once /cmd_vel_in geometry_msgs/msg/Twist '{}'
ros2 service call /lunabot/safety/reset std_srvs/srv/Trigger

## Acceptance and evidence
tests/test_control_safety.py uses real DDS/services, with isolated-domain and
--live modes. Evidence: evidence/runs/20261103-p202/P2-02/.
Isolated tests verify clamps/ramps, latch/reset, invalid input, watchdog, conflict.
Live Phase B test verifies the same usable behaviors and settled wheel odometry.
Measured command-zero delay after emergency request about 0.0018s; watchdog delay
from last publish about 0.513s. Test limits were 0.15s and 0.65s respectively.
These are observations on this environment, not hard realtime guarantees.
Phase A smoke: forward 1.091m and yaw 1.566rad; clean shutdown.
A/B/C static regressions pass in an isolated copy; all launcher Bash syntax passes.

## Subsequent P2-03 refinement
The P2-02 measurements above describe its original acceptance run.
P2-03 now adds a separate raw-data health monitor, startup inhibition and an
independent native actuator watchdog. ROS /cmd_vel is bridged to a guarded request
channel; native /cmd_vel is the actual DiffDrive input. Controller or bridge loss
therefore no longer depends on that failed process publishing a final zero.
A restarted controller requires explicit arming and does not resume held input.

See safety-faults.md and evidence/runs/20261103-p203/P2-03/summary.json.
Unstamped Twist still has receipt-age semantics. Localization accuracy, broad A-L
runtime coverage, security/access control and full mission acceptance remain
later work; wheel odometry observations are not independent world truth.
