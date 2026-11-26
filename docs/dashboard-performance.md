# Six-view dashboard performance gate

## Acceptance continuation (2026-11-17)

The local Foxglove extension `LunaBot P9-03 Dashboard Tools` version 0.2.0
was installed and is enabled in the signed-in client. Its picker exposes six
read-only topic-health panel types, one per contract view group. The extension
build and lint passed. A panel was rendered in the `Coffee Dragonfly` draft;
it showed the expected sensor contract rows, all `unknown · no metadata`.
The draft currently has no data source, is marked unsaved, and does not yet
place and verify all six contract panels in their matching tabs. This proves
the extension loads, not that the contract is live or accepted. The extension
is a local install saved on this device, not a hosted organization install.

The six view groups and adapter mapping were statically audited: all 41 topics
in the six view groups have adapter entries, with no contract read-only topic
missing a row. Static mapping does not substitute for live panel verification.
The six-panel layout still needs a live connection, source/status validation,
reconnect recovery, baseline and doubled-traffic trials, and target-size
measurements.

The target profile is now frozen in
`config/profiling/resources.json` as
`wsl2-hpvictus15-20261117-v1`: stack CPU p95 ≤65% of host, stack RSS p95
≤3500 MiB, GPU VRAM p95 ≤3072 MiB, RTF ≥0.5, and client receive-to-render
p95 ≤100 ms. These are project engineering limits, not manufacturer
specifications. WSL reports 8 logical CPUs and 4920 MiB RAM; Windows display
scaling is 125% at 1536×864. CIM access was denied, so the current host model is
unverified, and exact application-viewport rendering still needs evidence.

A five-minute profile captured during an earlier mixed ROS domain 42 session
with an incomplete browser layout is diagnostic only: stack CPU p95 65.85%
(FAIL), RSS p95 2482.4 MiB (PASS), GPU VRAM p95 273 MiB (PASS), RTF 0.16885
(FAIL), and client render p95 unavailable (INCOMPLETE). It is not isolated
target acceptance and does not establish safe overload behavior. The isolated
domain 43 restart regression reached the goal but failed dynamic terrain-plan
replanning; it is not a valid dashboard acceptance workload. No overload
injection acceptance trial has passed.

P9-03 remains **IN PROGRESS**. Do not use the planner-only P10 scenarios as
live mission evidence, claim physical hardware qualification, or treat the
missing separate Goal Master Plan as resolved.

P9-03 requires an actual six-view client session on the target WSL/laptop
profile. A prior six-panel smoke session and app-level diagnostics plus a
visible camera frame at 1536×864 are recorded. A custom camera probe now has
diagnostic runs, but the release gate remains open until the full contract
workload, browser resource, reconnect, doubled-traffic, and target-budget
measurements are captured.
P5 inference timing and planner timing are not dashboard render measurements.

## Workload

Run the Phase L stack with the P9-02 telemetry adapter and P9-03 control gateway.
Open all six views from `config/dashboard/contracts.json`, including the local
occupancy grid, camera/depth, terrain layers, plan/status, freshness metadata,
and the guarded control panel. Warm up for 60 seconds, then collect three
five-minute trials. Repeat with camera/depth and map traffic doubled where the
source profile permits. Do not connect simulator-truth topics.

## Record per trial

- source-stamp to rendered-frame age p50/p95 for image, map, plan and status;
- camera render FPS and stale/dropped frame counts;
- ROS bridge, adapter, dashboard client and simulator CPU and RSS;
- dashboard-client GPU/VRAM use and WSL graphics mode;
- request to controller acknowledgment for confirmed e-stop, while retaining
  the existing controller/native-watchdog safety measurements separately;
- connection startup, disconnect, and reconnect outcome.

The render timestamp must come from the client at actual display/update time;
adapter receive age alone does not measure render latency. Capture the source
message stamp and both client receive/render monotonic times. Report unavailable
measurements explicitly. Freeze numeric CPU/RSS/VRAM limits with the target
machine before treating those values as a release gate; this repo does not
identify that machine or establish its limits yet.

`foxglove/p903-render-probe/` contains a custom camera panel and a `.foxe`
package for collecting camera-panel receive-to-paint samples. Version 0.1.1
has been installed locally in Foxglove and completed three diagnostic runs.
The 480×667 CSS-pixel run measured 4.10 painted FPS, receive-to-paint p50/p95
of 46.6/66.0 ms, and estimated source-to-paint wall age p50/p95 of
733.6/1,525.6 ms. Two temporary 1536×864 CSS viewport runs measured painted
FPS of 4.30 and 4.16; receive-to-paint p50/p95 of 42.9/58.3 ms and
45.1/61.9 ms; and estimated source-to-paint wall age p50/p95 of 720.1/1,513.0
ms and 713.4/1,378.3 ms. The source was running near 0.02× real time. Its
source-stamp-to-wall-age metric is estimated from `/clock` and observed
simulation rate; its source-gap count is cadence-derived. Neither is a
transport-level drop counter or a measurement of Foxglove's built-in Image
panel. These values are diagnostic and do not pass the target-session gate.
The 1536×864 values are in-app browser viewport overrides, not native desktop
measurements. Raw data and limitations are recorded in
`evidence/runs/20261114-p903/P9-03/followup/summary.md`.

## Current result

The ROS workspace builds, the telemetry adapter publishes a live snapshot,
and the guarded gateway has been exercised against the Phase L ROS graph.
The gateway dispatched PAUSE and RESUME, though the mission lifecycle was
already FAILED during the saved probe and did not accept those transitions.
Confirmed ESTOP and RESET_ESTOP reached their safety services; the recorded
request-to-service acknowledgment times were 96 ms and 10 ms. The underlying
Phase L demo did not pass terrain-plan acceptance: its cost map waited for
synchronized slope data and its planner reported an occupancy constraint.

A follow-up was performed in the hosted Foxglove web client after the user
explicitly authorized sign-in, organization setup, and the local ROS telemetry
connection. The personal `Cyan Porpoise` layout was reopened with six panels:
camera/perception image, mission status, planner status, two 3D views, and a
guarded request-only control panel. This proves local connectivity and camera
display, but is a smoke layout rather than the complete six view contract: it
does not yet show all required health, depth, freshness/metadata, terrain-layer,
and planning/evaluation topics as specified in
`config/dashboard/contracts.json`. Restarting the headless ROS graph and local
Rosbridge restored live subscriptions; the Image panel visibly rendered the
camera feed. The source topic averaged 4.043 Hz wall rate in a separate local
probe while the simulation ran below real time. The control panel remained
configured for `/lunabot/dashboard/command_request`; its blank request was not
published.

In the 2026-11-14 continuation, the authorized WSL path restored the ROS graph
and Rosbridge connection. The Phase L graph, telemetry adapter, and guarded
gateway were live. A fresh comparison found all 36 configured read-only
contract topics in the graph and no mismatches for the 27 adapter-typed topics;
the inventory is `evidence/runs/20261114-p903/P9-03/followup/live-ros-topic-inventory.csv`.
Telemetry explicitly reported no-sample sources as `unknown` and expired
streams as `stale`. The unsaved Foxglove draft now has three exploratory tabs,
but the added depth image source and the six-view contract coverage remain
unverified. This reconnect and inventory do not constitute acceptance data.

Four 300-second resource trials collected 292, 286, 298, and 295 one-second
samples. Host CPU p95 was 87.9–89.4%, host memory p95 was 2,112–2,121 MiB,
stack CPU p95 was 58.7–59.1% of host, stack RSS p95 was 1,327–1,329 MiB, and
RTF ranged from 0.213 to 0.257. NVIDIA GPU use reported zero under the
software-rendered WSL profile. Foxglove's tab memory indicator from the earlier
session ranged from 458 to 592 MB. Trial 04 ran with the reconnected six-panel
Foxglove session open. A Windows process probe found multiple browser processes
and could not identify the Foxglove renderer, so those CPU/RSS values were
discarded. The narrow in-app viewport compressed the image canvas, but a
separate maximized Chrome view at 1536×864 did display live camera frames. The
actual release target resolution remains unspecified. These samples do not
include browser CPU, VRAM, or a camera-specific rendered-frame counter.

P9-03 remains **IN PROGRESS**. The custom probe now measures camera-panel
receive-to-paint timing, painted FPS, and estimated source-to-paint wall age in
three diagnostic runs. These were not a native target session, and the
connected dashboard still lacks the full six-view contract workload. Client
CPU/VRAM, reconnect timing, doubled sensor/map traffic, and frozen release
limits remain open. App-level render FPS is reported below. The resource runs
are diagnostic and report `NOT_EVALUATED` because the profile limits are still
null. Follow-up records:
`evidence/runs/20261114-p903/P9-03/foxglove-followup.md` and
`evidence/runs/20261114-p903/P9-03/followup/summary.md`.

## 2026-11-17 live layout check

The saved `Coffee Dragonfly` layout is connected to `ws://localhost:9091` and
shows all six contract view names. The custom panels render live topic health,
source/receive age, and per-topic receive-to-render p95. The mission/safety
view showed fresh mission, safety, and control status, with p95 values varying
from about 83 to 202 ms during observation; some observed rows exceed the
100 ms limit. `/lunabot/mission/events` and
`/lunabot/dashboard/control_status` remained `unknown · no_sample_received`.
That view is not the image/map/plan rendering surface, so its timings do not
complete the source-to-render contract for those message types. The six-view
static contract covers 41 topics, but full-workload reconnect and native image,
map, and plan rendering still need capture.

Two isolated five-minute baseline captures ran with the layout source connected
to domain46. They are recorded under
`evidence/runs/20261117-p1103-restart/P11-01/`; both fail the frozen RTF limit
and omit a client render p95 value. No valid doubled-traffic or target-size
render trial is recorded. An idle-adapter reconnect check has now been
observed, but it is incomplete for target acceptance. P9-03 remains
**IN PROGRESS**.

The reconnect check stopped and restarted only the domain46 Rosbridge server
on port 9091. Foxglove reported 0 panel renders and unknown metadata while it
was offline. On restart, Rosbridge logged the client connection 2.94 seconds
after server startup and restored the custom panel subscriptions. The
mission/safety panel returned to fresh `/lunabot/safety/health`, with receive
age 22 ms and receive-to-render p95 33 ms. The simulator was not running, so
mission/control rows were stale; mission events and dashboard control status
remained unknown. This proves adapter-only reconnect behavior, not the full
workload or e-stop acknowledgment. Evidence:
`evidence/runs/20261117-p1103-restart/P9-03/reconnect-01/README.md`.

### 2026-11-17 restart follow-up

Run 14 used the saved six-view layout while the domain46 headless stack was
active. It did not produce image/map/plan render samples. Mission acceptance
failed on `HEALTH_STALE_SCAN`; the launch harness
also failed to retrieve a non-empty retained terrain plan after planner status
reported `TERRAIN_PLAN_PASS`. Evidence is in
`evidence/runs/20261117-p1103-restart/P11-03/run-14/README.md`. No doubled
traffic was injected, and P9-03 remains **IN PROGRESS**.

### Foxglove client diagnostics

A separate five-minute capture recorded 60 five-second averages from the live
Foxglove Performance sidebar with the six-panel layout connected. Main-thread
render FPS was 30 p50 / 37 p95 (range 18–42); data-pipeline frame rate was 17.8
p50 / 18.9 p95 (range 11.3–19.3); event-loop latency was 66.9 ms p50 / 85.6 ms
p95 (range 55.4–94.6 ms); JS heap used was 254.2 MB p50 / 314.5 MB p95 (range
198.2–327.5 MB). The in-app browser viewport was 480 pixels wide, so these
diagnostics are not a target-size benchmark. A separate maximized Chrome view
at 1536×864 displayed a live camera frame; the actual target resolution remains
unspecified. Main-thread FPS does not provide camera-specific FPS or
dropped/stale frame counts, and the live-source data-loading statistic was
unavailable. Raw data:
`evidence/runs/20261114-p903/P9-03/followup/ui-performance-samples.csv`.

### 2026-11-16 Phase L workload update

The headless workload prerequisite for dashboard acceptance now passes with
the active P5-03 three-class model. The accepted Phase L run processed 365
frames and exercised semantic/cost mapping, planning, dynamic replanning,
mission completion, map save, and clean shutdown. Evidence and the full run
summary are in
`evidence/runs/20261116-threeclass-phase-l-retry2/README.md`. This supersedes
older statements that Phase L mapping/planner synchronization had not passed.

This does not close P9-03: the complete six-view native dashboard, target
resolution, client render/resource instrumentation, reconnect, doubled-traffic
trials, and approved limits remain unverified. The desktop inspection attempt
was blocked by a computer-use approval timeout. The later 2026-11-17 execution
update above supersedes that earlier state: the extension is installed and the
six-view hosted layout is saved, though the acceptance workload is incomplete.

Run15 tried the D3D12 renderer and still did not meet target acceptance. Its
five-minute RTF was 0.2066 against the frozen 0.5 minimum. The mission failed
with `HEALTH_CLOCK_STALLED` and `NONPOSITIVE_DIMENSIONS`; the startup topic
probe also timed out on the first depth sample even though the semantic mapper
later consumed valid depth frames. The image/depth startup probes now retry a
header-only sample for up to 90 seconds to account for delayed rendering and
discovery. See
`evidence/runs/20261117-p1103-restart/P11-03/run-15-renderer-d3d12/README.md`.
The restart and P9-03 workload acceptance gates remain open.

Run16 confirmed the empty SLAM map startup fix, but its live `/clock`, odometry,
and control-status samples failed. Run17 then passed all required startup
topics and dynamic replanning, but did not reach the goal in its 900-second
wait. The concurrent five-minute RTF was 0.2555, below 0.5; dashboard render
latency was not captured. Both run reports are linked from
`docs/p9-03-completion-plan.md` and the P11-03 evidence directory.

### Run 21 and run 22 — live dashboard follow-up

Run21 used an evidence-only world copy with a 2 ms physics step. Its five-minute
profile passed CPU/RSS/VRAM and measured RTF 0.5057, but client latency was
missing. The full simulated mission, evaluator, map save, and clean shutdown
passed. Three fresh P3 trials on that same world yielded two passes and one
turn-accuracy failure at 0.2912 rad (limit 0.10 rad), so the 2 ms profile is
not qualified and the canonical 1 ms world remains unchanged. See
`evidence/runs/20261117-p1103-restart/P11-01/run21-physics-2ms-diagnostic/README.md`
and the companion P11-03 report.

Run22 connected the saved six-view Foxglove layout during a live Phase L run.
Startup, planning, dynamic replanning, goal completion, and evaluator passed;
after that, the safety monitor latched `HEALTH_STAMP_IMU` and the launcher hit
its 900-second cap before clean shutdown. CPU/RSS/VRAM passed in the post-startup
five-minute sample. The profiler could not see `/clock`, so RTF is incomplete
even after its listener was switched to best-effort QoS. Foxglove showed
fresh/stale/unknown rows, while sensor image receive-to-render p95 remained
`n/a`; target display size was not verified. Run22 is diagnostic, not a valid
P9/P11 acceptance trial. No baseline/doubled-traffic set or overload result has
been collected. See
`evidence/runs/20261117-p1103-restart/P11-03/run-22-live-six-view-profile/README.md`.
