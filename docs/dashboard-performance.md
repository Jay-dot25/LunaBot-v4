# Six-view dashboard performance gate

P9-03 requires an actual six-view client session on the target WSL/laptop
profile. A live six-panel Foxglove session is now verified, and app-level
diagnostics plus a visible camera frame at 1536×864 are recorded. A custom
camera probe now has diagnostic runs, but the release gate remains open until
the full contract workload, browser resource, reconnect, doubled-traffic, and
target-budget measurements are captured.
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
was blocked by a computer-use approval timeout; no dashboard changes were made.
