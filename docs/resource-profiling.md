# Runtime resource profiling (P11-01)

`tools/profile_runtime.py` records one-second host and selected-stack CPU/RSS
samples, NVIDIA GPU utilization/VRAM when `nvidia-smi` is available, and the
real-time factor from ROS `/clock`. It writes `samples.csv` and `summary.json`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/profile_runtime.py \
  --config config/profiling/resources.json \
  --duration-seconds 300 \
  --interval-seconds 1 \
  --output-dir evidence/runs/<run-id>/P11-01/profile
```

The target laptop is documented in the user's project plan as an HP Victus 15
with a Ryzen 5 5600H (6 cores / 12 threads), 8 GiB host RAM, and an RTX 3050
Laptop GPU with 4 GiB VRAM. The plan allocates 5 GiB RAM, 8 processors, and
8 GiB swap to WSL2. Profile `wsl2-hpvictus15-20261117-v1` in
`config/profiling/resources.json` freezes engineering limits before acceptance
measurements: stack CPU p95 ≤65% of host, RSS p95 ≤3500 MiB, GPU VRAM p95
≤3072 MiB, RTF ≥0.5, and dashboard client receive-to-render p95 ≤100 ms.
These are project limits, not vendor specifications.

The HP Victus target identity is documented in the supplied project plan, but
CIM inventory was denied, so the current host model is not verified. WSL reports
8 logical CPUs and 4920 MiB RAM. The host inventory reports Windows
10.0.26340.0, 12 logical CPUs, a 1536×864 primary display, NVIDIA driver
621.14, and 125% display scaling (registry `AppliedDPI=120`). Native GUI
rendering still needs a measured application viewport at that target size.

## 2026-11-17 profile result

One 300-second diagnostic capture from a mixed existing ROS domain 42 session
recorded stack CPU p95 65.85% (FAIL), stack RSS p95 2482.4 MiB (PASS), GPU
VRAM p95 273 MiB (PASS), RTF 0.16885 (FAIL), and client render p95 unavailable
(INCOMPLETE). It ran with an incomplete Foxglove layout and pre-existing ROS
processes, so it is not an isolated target acceptance result. The raw capture
is in `evidence/runs/20261117-p1103-restart/P11-01/profile/`.

P11-01 remains **IN PROGRESS**. Repeat target-profile trials only after the
full six-view P9-03 workload and exact target runtime are verified. Include a
controlled overload test with recorded safe degradation and recovery; do not
infer overload safety from this profile or the planner-only P10 scenarios.

## Isolated acceptance baseline — 2026-11-17

The profiler now normalizes stack CPU against the frozen 12-thread host count
and accepts `--ros-domain-id` to exclude unrelated ROS stacks. Two five-minute
captures were taken from the live domain46 mission/dashboard workload after
warm-up:

| Trial | Stack CPU p95 | RSS p95 | VRAM p95 | RTF | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| run13-baseline | 46.7% | 2,459 MiB | 273 MiB | 0.174 | FAIL: RTF; render latency incomplete |
| run13-baseline-02 | 46.3% | 2,463 MiB | 273 MiB | 0.165 | FAIL: RTF; render latency incomplete |

Both trials pass the frozen CPU, RSS, and VRAM limits and fail the RTF ≥0.5
limit. The profiler did not receive a client render p95 value, so those samples
remain incomplete as dashboard measurements. Raw CSV and JSON summaries are
under `evidence/runs/20261117-p1103-restart/P11-01/`. No doubled-traffic or
controlled-overload trial has passed. Host CPU utilization was 92.6% and 93.1%
p95 while the measured stack used 46.7% and 46.3% of the 12-thread host,
respectively; unrelated host load is material to this workstation sample.
Windows registry and runtime inventory now identify `Victus by HP Gaming
Laptop 15-fb0xxx`, Ryzen 5 5600H, RTX 3050 Laptop GPU (4096 MiB, driver
621.14), and 7,883,206,656 bytes reported available to Windows. This matches
the documented laptop target family; the measured simulator remains WSL
software-rendered, and the physical rover remains unqualified.

### D3D12 renderer diagnostic — 2026-11-17

Run15 in `evidence/runs/20261117-p1103-restart/P11-01/run15-renderer-d3d12/`
also failed the frozen profile: stack CPU p95 43.81%, RSS p95 2,521.8 MiB,
GPU VRAM p95 273 MiB, and RTF 0.2066. CPU, RSS, and VRAM passed; RTF did not
meet the 0.5 minimum, and dashboard client latency remained unmeasured. This
D3D12 run is diagnostic, not acceptance evidence.

### Run 17 restart workload — 2026-11-17

The five-minute software-rendered restart workload recorded stack CPU p95
46.89%, RSS p95 2,428.4 MiB, GPU VRAM p95 273 MiB, and RTF 0.2555. CPU, RSS,
and VRAM passed; RTF failed the frozen minimum of 0.5. Client render latency
was not collected. Raw samples and summary are under
`evidence/runs/20261117-p1103-restart/P11-01/run17-restart-full-mission/`.

Run18 with the documented 4 m demo goal recorded stack CPU p95 47.03%, RSS
p95 2,418.6 MiB, GPU VRAM p95 273 MiB, and RTF 0.3336. CPU, RSS, and VRAM
passed; RTF remained below 0.5. Client render latency was not collected. Raw
samples and summary are under
`evidence/runs/20261117-p1103-restart/P11-01/run18-goal-distance-fix/`.

### Full-mission restart follow-up — 2026-11-17

Run 14 at `evidence/runs/20261117-p1103-restart/P11-03/run-14/` failed the
safety health gate with `HEALTH_STALE_SCAN`. It also failed to capture a
non-empty retained terrain plan after the planner status reported
`TERRAIN_PLAN_PASS`. The WSL stack was stopped after the failed mission gate.
This run is not an overload profile: no doubled-traffic workload or client
render samples were collected.

### Physics and live-dashboard diagnostics — 2026-11-17

Run21's evidence-only 2 ms world reached RTF 0.5057 with stack CPU p95
47.44%, RSS p95 2,432.5 MiB, and VRAM p95 273 MiB. Dashboard latency was
missing. Its three P3 repeats produced one turn-accuracy failure, so this
physics profile is not qualified and is not the acceptance target. See
`evidence/runs/20261117-p1103-restart/P11-01/run21-physics-2ms-diagnostic/`.

Run22 connected the six-view Foxglove layout during a full Phase L run. The
corrected post-startup five-minute profile measured CPU p95 47.85%, RSS p95
2,438.7 MiB, and VRAM p95 273 MiB. It did not observe `/clock`; RTF and client
latency are therefore incomplete. A best-effort `/clock` listener change
passed an isolated synthetic-clock probe but did not resolve the live-domain
measurement. Run22's mission passed its goal/evaluator gates, then latched
`HEALTH_STAMP_IMU`; the 900-second cap interrupted shutdown. It is diagnostic,
not acceptance evidence. Raw profiles and the run report are under
`evidence/runs/20261117-p1103-restart/P11-03/run-22-live-six-view-profile/`.

No doubled-traffic or overload trial has been collected. Do not promote the
2 ms world or mark P11-01 complete from these diagnostics.
