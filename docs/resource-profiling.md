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
8 GiB swap to WSL2. The `wsl2-jay-desktop-20261114` record remains a provisional
runtime baseline: 8 logical CPUs, 4,920 MiB available memory, and the same GPU.
A host inventory recorded Windows version 10.0.26340.0, 12 host logical CPUs,
a 1536x864 primary display, and NVIDIA driver 621.14. Windows CIM and WSL
launch are currently denied, so runtime memory and graphics settings have not
been revalidated against the laptop plan. Display scaling is still unknown.
CPU, RSS, VRAM, RTF, and latency limits remain null pending approved acceptance
limits and the P9-03 GUI benchmark. The profiler reports `NOT_EVALUATED` while
any limit is unset.

The P11-01 headless soak is only a partial environment baseline. It does not
measure dashboard render latency or overload safety, and it must not be used as
release acceptance. Run the target six-view workload first, freeze limits with
headroom, then repeat the full soak and safe-overload checks against them.
