#!/usr/bin/env python3
"""Collect host/stack CPU, RSS, GPU and ROS simulation real-time-factor samples."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path

import psutil


ROOT = Path(__file__).resolve().parents[1]


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(quantile * len(ordered)) - 1))
    return ordered[index]


def describe(values: list[float]) -> dict[str, float | None]:
    return {
        "min": min(values) if values else None,
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def gpu_snapshot() -> tuple[float | None, float | None]:
    if shutil.which("nvidia-smi") is None:
        return None, None
    try:
        output = subprocess.check_output([
            "nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
            "--format=csv,noheader,nounits",
        ], text=True, stderr=subprocess.DEVNULL, timeout=3)
        utilization, memory = output.strip().splitlines()[0].split(",")
        return float(utilization.strip()), float(memory.strip())
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None, None


def stack_processes(patterns: list[re.Pattern[str]],
                    ros_domain_id: int | None = None) -> list[psutil.Process]:
    found = []
    for process in psutil.process_iter(["cmdline"]):
        try:
            command = " ".join(process.info.get("cmdline") or [])
            if ros_domain_id is not None:
                process_domain = process.environ().get("ROS_DOMAIN_ID")
                if process_domain != str(ros_domain_id):
                    continue
            if command and any(pattern.search(command) for pattern in patterns):
                found.append(process)
                process.cpu_percent(None)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess,
                OSError):
            continue
    return found


def sample_stack(processes: list[psutil.Process], host_cpu_count: int) -> tuple[float, float]:
    total_cpu = 0.0
    total_rss = 0
    for process in processes:
        try:
            total_cpu += process.cpu_percent(None)
            total_rss += process.memory_info().rss
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return total_cpu / max(1, host_cpu_count), total_rss / (1024 * 1024)


def load_clock_listener():
    try:
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError:
        return None

    class ClockListener(Node):
        def __init__(self):
            super().__init__("lunabot_runtime_profiler")
            self.first_wall = None
            self.last_wall = None
            self.first_sim = None
            self.last_sim = None
            # Use best-effort to match the simulator's high-rate clock stream
            # across ROS distributions and bridge QoS overrides.
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)

        def on_clock(self, message):
            wall_now = time.monotonic()
            sim_now = message.clock.sec + message.clock.nanosec / 1_000_000_000
            if self.first_wall is None:
                self.first_wall = wall_now
                self.first_sim = sim_now
            self.last_wall = wall_now
            self.last_sim = sim_now

        def rtf(self):
            if (self.first_wall is None or self.last_wall is None or
                    self.first_sim is None or self.last_sim is None):
                return None
            wall_delta = self.last_wall - self.first_wall
            return (self.last_sim - self.first_sim) / wall_delta if wall_delta > 0 else None

    rclpy.init()
    return rclpy, ClockListener()


def run(config_path: Path, output_dir: Path, duration: float,
        interval: float, client_render_p95_ms: float | None = None,
        ros_domain_id: int | None = None) -> dict:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    output_dir.mkdir(parents=True, exist_ok=True)
    patterns = [re.compile(pattern) for pattern in config["stack_process_patterns"]]
    host_cpu_count = int(config.get("environment", {}).get(
        "host_logical_cpu_count") or psutil.cpu_count() or 1)
    clock = load_clock_listener()
    rclpy = clock[0] if clock else None
    clock_node = clock[1] if clock else None
    executor = None
    if clock_node is not None:
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(clock_node)

    fields = ["wall_monotonic_seconds", "host_cpu_percent", "host_memory_used_mib",
              "stack_cpu_percent_of_host", "stack_rss_mib", "gpu_utilization_percent",
              "gpu_memory_used_mib"]
    rows = []
    processes = stack_processes(patterns, ros_domain_id)
    started = time.monotonic()
    next_sample = started
    try:
        while time.monotonic() - started < duration:
            if executor is not None:
                executor.spin_once(timeout_sec=0.05)
            else:
                time.sleep(0.05)
            now = time.monotonic()
            if now < next_sample:
                continue
            stack_cpu, stack_rss = sample_stack(processes, host_cpu_count)
            gpu_util, gpu_memory = gpu_snapshot()
            virtual = psutil.virtual_memory()
            rows.append({
                "wall_monotonic_seconds": now,
                "host_cpu_percent": psutil.cpu_percent(None),
                "host_memory_used_mib": virtual.used / (1024 * 1024),
                "stack_cpu_percent_of_host": stack_cpu,
                "stack_rss_mib": stack_rss,
                "gpu_utilization_percent": gpu_util,
                "gpu_memory_used_mib": gpu_memory,
            })
            if len(rows) % 10 == 0:
                processes = stack_processes(patterns, ros_domain_id)
            next_sample = now + interval
    finally:
        if clock_node is not None:
            clock_node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()

    csv_path = output_dir / "samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    def series(name):
        return [float(row[name]) for row in rows if row[name] is not None]

    rtf = clock_node.rtf() if clock_node is not None else None
    client_latency = client_render_p95_ms
    measured = {
        "stack_cpu_percent_of_host": percentile(series("stack_cpu_percent_of_host"), 0.95),
        "stack_rss_mib": percentile(series("stack_rss_mib"), 0.95),
        "gpu_vram_mib": percentile(series("gpu_memory_used_mib"), 0.95),
        "rtf_min": rtf,
        "latency_p95_ms": client_latency,
    }
    checks = {}
    limits = config["limits"]
    for key, value in measured.items():
        limit = limits.get(key)
        if value is None or limit is None:
            checks[key] = "INCOMPLETE"
        elif key == "rtf_min":
            checks[key] = "PASS" if value >= limit else "FAIL"
        else:
            checks[key] = "PASS" if value <= limit else "FAIL"
    if any(result == "FAIL" for result in checks.values()):
        gate_state = "FAIL"
    elif any(result == "INCOMPLETE" for result in checks.values()):
        gate_state = "INCOMPLETE"
    else:
        gate_state = "PASS"
    summary = {
        "schema_version": 1,
        "config_profile_id": config["profile_id"],
        "ros_domain_id": ros_domain_id,
        "cpu_normalization_host_logical_cpu_count": host_cpu_count,
        "duration_seconds_requested": duration,
        "sample_interval_seconds": interval,
        "sample_count": len(rows),
        "ros_clock_observed": clock_node is not None and clock_node.last_sim is not None,
        "real_time_factor": rtf,
        "metrics": {
            "host_cpu_percent": describe(series("host_cpu_percent")),
            "host_memory_used_mib": describe(series("host_memory_used_mib")),
            "stack_cpu_percent_of_host": describe(series("stack_cpu_percent_of_host")),
            "stack_rss_mib": describe(series("stack_rss_mib")),
            "gpu_utilization_percent": describe(series("gpu_utilization_percent")),
            "gpu_memory_used_mib": describe(series("gpu_memory_used_mib")),
            "latency_p95_ms": client_latency,
        },
        "measured_gate_values": measured,
        "gate_checks": checks,
        "gate_state": gate_state,
        "limits": config["limits"],
        "samples_csv": str(csv_path),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "config/profiling/resources.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-seconds", type=float)
    parser.add_argument("--interval-seconds", type=float)
    parser.add_argument("--client-render-p95-ms", type=float,
                        help="p95 client receive-to-render latency captured by the dashboard probe")
    parser.add_argument("--ros-domain-id", type=int,
                        help="include only stack processes from this ROS_DOMAIN_ID")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    duration = args.duration_seconds or float(config["sampling"]["default_duration_seconds"])
    interval = args.interval_seconds or float(config["sampling"]["interval_seconds"])
    if duration <= 0 or interval <= 0:
        parser.error("duration and interval must be positive")
    if args.client_render_p95_ms is not None and args.client_render_p95_ms < 0:
        parser.error("client render latency must be non-negative")
    summary = run(args.config, args.output_dir, duration, interval,
                  args.client_render_p95_ms, args.ros_domain_id)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
