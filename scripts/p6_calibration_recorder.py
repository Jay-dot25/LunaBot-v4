#!/usr/bin/env python3
"""Record live RGB-D point residuals against the exact seeded Gazebo mesh.

The recorder is evaluation-only. It subscribes to production sensor/TF topics
and publishes no operational ROS topics or commands.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from semantic_projection import CameraIntrinsics, RigidTransform, decode_depth_buffer  # noqa: E402
from terrain_geometry import derive_terrain_layers  # noqa: E402


def load_terrain_generator():
    path = ROOT / "tools" / "generate_lunar_terrain.py"
    spec = importlib.util.spec_from_file_location("p6_terrain_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import terrain generator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class P6CalibrationRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("p6_calibration_recorder")
        self.args = args
        self.output = Path(args.output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.gates = json.loads((ROOT / args.gates).read_text(encoding="utf-8"))
        terrain = load_terrain_generator()
        self.x_grid, self.y_grid, self.z_grid, *_ = terrain.generate_terrain(
            self.gates["reference"]["terrain_seed"])
        self.first_xy = -terrain.EXTENT / 2 + terrain.EXTENT / terrain.N_FINE / 2
        self.mesh_step = terrain.EXTENT / terrain.N_FINE
        self.mask: Image | None = None
        self.camera_info: CameraInfo | None = None
        self.depth_history: deque[Image] = deque(maxlen=30)
        self.last_processed_mask_stamp_ns = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, args.mask_topic, self._mask_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic,
                                 self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self._depth_cb,
                                 qos_profile_sensor_data)
        self.layers: dict[str, OccupancyGrid] = {}
        for layer in ("slope", "roughness", "crater_risk", "uncertainty"):
            self.create_subscription(
                OccupancyGrid, f"/lunabot/terrain/semantic_map/{layer}",
                lambda msg, layer_name=layer: self.layers.__setitem__(layer_name, msg), 3)
        self.csv_path = self.output / "projection_points.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            "run_id", "stamp_ns", "u", "v", "depth_m", "range_bin",
            "x_map", "y_map", "z_map", "x_truth", "y_truth", "z_truth",
            "horizontal_error_m", "vertical_error_m", "error_3d_m"])
        self.writer.writeheader()
        self.points: list[dict[str, float | str]] = []
        self.camera_calibration: dict[str, object] = {}
        self.started_wall = time.monotonic()
        self.done = False
        self.timer = self.create_timer(0.25, self._finish_if_due)
        self.get_logger().info(
            f"P6 reference recorder seed={self.gates['reference']['terrain_seed']} "
            f"duration={args.duration_s}s run={args.run_id}")

    def _mask_cb(self, msg: Image) -> None:
        self.mask = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    @staticmethod
    def _stamp_ns(msg: Image | CameraInfo) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _transform(msg) -> RigidTransform:
        t = msg.transform.translation
        q = msg.transform.rotation
        return RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _mesh_height(self, x: float, y: float) -> float | None:
        ix = (x - self.first_xy) / self.mesh_step
        iy = (y - self.first_xy) / self.mesh_step
        col, row = math.floor(ix), math.floor(iy)
        if col < 0 or row < 0 or col >= self.z_grid.shape[1] - 1 or row >= self.z_grid.shape[0] - 1:
            return None
        fx, fy = ix - col, iy - row
        a = float(self.z_grid[row, col])
        b = float(self.z_grid[row, col + 1])
        d = float(self.z_grid[row + 1, col])
        e = float(self.z_grid[row + 1, col + 1])
        # Match tools/generate_lunar_terrain.py's emitted OBJ diagonal (a-e).
        if fy <= fx:
            return a * (1.0 - fx) + b * (fx - fy) + e * fy
        return a * (1.0 - fy) + e * fx + d * (fy - fx)

    def _ray_truth(self, camera_to_map: RigidTransform, xnorm: float,
                   ynorm: float) -> tuple[float, float, float] | None:
        direction = np.asarray([xnorm, ynorm, 1.0], dtype=np.float64)
        world_rotation = self._world_from_target_rotation()
        origin = self._target_point_to_world(np.asarray(camera_to_map.translation))
        direction = world_rotation @ camera_to_map.rotation_matrix() @ direction

        if self.args.reference_surface == "plane":
            if abs(float(direction[2])) < 1e-9:
                return None
            distance = (self.args.plane_world_z_m - float(origin[2])) / float(direction[2])
            if distance <= 0.0 or distance > self.args.max_reference_depth_m:
                return None
            point = origin + direction * distance
            return float(point[0]), float(point[1]), float(point[2])

        def residual(depth: float) -> tuple[float, np.ndarray] | None:
            p = origin + direction * depth
            h = self._mesh_height(float(p[0]), float(p[1]))
            if h is None:
                return None
            return float(p[2] - h), p

        previous_depth = 0.1
        previous = residual(previous_depth)
        if previous is None:
            return None
        for depth in np.linspace(0.2, self.args.max_reference_depth_m, 80):
            current = residual(float(depth))
            if current is None:
                previous_depth, previous = float(depth), current
                continue
            if previous is not None and previous[0] > 0.0 >= current[0]:
                lo, hi = previous_depth, float(depth)
                for _ in range(24):
                    mid = (lo + hi) / 2.0
                    test = residual(mid)
                    if test is None:
                        break
                    if test[0] > 0:
                        lo = mid
                    else:
                        hi = mid
                hit = residual((lo + hi) / 2.0)
                if hit is None:
                    return None
                p = hit[1].copy()
                return float(p[0]), float(p[1]), float(p[2])
            previous_depth, previous = float(depth), current
        return None

    def _world_from_target_rotation(self) -> np.ndarray:
        roll = self.args.truth_world_roll_offset_rad
        pitch = self.args.truth_world_pitch_offset_rad
        yaw = self.args.truth_world_yaw_offset_rad
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.asarray([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=np.float64)

    def _target_point_to_world(self, point: np.ndarray) -> np.ndarray:
        translation = np.asarray([self.args.truth_world_x_offset_m,
                                  self.args.truth_world_y_offset_m,
                                  self.args.truth_world_z_offset_m], dtype=np.float64)
        return self._world_from_target_rotation() @ point + translation

    @staticmethod
    def _range_bin(depth: float, ranges: list[list[float]]) -> str | None:
        for index, (lower, upper) in enumerate(ranges):
            if lower <= depth < upper:
                return f"bin_{index + 1}"
        return None

    def _depth_cb(self, depth: Image) -> None:
        self.depth_history.append(depth)
        mask = self.mask
        info = self.camera_info
        if mask is None or info is None:
            return
        mask_stamp_ns = self._stamp_ns(mask)
        if mask_stamp_ns <= self.last_processed_mask_stamp_ns:
            return
        depth = min(self.depth_history,
                    key=lambda frame: abs(mask_stamp_ns - self._stamp_ns(frame)))
        if abs(mask_stamp_ns - self._stamp_ns(depth)) > 75_000_000:
            return
        if mask.encoding.lower() not in ("mono8", "8uc1") or info.width != depth.width or info.height != depth.height:
            return
        try:
            intrinsics = CameraIntrinsics.from_k(depth.width, depth.height, list(info.k))
            depth_m = decode_depth_buffer(depth.data, depth.width, depth.height,
                                          depth.step, depth.encoding, depth.is_bigendian)
            labels = np.ndarray((mask.height, mask.width), dtype=np.uint8,
                                buffer=mask.data, strides=(mask.step, 1))
            transform = self.tf_buffer.lookup_transform(
                self.args.target_frame, depth.header.frame_id,
                Time.from_msg(depth.header.stamp), timeout=Duration(seconds=0.05))
            camera_to_map = self._transform(transform)
        except Exception as exc:
            self.get_logger().debug(f"skip frame: {exc}")
            return

        self.last_processed_mask_stamp_ns = mask_stamp_ns
        self.camera_calibration = {
            "target_frame": self.args.target_frame,
            "depth_frame_id": depth.header.frame_id,
            "width_px": depth.width, "height_px": depth.height,
            "intrinsics_k": [float(value) for value in info.k],
            "transform_translation": list(camera_to_map.translation),
            "transform_rotation_xyzw": list(camera_to_map.rotation_xyzw),
            "depth_stamp_ns": self._stamp_ns(depth),
            "mask_stamp_ns": mask_stamp_ns,
            "mask_depth_skew_ms": abs(mask_stamp_ns - self._stamp_ns(depth)) / 1e6,
        }

        gates = self.gates["projection"]
        ranges = gates["depth_range_bins_m"]
        stride = max(1, self.args.sample_stride)
        for v in range(0, depth.height, stride):
            for u in range(0, depth.width, stride):
                # Active P5-02 class 0 is regolith; exclude rock and shadow.
                if int(labels[v, u]) != 0:
                    continue
                d = float(depth_m[v, u])
                bin_id = self._range_bin(d, ranges)
                if bin_id is None:
                    continue
                xnorm = (u - intrinsics.cx) / intrinsics.fx
                ynorm = (v - intrinsics.cy) / intrinsics.fy
                truth = self._ray_truth(camera_to_map, xnorm, ynorm)
                if truth is None:
                    continue
                point = self._target_point_to_world(
                    camera_to_map.apply(np.asarray([[xnorm * d, ynorm * d, d]]))[0])
                dx, dy, dz = point[0] - truth[0], point[1] - truth[1], point[2] - truth[2]
                row = {
                    "run_id": self.args.run_id, "stamp_ns": self._stamp_ns(depth),
                    "u": u, "v": v, "depth_m": d, "range_bin": bin_id,
                    "x_map": point[0], "y_map": point[1], "z_map": point[2],
                    "x_truth": truth[0], "y_truth": truth[1], "z_truth": truth[2],
                    "horizontal_error_m": math.hypot(dx, dy),
                    "vertical_error_m": abs(dz), "error_3d_m": math.sqrt(dx*dx + dy*dy + dz*dz),
                }
                self.writer.writerow(row)
                self.points.append(row)
        self.csv_file.flush()

    def _finish_if_due(self) -> None:
        if self.done or time.monotonic() - self.started_wall < self.args.duration_s:
            return
        self.done = True
        self.csv_file.close()
        summary = self._summary()
        path = self.output / "projection_summary.json"
        path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"P6 calibration capture finished: {summary['result']} ")
        self.get_logger().info(f"P6 projection summary: {path}")

    def _summary(self) -> dict:
        gates = self.gates["projection"]
        ranges = [f"bin_{i+1}" for i in range(len(gates["depth_range_bins_m"]))]
        by_bin: dict[str, list[dict]] = {name: [] for name in ranges}
        for row in self.points:
            by_bin[str(row["range_bin"])].append(row)
        metrics = {}
        passed = True
        for name, rows in by_bin.items():
            if rows:
                horiz = np.asarray([r["horizontal_error_m"] for r in rows], dtype=float)
                vert = np.asarray([r["vertical_error_m"] for r in rows], dtype=float)
                metric = {
                    "count": len(rows), "horizontal_p95_m": float(np.percentile(horiz, 95)),
                    "horizontal_max_m": float(np.max(horiz)),
                    "vertical_p95_m": float(np.percentile(vert, 95)),
                    "vertical_max_m": float(np.max(vert)),
                }
                bin_pass = (len(rows) >= gates["minimum_points_per_range_bin_per_run"] and
                            metric["horizontal_p95_m"] <= gates["max_horizontal_error_p95_m"] and
                            metric["horizontal_max_m"] <= gates["max_horizontal_error_m"] and
                            metric["vertical_p95_m"] <= gates["max_vertical_error_p95_m"] and
                            metric["vertical_max_m"] <= gates["max_vertical_error_m"])
            else:
                metric, bin_pass = {"count": 0}, False
            metric["pass"] = bool(bin_pass)
            metrics[name] = metric
            passed &= bin_pass
        total = len(self.points)
        projection_pass = bool(passed and total >= gates["minimum_valid_ground_points_per_run"])
        if self.args.skip_projection_gates:
            passed = True
        else:
            passed = projection_pass
        layer_summary = self._layer_summary()
        if self.args.capture_kind == "flat":
            passed &= layer_summary.get("flat_pass", False)
        elif self.args.capture_kind == "bowl":
            passed &= layer_summary.get("bowl_pass", False)
        return {
            "schema_version": 1, "run_id": self.args.run_id,
            "gate_id": self.gates["gate_id"], "terrain_seed": self.gates["reference"]["terrain_seed"],
            "scope": self.gates["scope"], "duration_wall_s": self.args.duration_s,
            "reference_surface": self.args.reference_surface,
            "projection_gate_skipped": self.args.skip_projection_gates,
            "projection_gate_result": "SKIPPED" if self.args.skip_projection_gates else
                                      ("PASS" if projection_pass else "FAIL"),
            "truth_world_pose": {
                "x_m": self.args.truth_world_x_offset_m,
                "y_m": self.args.truth_world_y_offset_m,
                "z_m": self.args.truth_world_z_offset_m,
            "roll_rad": self.args.truth_world_roll_offset_rad,
            "pitch_rad": self.args.truth_world_pitch_offset_rad,
                "yaw_rad": self.args.truth_world_yaw_offset_rad,
            },
            "camera_calibration": self.camera_calibration,
            "sample_count": total, "range_bins": metrics,
            "terrain_layers": layer_summary,
            "result": "PASS" if passed else "FAIL",
        }

    def _layer_summary(self) -> dict:
        if not self.layers:
            return {"result": "FAIL", "reason": "no live terrain layers received"}
        msg = self.layers.get("slope") or next(iter(self.layers.values()))
        width, height = int(msg.info.width), int(msg.info.height)
        resolution = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        xs = ox + (np.arange(width, dtype=np.float64) + 0.5) * resolution
        ys = oy + (np.arange(height, dtype=np.float64) + 0.5) * resolution
        map_x, map_y = np.meshgrid(xs, ys)
        try:
            odom_from_map_msg = self.tf_buffer.lookup_transform(
                "odom", msg.header.frame_id, Time())
            odom_from_map = self._transform(odom_from_map_msg)
        except Exception as exc:
            return {"frame": msg.header.frame_id, "result": "FAIL",
                    "reason": f"map registration TF unavailable: {exc}"}
        world_rotation = (self._world_from_target_rotation()
                          @ odom_from_map.rotation_matrix())
        world_translation = self._target_point_to_world(
            np.asarray(odom_from_map.translation, dtype=np.float64))
        map_points = np.stack([map_x, map_y, np.zeros_like(map_x)], axis=-1)
        world_points = map_points @ world_rotation.T + world_translation
        world_x, world_y = world_points[..., 0], world_points[..., 1]
        reference_z = np.empty((height, width), dtype=np.float64)
        for row in range(height):
            for col in range(width):
                value = (self.args.plane_world_z_m
                         if self.args.reference_surface == "plane"
                         else self._mesh_height(float(world_x[row, col]), float(world_y[row, col])))
                if value is None:
                    reference_z[row, col] = np.nan
                else:
                    point_world = np.asarray([world_x[row, col], world_y[row, col], value])
                    reference_z[row, col] = (
                        world_rotation.T @ (point_world - world_translation))[2]
        valid_z = np.isfinite(reference_z)
        seen_z = np.where(valid_z, 1, -1).astype(np.int64)
        labels = np.where(valid_z, 1, 0).astype(np.uint8)
        confidence = np.ones((height, width), dtype=np.float32)
        seen = np.where(valid_z, 1, -1).astype(np.int64)
        reference = derive_terrain_layers(
            reference_z, seen_z, labels, confidence, seen, now_ns=1,
            resolution=resolution)
        bowl_radius_m = self.gates["terrain_geometry"]["bowl_radius_m"]
        center_x, center_y = self.gates["terrain_geometry"]["bowl_center_world_m"]
        bowl_surface = (np.hypot(world_x - center_x, world_y - center_y)
                        <= bowl_radius_m)
        result: dict[str, object] = {"frame": msg.header.frame_id,
                                     "resolution_m": resolution, "layers": {}}
        all_pass = True
        cell_diagnostics: list[dict[str, float | int | str]] = []
        for name in ("slope", "roughness", "crater_risk"):
            live_msg = self.layers.get(name)
            if live_msg is None or live_msg.info.width != width or live_msg.info.height != height:
                result["layers"][name] = {"count": 0, "pass": False}
                all_pass = False
                continue
            live = np.asarray(live_msg.data, dtype=np.int16).reshape(height, width)
            expected = reference[name].astype(np.int16)
            known = (live >= 0) & (expected >= 0) & valid_z
            if self.args.capture_kind == "bowl":
                known &= bowl_surface
            count = int(np.count_nonzero(known))
            if count:
                errors = np.abs(live[known] - expected[known])
                if self.args.capture_kind == "bowl":
                    for row, col in zip(*np.nonzero(known)):
                        cell_diagnostics.append({
                            "layer": name, "world_x": float(world_x[row, col]),
                            "world_y": float(world_y[row, col]),
                            "live": int(live[row, col]), "reference": int(expected[row, col]),
                            "absolute_error": int(abs(int(live[row, col]) - int(expected[row, col]))),
                        })
                score = {"count": count, "p95_absolute_error_points": float(np.percentile(errors, 95)),
                         "max_absolute_error_points": int(np.max(errors))}
                gate = self.gates["terrain_geometry"]["max_reference_risk_error_p95_points"]
                score["pass"] = count >= self.gates["terrain_geometry"]["minimum_valid_cells_per_surface"] and score["p95_absolute_error_points"] <= gate
            else:
                score = {"count": 0, "pass": False}
            result["layers"][name] = score
            all_pass &= bool(score["pass"])

        if self.args.capture_kind == "flat":
            if self.args.reference_surface == "plane":
                # The calibration fixture is a single known plane. Gate every
                # observed valid cell on that plane, independent of map origin.
                home = valid_z
            else:
                # Match the SDF staging pad footprint centered at world (0, 0),
                # leaving a 0.1 m edge margin around its 4 m square surface.
                home = ((world_x >= -1.9) & (world_x <= 1.9) &
                        (world_y >= -1.9) & (world_y <= 1.9))
            for name, limit in (("slope", self.gates["terrain_geometry"]["maximum_flat_surface_slope_risk_p95"]),
                                ("crater_risk", self.gates["terrain_geometry"]["maximum_flat_surface_crater_risk_p95"])):
                live_msg = self.layers.get(name)
                if live_msg is None:
                    continue
                live = np.asarray(live_msg.data, dtype=np.int16).reshape(height, width)
                selected = live[home & (live >= 0)]
                p95 = float(np.percentile(selected, 95)) if selected.size else math.inf
                result[f"flat_{name}_risk_p95"] = p95
                all_pass &= selected.size >= self.gates["terrain_geometry"]["minimum_valid_cells_per_surface"] and p95 <= limit
            result["flat_pass"] = bool(all_pass)
        elif self.args.capture_kind == "bowl":
            near_center = bowl_surface
            live_msg = self.layers.get("crater_risk")
            values = (np.asarray(live_msg.data, dtype=np.int16).reshape(height, width)
                      if live_msg is not None else np.full((height, width), -1))
            selected = values[near_center & (values >= 0)]
            center_max = int(np.max(selected)) if selected.size else -1
            bowl_pass = (selected.size >= self.gates["terrain_geometry"]["minimum_valid_cells_per_surface"] and
                         center_max >= self.gates["terrain_geometry"]["minimum_bowl_center_crater_risk"])
            result["bowl_center_valid_cells"] = int(selected.size)
            result["bowl_center_crater_risk_max"] = center_max
            result["bowl_pass"] = bool(bowl_pass)
            all_pass &= bowl_pass
            diagnostic_path = self.output / "layer_cells.csv"
            with diagnostic_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=[
                    "layer", "world_x", "world_y", "live", "reference", "absolute_error"])
                writer.writeheader()
                writer.writerows(cell_diagnostics)
        result["result"] = "PASS" if all_pass else "FAIL"
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--capture-kind", choices=("flat", "bowl", "other"), default="other")
    parser.add_argument("--gates", default="config/p6_calibration_gates.json")
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--sample-stride", type=int, default=8)
    parser.add_argument("--max-reference-depth-m", type=float, default=5.1)
    parser.add_argument("--reference-surface", choices=("terrain", "plane"), default="terrain")
    parser.add_argument("--plane-world-z-m", type=float, default=-2.788)
    parser.add_argument("--skip-projection-gates", action="store_true",
                        help="evaluate only terrain layers for a dedicated geometry capture")
    parser.add_argument("--target-frame", default="odom",
                        help="world-aligned frame used for comparison with the seeded terrain")
    parser.add_argument("--truth-world-x-offset-m", type=float, default=0.0)
    parser.add_argument("--truth-world-y-offset-m", type=float, default=0.0)
    parser.add_argument("--truth-world-z-offset-m", type=float, default=-1.788)
    parser.add_argument("--truth-world-yaw-offset-rad", type=float, default=0.0)
    parser.add_argument("--truth-world-roll-offset-rad", type=float, default=0.0)
    parser.add_argument("--truth-world-pitch-offset-rad", type=float, default=0.0)
    parser.add_argument("--depth-topic", default="/lunabot/depth/image_raw")
    parser.add_argument("--mask-topic", default="/lunabot/terrain/segmentation")
    parser.add_argument("--camera-info-topic", default="/lunabot/depth/camera_info")
    args = parser.parse_args()
    rclpy.init()
    node = P6CalibrationRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.done:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
