#!/usr/bin/env python3
"""LunaBot V4 Phase F: semantic terrain map fusion.

The node fuses the Phase E RGB-D segmentation mask with depth and the
approved map->chassis localization transform. It accumulates observations in a
fixed map-frame grid and publishes a single auditable OccupancyGrid:

  -1 = unknown
   25 = terrain candidate
  100 = obstacle candidate

This is a semantic terrain map, not a second localization or SLAM system. It
uses the existing slam_toolbox TF and does not publish velocity or modify the
planner/controller path.
"""

from __future__ import annotations

from collections import deque
import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
from semantic_projection import (
    CameraIntrinsics, RigidTransform, decode_depth_buffer, project_synced_frame)
from semantic_layers import SemanticGrid, three_class_to_map_labels
from terrain_geometry import derive_terrain_layers


UNKNOWN = 0
TERRAIN = 1
OBSTACLE = 2


class SemanticTerrainMapper(Node):
    """Accumulate RGB-D segmentation labels in the existing map frame."""

    def __init__(self) -> None:
        super().__init__("lunabot_semantic_terrain_mapper")
        self.declare_parameter("mask_topic", "/lunabot/terrain/segmentation")
        self.declare_parameter("depth_topic", "/lunabot/depth/image_raw")
        self.declare_parameter("rgb_info_topic", "/lunabot/camera/camera_info")
        self.declare_parameter("depth_info_topic", "/lunabot/depth/camera_info")
        self.declare_parameter("map_topic", "/lunabot/terrain/semantic_map")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/semantic_map/status")
        self.declare_parameter(
            "confidence_topic", "/lunabot/terrain/segmentation_confidence")
        self.declare_parameter(
            "confidence_map_topic", "/lunabot/terrain/semantic_map/confidence")
        self.declare_parameter(
            "age_map_topic", "/lunabot/terrain/semantic_map/age")
        self.declare_parameter(
            "slope_map_topic", "/lunabot/terrain/semantic_map/slope")
        self.declare_parameter(
            "roughness_map_topic", "/lunabot/terrain/semantic_map/roughness")
        self.declare_parameter(
            "crater_risk_map_topic", "/lunabot/terrain/semantic_map/crater_risk")
        self.declare_parameter(
            "uncertainty_map_topic", "/lunabot/terrain/semantic_map/uncertainty")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("width", 160)
        self.declare_parameter("height", 160)
        self.declare_parameter("origin_x", -8.0)
        self.declare_parameter("origin_y", -8.0)
        self.declare_parameter("recenter_window", True)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("sample_stride", 8)
        self.declare_parameter("max_sync_skew_s", 0.075)
        self.declare_parameter("max_input_age_s", 0.5)
        self.declare_parameter("confidence_min", 0.35)
        self.declare_parameter("confidence_half_life_s", 10.0)
        self.declare_parameter("observation_ttl_s", 30.0)
        self.declare_parameter("update_rate", 2.0)

        get = self.get_parameter
        self.mask_topic = str(get("mask_topic").value)
        self.depth_topic = str(get("depth_topic").value)
        self.rgb_info_topic = str(get("rgb_info_topic").value)
        self.depth_info_topic = str(get("depth_info_topic").value)
        self.map_topic = str(get("map_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.confidence_topic = str(get("confidence_topic").value)
        self.confidence_map_topic = str(get("confidence_map_topic").value)
        self.age_map_topic = str(get("age_map_topic").value)
        self.slope_map_topic = str(get("slope_map_topic").value)
        self.roughness_map_topic = str(get("roughness_map_topic").value)
        self.crater_risk_map_topic = str(get("crater_risk_map_topic").value)
        self.uncertainty_map_topic = str(get("uncertainty_map_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.resolution = max(0.02, float(get("resolution").value))
        self.width = max(10, int(get("width").value))
        self.height = max(10, int(get("height").value))
        self.origin_x = float(get("origin_x").value)
        self.origin_y = float(get("origin_y").value)
        self.recenter_window = bool(get("recenter_window").value)
        self.max_depth = max(0.5, float(get("max_depth").value))
        self.sample_stride = max(1, int(get("sample_stride").value))
        self.max_sync_skew_ns = int(max(0.001, float(get("max_sync_skew_s").value)) * 1e9)
        self.max_input_age_ns = int(max(0.05, float(get("max_input_age_s").value)) * 1e9)
        self.confidence_min = min(1.0, max(0.0, float(get("confidence_min").value)))
        self.confidence_half_life_s = max(
            0.1, float(get("confidence_half_life_s").value))
        self.observation_ttl_s = max(0.2, float(get("observation_ttl_s").value))
        self.update_rate = max(0.2, float(get("update_rate").value))

        self.mask_sub = self.create_subscription(
            Image, self.mask_topic, self._mask_callback, qos_profile_sensor_data)
        self.confidence_sub = self.create_subscription(
            Image, self.confidence_topic, self._confidence_callback,
            qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, self.rgb_info_topic, self._rgb_info_callback, qos_profile_sensor_data)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, self.depth_info_topic, self._depth_info_callback, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.confidence_map_pub = self.create_publisher(
            OccupancyGrid, self.confidence_map_topic, map_qos)
        self.age_map_pub = self.create_publisher(OccupancyGrid, self.age_map_topic, map_qos)
        self.geometry_pubs = {
            "slope": self.create_publisher(OccupancyGrid, self.slope_map_topic, map_qos),
            "roughness": self.create_publisher(
                OccupancyGrid, self.roughness_map_topic, map_qos),
            "crater_risk": self.create_publisher(
                OccupancyGrid, self.crater_risk_map_topic, map_qos),
            "uncertainty": self.create_publisher(
                OccupancyGrid, self.uncertainty_map_topic, map_qos),
        }
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_mask: Optional[Image] = None
        self.latest_confidence: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        # Segmentation inference is slower than the camera stream. Keep a
        # short depth history so a delayed mask can use the depth frame closest
        # to its capture stamp instead of whichever depth frame arrived last.
        self.depth_history: deque[Image] = deque(maxlen=30)
        self.latest_rgb_info: Optional[CameraInfo] = None
        self.latest_depth_info: Optional[CameraInfo] = None
        self.last_mask_stamp = 0
        self.last_process_ns = 0
        self.frame_count = 0
        self.observation_count = 0
        self.last_status = ""
        self.grid = SemanticGrid(
            self.width, self.height, self.resolution, self.origin_x, self.origin_y,
            half_life_s=self.confidence_half_life_s,
            ttl_s=self.observation_ttl_s, min_confidence=self.confidence_min)
        self.last_clear_count = 0
        self.last_recenter_discarded = 0
        self.timer = self.create_timer(1.0 / self.update_rate, self._process_latest)
        self.publish_status("SEMANTIC_MAP_WAITING_FOR_SEGMENTATION")

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _mask_callback(self, msg: Image) -> None:
        self.latest_mask = msg

    def _confidence_callback(self, msg: Image) -> None:
        self.latest_confidence = msg

    def _depth_callback(self, msg: Image) -> None:
        self.latest_depth = msg
        self.depth_history.append(msg)

    def _rgb_info_callback(self, msg: CameraInfo) -> None:
        self.latest_rgb_info = msg

    def _depth_info_callback(self, msg: CameraInfo) -> None:
        self.latest_depth_info = msg

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _mask_array(msg: Image) -> np.ndarray:
        if (msg.encoding or "").lower() not in ("mono8", "8uc1"):
            raise ValueError(f"semantic mask must be mono8, got {msg.encoding}")
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width or len(msg.data) < msg.step * msg.height:
            raise ValueError("semantic mask buffer dimensions are invalid")
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return rows[:, :msg.width].copy()

    @staticmethod
    def _depth_array(msg: Image) -> np.ndarray:
        return decode_depth_buffer(msg.data, msg.width, msg.height, msg.step,
                                   msg.encoding, bool(msg.is_bigendian))

    @staticmethod
    def _intrinsics(info: CameraInfo, width: int, height: int) -> CameraIntrinsics:
        if info.width != width or info.height != height:
            raise ValueError("CameraInfo dimensions do not match the image")
        if any(abs(float(value)) > 1e-9 for value in info.d):
            raise ValueError("distorted CameraInfo is unsupported; simulated cameras must be rectified")
        return CameraIntrinsics.from_k(width, height, info.k)

    @staticmethod
    def _rigid_transform(message) -> RigidTransform:
        transform = message.transform
        translation, rotation = transform.translation, transform.rotation
        return RigidTransform((translation.x, translation.y, translation.z),
                              (rotation.x, rotation.y, rotation.z, rotation.w))

    def _lookup_transform_at(self, target: str, source: str, stamp_ns: int):
        try:
            stamp = Time(nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type)
            return self.tf_buffer.lookup_transform(target, source, stamp)
        except tf2_ros.TransformException as exc:
            self.publish_status(f"SEMANTIC_MAP_WAITING_FOR_TF {exc}")
            return None

    def _lookup_transform_across_stamps(self, target: str, target_stamp_ns: int,
                                        source: str, source_stamp_ns: int,
                                        fixed_frame: str):
        try:
            clock_type = self.get_clock().clock_type
            target_time = Time(nanoseconds=target_stamp_ns, clock_type=clock_type)
            source_time = Time(nanoseconds=source_stamp_ns, clock_type=clock_type)
            return self.tf_buffer.lookup_transform_full(
                target, target_time, source, source_time, fixed_frame)
        except tf2_ros.TransformException as exc:
            self.publish_status(f"SEMANTIC_MAP_WAITING_FOR_TF {exc}")
            return None

    def _publish_map(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp = self.get_clock().now().to_msg()
        layers = {
            "semantic": self.grid.semantic_data(now_ns),
            "confidence": self.grid.confidence_data(now_ns),
            "age": self.grid.age_data(now_ns),
        }
        geometry = derive_terrain_layers(
            self.grid.elevation, self.grid.elevation_seen_ns,
            self.grid.labels, self.grid.confidence, self.grid.last_seen_ns,
            now_ns=now_ns, resolution=self.resolution,
            confidence_half_life_s=self.confidence_half_life_s,
            observation_ttl_s=self.observation_ttl_s)
        publishers = {
            "semantic": self.map_pub,
            "confidence": self.confidence_map_pub,
            "age": self.age_map_pub,
            **self.geometry_pubs,
        }
        for name, data in layers.items():
            self._publish_layer(publishers[name], data, stamp)
        for name, layer in geometry.items():
            self._publish_layer(publishers[name], layer.ravel().tolist(), stamp)

    def _publish_layer(self, publisher, data: list[int], stamp) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        # This is a 2D SLAM map frame, not Gazebo's 3D world frame. Keep all
        # OccupancyGrid layers on its z=0 plane; sensor-point elevations are
        # terrain data and must not move the entire map display vertically.
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        publisher.publish(msg)

    def _filter_ground_level_obstacles(
            self, observations: dict[int, tuple[int, float]],
            elevations: dict[int, float]) -> tuple[int, int]:
        """Reject rock labels unsupported by local above-ground geometry.

        A dark shadow or a low-confidence visual false positive lies on the
        same surface as nearby regolith. A real rock return should rise above
        the locally observed terrain. Without enough nearby ground samples the
        semantic image alone cannot prove height, so leave that cell unknown;
        the independent LiDAR obstacle map still reports physical obstacles.
        """
        radius = max(2, int(math.ceil(1.0 / self.resolution)))
        known_ground = ((self.grid.labels == TERRAIN)
                        & (self.grid.last_seen_ns >= 0)
                        & np.isfinite(self.grid.elevation))
        ground_elevation = np.where(known_ground, self.grid.elevation, np.nan)
        for index, (label, confidence) in observations.items():
            elevation = elevations.get(index)
            if (label == TERRAIN and confidence >= self.confidence_min
                    and elevation is not None and np.isfinite(elevation)):
                row, col = divmod(int(index), self.width)
                ground_elevation[row, col] = float(elevation)

        rejected = unsupported = 0
        for index, (label, confidence) in list(observations.items()):
            if label != OBSTACLE:
                continue
            z = elevations.get(index)
            if z is None or not np.isfinite(z):
                observations[index] = (UNKNOWN, 0.0)
                rejected += 1
                continue
            row, col = divmod(int(index), self.width)
            row0, row1 = max(0, row - radius), min(self.height, row + radius + 1)
            col0, col1 = max(0, col - radius), min(self.width, col + radius + 1)
            nearby = ground_elevation[row0:row1, col0:col1]
            nearby = nearby[np.isfinite(nearby)]
            if len(nearby) < 3:
                # RGB semantics cannot distinguish a rock from the rover's
                # shadow without geometric support. Keep the cell unknown;
                # the LiDAR obstacle map provides the independent safety path.
                unsupported += 1
                observations[index] = (UNKNOWN, 0.0)
            else:
                ground = float(np.median(nearby))
                if abs(float(z) - ground) <= 0.12:
                    # Geometry says this visually dark/rock-labeled return is
                    # on the ground. Promote it to terrain so it can also
                    # replace a stale shadow false-positive already in-grid.
                    observations[index] = (TERRAIN, 1.0)
                elif float(z) < ground:
                    observations[index] = (UNKNOWN, 0.0)
                if float(z) < ground + 0.12:
                    rejected += 1
        return rejected, unsupported

    def _process_latest(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.last_clear_count = self.grid.decay(now_ns)
        if self.last_clear_count or np.any(self.grid.labels):
            self._publish_map()
        mask = self.latest_mask
        confidence_msg = self.latest_confidence
        depth = self.latest_depth
        rgb_info = self.latest_rgb_info
        depth_info = self.latest_depth_info
        if (mask is None or confidence_msg is None or depth is None
                or rgb_info is None or depth_info is None
                or mask.width <= 0 or mask.height <= 0):
            return
        mask_stamp_ns = self._stamp_ns(mask)
        if self.depth_history:
            depth = min(self.depth_history,
                        key=lambda frame: abs(mask_stamp_ns - self._stamp_ns(frame)))
        depth_stamp_ns = self._stamp_ns(depth)
        if mask_stamp_ns <= self.last_mask_stamp:
            return
        skew_ns = abs(mask_stamp_ns - depth_stamp_ns)
        if skew_ns > self.max_sync_skew_ns:
            self.publish_status(
                f"SEMANTIC_MAP_WAITING_FOR_SYNC skew_ms={skew_ns / 1e6:.1f} "
                f"limit_ms={self.max_sync_skew_ns / 1e6:.1f}")
            return
        confidence_stamp_ns = self._stamp_ns(confidence_msg)
        if (abs(mask_stamp_ns - confidence_stamp_ns) > self.max_sync_skew_ns
                or confidence_msg.header.frame_id != mask.header.frame_id
                or confidence_msg.width != mask.width
                or confidence_msg.height != mask.height):
            self.publish_status("SEMANTIC_MAP_WAITING_FOR_CONFIDENCE_SYNC")
            return
        age_ns = now_ns - max(mask_stamp_ns, depth_stamp_ns)
        if age_ns < -self.max_sync_skew_ns:
            self.publish_status(f"SEMANTIC_MAP_WAITING_FOR_FRESH_INPUT future_ms={-age_ns / 1e6:.1f}")
            return
        if age_ns > self.max_input_age_ns:
            self.publish_status(
                f"SEMANTIC_MAP_WAITING_FOR_FRESH_INPUT age_ms={age_ns / 1e6:.1f} "
                f"limit_ms={self.max_input_age_ns / 1e6:.1f}")
            return

        rgb_frame = rgb_info.header.frame_id or mask.header.frame_id
        depth_frame = depth_info.header.frame_id or depth.header.frame_id
        if (not rgb_frame or not depth_frame or mask.header.frame_id != rgb_frame
                or depth.header.frame_id != depth_frame):
            self.publish_status("SEMANTIC_MAP_WAITING_FOR_CALIBRATION frame_id_mismatch")
            return
        map_from_depth_msg = self._lookup_transform_at(
            self.map_frame, depth_frame, depth_stamp_ns)
        rgb_from_depth_msg = self._lookup_transform_across_stamps(
            rgb_frame, mask_stamp_ns, depth_frame, depth_stamp_ns, self.map_frame)
        if map_from_depth_msg is None or rgb_from_depth_msg is None:
            return

        camera_pose = map_from_depth_msg.transform.translation
        self.last_recenter_discarded = (
            self.grid.recenter(camera_pose.x, camera_pose.y)
            if self.recenter_window else 0)
        self.origin_x, self.origin_y = self.grid.origin_x, self.grid.origin_y

        try:
            semantic_mask = three_class_to_map_labels(self._mask_array(mask))
            confidence = self._mask_array(confidence_msg).astype(np.float32) / 255.0
            depth_m = self._depth_array(depth)
            rgb_model = self._intrinsics(rgb_info, mask.width, mask.height)
            depth_model = self._intrinsics(depth_info, depth.width, depth.height)
            projected = project_synced_frame(
                semantic_mask, depth_m, depth_model, rgb_model,
                self._rigid_transform(rgb_from_depth_msg),
                self._rigid_transform(map_from_depth_msg),
                mask_stamp_ns=mask_stamp_ns, depth_stamp_ns=depth_stamp_ns,
                max_sync_skew_ns=self.max_sync_skew_ns,
                resolution=self.resolution, origin_x=self.origin_x,
                origin_y=self.origin_y, grid_width=self.width,
                grid_height=self.height, sample_stride=self.sample_stride,
                min_depth=0.1, max_depth=self.max_depth,
                confidence_image=confidence)
        except (ValueError, TypeError) as exc:
            self.publish_status(f"SEMANTIC_MAP_INVALID_CALIBRATION {exc}")
            return
        if projected.status in ("WAITING_FOR_SYNC", "WAITING_FOR_TF"):
            self.publish_status(f"SEMANTIC_MAP_{projected.status} ")
            return

        # Mark the synchronized pair consumed only after TF and calibration pass.
        self.last_mask_stamp = mask_stamp_ns
        confidences = projected.observation_confidence or {}
        observations = {
            index: (label, confidences.get(index, 0.0))
            for index, label in projected.observations.items()
        }
        elevations = projected.observation_elevation or {}
        rejected_ground_level, unsupported_rock = self._filter_ground_level_obstacles(
            observations, elevations)
        update = self.grid.update(observations, now_ns, elevations=elevations)
        samples = len(projected.observations)
        self.frame_count += 1
        self.observation_count += samples
        terrain_cells, obstacle_cells, _ = self.grid.counts(now_ns)
        self._publish_map()
        self.publish_status(
            f"SEMANTIC_MAP_PASS frames={self.frame_count} "
            f"observations={self.observation_count} samples={samples} "
            f"accepted={update.accepted} low_confidence={update.rejected_low_confidence} "
            f"ground_level_rock_rejected={rejected_ground_level} "
            f"rock_without_ground_support={unsupported_rock} "
            f"cleared={update.cleared_stale + self.last_clear_count} "
            f"recenter_discarded={self.last_recenter_discarded} "
            f"valid_depth={projected.valid_depth_pixels} "
            f"occluded={projected.occluded_pixels} "
            f"out_of_grid={projected.out_of_grid_pixels} "
            f"terrain_cells={terrain_cells} obstacle_cells={obstacle_cells} "
            f"grid={self.width}x{self.height} memory_bytes={self.grid.memory_bytes} "
            f"coverage_m={self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} "
            f"origin={self.grid.origin_x:.2f},{self.grid.origin_y:.2f} "
            f"resolution={self.resolution:.2f} frame={self.map_frame}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticTerrainMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
