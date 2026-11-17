#!/usr/bin/env python3
"""LunaBot V4 Phase E: lightweight RGB-D terrain segmentation.

This node deliberately stays at the perception layer. It consumes the existing
Gazebo RGB and depth camera topics and publishes the qualified P5-02 three-class
mask plus a visual overlay; it does not modify the planner, controller, or rover
model. The class IDs are regolith=0, rock=1, shadow=2, unknown=255.

Mask labels are encoded as mono8 values: 0=regolith, 1=rock, 2=shadow,
255=unknown. The runtime also publishes calibrated per-pixel confidence.
"""

from __future__ import annotations

import math
import struct
import os
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from p502_threeclass_inference import P502ThreeClassRuntime


UNKNOWN = 0
REGOLITH = 0
ROCK = 1
SHADOW = 2
UNKNOWN_ID = 255
# IDs used only by the explicit no-model development fallback below.
TERRAIN = 1
OBSTACLE = 2


class TerrainSegmentation(Node):
    """Segment synchronized RGB-D frames into auditable terrain classes."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_segmentation")
        self.declare_parameter("image_topic", "/lunabot/camera/image_raw")
        self.declare_parameter("depth_topic", "/lunabot/depth/image_raw")
        self.declare_parameter("mask_topic", "/lunabot/terrain/segmentation")
        self.declare_parameter("overlay_topic", "/lunabot/terrain/overlay")
        self.declare_parameter(
            "confidence_topic", "/lunabot/terrain/segmentation_confidence")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/segmentation/status")
        self.declare_parameter("ground_roi_start", 0.35)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("max_rate", 5.0)
        self.declare_parameter("saturation_threshold", 0.55)
        self.declare_parameter("model_package", os.environ.get("P502_MODEL_PACKAGE", ""))

        get = self.get_parameter
        self.image_topic = str(get("image_topic").value)
        self.depth_topic = str(get("depth_topic").value)
        self.mask_topic = str(get("mask_topic").value)
        self.overlay_topic = str(get("overlay_topic").value)
        self.confidence_topic = str(get("confidence_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.ground_roi_start = max(0.0, min(1.0, float(get("ground_roi_start").value)))
        self.max_depth = max(0.1, float(get("max_depth").value))
        self.max_rate = max(0.1, float(get("max_rate").value))
        self.saturation_threshold = max(
            0.0, min(1.0, float(get("saturation_threshold").value)))
        package = str(get("model_package").value).strip()
        self.semantic_model = None
        if package:
            self.semantic_model = P502ThreeClassRuntime(Path(package))

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self.mask_pub = self.create_publisher(Image, self.mask_topic, image_qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, image_qos)
        self.confidence_pub = self.create_publisher(
            Image, self.confidence_topic, image_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)

        self.latest_image: Optional[Image] = None
        self.latest_image_monotonic_s = 0.0
        self.latest_depth: Optional[Image] = None
        self.last_stamp_ns = 0
        self.last_process_ns = 0
        self.frame_count = 0
        self.last_status = ""
        self.timer = self.create_timer(0.05, self._process_latest)
        self.publish_status("SEGMENTATION_WAITING_FOR_RGB_D")

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg
        import time
        self.latest_image_monotonic_s = time.monotonic()

    def _depth_callback(self, msg: Image) -> None:
        self.latest_depth = msg

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _encoding_lower(msg: Image) -> str:
        return (msg.encoding or "").lower().replace(" ", "")

    def _rgb_at(self, msg: Image, x: int, y: int) -> tuple[int, int, int]:
        encoding = self._encoding_lower(msg)
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            return 128, 128, 128
        offset = y * msg.step + x * channels
        if offset + channels > len(msg.data):
            return 128, 128, 128
        values = msg.data[offset:offset + channels]
        if encoding.startswith("bgr"):
            return int(values[2]), int(values[1]), int(values[0])
        return int(values[0]), int(values[1]), int(values[2])

    @classmethod
    def _rgb_array(cls, msg: Image):
        import numpy as np

        encoding = cls._encoding_lower(msg)
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            raise ValueError(f"Unsupported camera encoding for P5-02 model: {msg.encoding}")
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        pixels = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if encoding.startswith("bgr"):
            pixels = pixels[:, :, [2, 1, 0]]
        return pixels[:, :, :3].copy()

    def _depth_at(self, msg: Image, x: int, y: int) -> float:
        encoding = self._encoding_lower(msg)
        if msg.width <= 0 or msg.height <= 0:
            return math.nan
        offset = y * msg.step
        try:
            if encoding in ("16uc1", "mono16"):
                offset += x * 2
                raw = struct.unpack_from(
                    ">H" if msg.is_bigendian else "<H", msg.data, offset)[0]
                return raw / 1000.0
            if encoding in ("32fc1", "32fc"):
                offset += x * 4
                return float(struct.unpack_from(
                    ">f" if msg.is_bigendian else "<f", msg.data, offset)[0])
            if encoding in ("64fc1", "64fc"):
                offset += x * 8
                return float(struct.unpack_from(
                    ">d" if msg.is_bigendian else "<d", msg.data, offset)[0])
        except (IndexError, struct.error):
            return math.nan
        return math.nan

    @staticmethod
    def _make_image(header, width: int, height: int, encoding: str,
                    step: int, data: bytes) -> Image:
        msg = Image()
        msg.header = header
        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data
        return msg

    def _process_latest(self) -> None:
        image = self.latest_image
        depth = self.latest_depth
        if image is None or depth is None or image.width <= 0 or image.height <= 0:
            return
        stamp_ns = max(self._stamp_ns(image), self._stamp_ns(depth))
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= self.last_stamp_ns:
            return
        if self.last_process_ns and now_ns - self.last_process_ns < int(1e9 / self.max_rate):
            return
        self.last_stamp_ns = stamp_ns
        self.last_process_ns = now_ns

        width, height = int(image.width), int(image.height)
        if self.semantic_model is not None:
            try:
                result = self.semantic_model.infer(
                    self._rgb_array(image),
                    captured_monotonic_s=self.latest_image_monotonic_s)
            except (ValueError, RuntimeError) as exc:
                self.publish_status(f"SEGMENTATION_MODEL_ERROR {exc}")
                return
            labels = result.labels
            confidence_values = result.confidence
            if result.stale or result.status.startswith("INVALID_"):
                labels.fill(UNKNOWN_ID)
                confidence_values.fill(0.0)
            confidence = (confidence_values * 255.0).round().astype("uint8")
            overlay = bytearray(width * height * 3)
            counts = {REGOLITH: 0, ROCK: 0, SHADOW: 0}
            colors = {REGOLITH: (40, 210, 60), ROCK: (235, 55, 45),
                      SHADOW: (110, 90, 155)}
            for y in range(height):
                for x in range(width):
                    label = int(labels[y, x])
                    if label in counts:
                        counts[label] += 1
                        color = colors[label]
                    else:
                        color = (20, 20, 20)
                    offset = (y * width + x) * 3
                    overlay[offset:offset + 3] = bytes(color)
            self.mask_pub.publish(self._make_image(
                image.header, width, height, "mono8", width, labels.tobytes()))
            self.confidence_pub.publish(self._make_image(
                image.header, width, height, "mono8", width, confidence.tobytes()))
            self.overlay_pub.publish(self._make_image(
                image.header, width, height, "rgb8", width * 3, bytes(overlay)))
            self.frame_count += 1
            unknown_count = width * height - sum(counts.values())
            self.publish_status(
                f"SEGMENTATION_PASS mode=three_class frames={self.frame_count} "
                f"provider={result.provider} latency_ms={result.latency_ms:.1f} "
                f"input_age_ms={result.input_age_ms:.1f} "
                f"regolith={counts[REGOLITH]} rock={counts[ROCK]} "
                f"shadow={counts[SHADOW]} unknown={unknown_count}")
            return

        # Keep the dependency-light rule segmenter available for isolated node
        # development; Phase L always supplies the qualified model package.
        from semantic_layers import decision_strength
        mask = bytearray(width * height)
        confidence = bytearray(width * height)
        overlay = bytearray(width * height * 3)
        terrain_count = 0
        obstacle_count = 0
        valid_count = 0
        for y in range(height):
            depth_y = min(depth.height - 1, int(y * depth.height / height))
            lower_roi = y / max(1, height - 1) >= self.ground_roi_start
            for x in range(width):
                depth_x = min(depth.width - 1, int(x * depth.width / width))
                distance = self._depth_at(depth, depth_x, depth_y)
                valid = math.isfinite(distance) and 0.05 <= distance <= self.max_depth
                label = UNKNOWN
                if valid:
                    valid_count += 1
                    red, green, blue = self._rgb_at(image, x, y)
                    luminance = max(red, green, blue) / 255.0
                    saturation = (max(red, green, blue) - min(red, green, blue)) / 255.0
                    row_fraction = y / max(1, height - 1)
                    roi_margin = ((row_fraction - self.ground_roi_start)
                                  / max(1e-6, 1.0 - self.ground_roi_start)
                                  if lower_roi else
                                  (self.ground_roi_start - row_fraction)
                                  / max(1e-6, self.ground_roi_start))
                    saturation_margin = abs(saturation - self.saturation_threshold) / max(
                        1e-6, self.saturation_threshold, 1.0 - self.saturation_threshold)
                    brightness_margin = abs(luminance - 0.98) / 0.98
                    confidence[y * width + x] = int(round(255 * decision_strength(
                        roi_margin=roi_margin,
                        appearance_margin=max(saturation_margin, brightness_margin))))
                    # Lunar terrain is expected to be a low-saturation return in
                    # the lower field. Bright or saturated returns are treated
                    # conservatively as obstacle candidates.
                    if lower_roi and saturation <= self.saturation_threshold and luminance < 0.98:
                        label = TERRAIN
                        terrain_count += 1
                    else:
                        label = OBSTACLE
                        obstacle_count += 1
                mask[y * width + x] = label
                index = (y * width + x) * 3
                if label == TERRAIN:
                    overlay[index:index + 3] = bytes((40, 210, 60))
                elif label == OBSTACLE:
                    overlay[index:index + 3] = bytes((235, 55, 45))
                else:
                    overlay[index:index + 3] = bytes((20, 20, 20))

        header = image.header
        self.mask_pub.publish(self._make_image(
            header, width, height, "mono8", width, bytes(mask)))
        self.confidence_pub.publish(self._make_image(
            header, width, height, "mono8", width, bytes(confidence)))
        self.overlay_pub.publish(self._make_image(
            header, width, height, "rgb8", width * 3, bytes(overlay)))
        self.frame_count += 1
        unknown_count = width * height - valid_count
        self.publish_status(
            f"SEGMENTATION_PASS frames={self.frame_count} "
            f"width={width} height={height} valid={valid_count} "
            f"terrain={terrain_count} obstacle={obstacle_count} "
            f"unknown={unknown_count}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainSegmentation()
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
