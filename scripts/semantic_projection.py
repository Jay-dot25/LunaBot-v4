"""Calibrated RGB-D semantic projection helpers with no ROS dependency."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


UNKNOWN = 0
TERRAIN = 1
OBSTACLE = 2


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, width: int, height: int, k: list[float] | tuple[float, ...]) -> "CameraIntrinsics":
        if width <= 0 or height <= 0 or len(k) != 9:
            raise ValueError("camera dimensions and 3x3 intrinsic matrix are required")
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            raise ValueError("camera focal lengths and principal point are invalid")
        return cls(width, height, fx, fy, cx, cy)


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform target_from_source, with a normalized xyzw quaternion."""

    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    def rotation_matrix(self) -> np.ndarray:
        x, y, z, w = self.rotation_xyzw
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("transform quaternion is invalid")
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=np.float64)

    def apply(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        return points @ self.rotation_matrix().T + np.asarray(self.translation, dtype=np.float64)


@dataclass(frozen=True)
class ProjectedFrame:
    status: str
    observations: dict[int, int]
    valid_depth_pixels: int = 0
    rgb_projected_pixels: int = 0
    occluded_pixels: int = 0
    unlabeled_pixels: int = 0
    out_of_grid_pixels: int = 0
    observation_confidence: dict[int, float] | None = None
    observation_elevation: dict[int, float] | None = None


def decode_depth_buffer(data: bytes | bytearray | memoryview, width: int, height: int,
                        step: int, encoding: str, is_bigendian: bool = False) -> np.ndarray:
    """Decode common ROS depth encodings to float32 metres, respecting row padding."""
    enc = (encoding or "").lower().replace(" ", "")
    if enc in ("32fc1", "32fc"):
        dtype, scale = (">f4" if is_bigendian else "<f4"), 1.0
    elif enc in ("64fc1", "64fc"):
        dtype, scale = (">f8" if is_bigendian else "<f8"), 1.0
    elif enc in ("16uc1", "mono16"):
        dtype, scale = (">u2" if is_bigendian else "<u2"), 0.001
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    bytes_per_pixel = np.dtype(dtype).itemsize
    if width <= 0 or height <= 0 or step < width * bytes_per_pixel or len(data) < step * height:
        raise ValueError("depth image buffer dimensions are invalid")
    view = np.ndarray((height, width), dtype=dtype, buffer=data,
                      strides=(step, bytes_per_pixel))
    return view.astype(np.float32) * scale


def project_pixel_to_target(u: float, v: float, depth_m: float,
                            intrinsics: CameraIntrinsics,
                            target_from_camera: RigidTransform) -> tuple[float, float, float]:
    """Project one optical-frame pixel (z-depth) through calibrated intrinsics and TF."""
    if not math.isfinite(depth_m) or depth_m <= 0:
        raise ValueError("depth must be finite and positive")
    camera_point = np.array([[(u - intrinsics.cx) * depth_m / intrinsics.fx,
                              (v - intrinsics.cy) * depth_m / intrinsics.fy,
                              depth_m]], dtype=np.float64)
    target_point = target_from_camera.apply(camera_point)[0]
    return float(target_point[0]), float(target_point[1]), float(target_point[2])


def project_synced_frame(
    mask: np.ndarray,
    depth_m: np.ndarray,
    depth_intrinsics: CameraIntrinsics,
    rgb_intrinsics: CameraIntrinsics,
    rgb_from_depth: RigidTransform | None,
    map_from_depth: RigidTransform | None,
    *,
    mask_stamp_ns: int,
    depth_stamp_ns: int,
    max_sync_skew_ns: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    grid_width: int,
    grid_height: int,
    sample_stride: int = 1,
    min_depth: float = 0.1,
    max_depth: float = 8.0,
    confidence_image: np.ndarray | None = None,
) -> ProjectedFrame:
    """Register visible depth points into the RGB mask, then project them to map.

    RGB z-buffering prevents a depth return hidden behind another sampled return
    from inheriting the foreground mask label. Transform availability is
    explicit: callers must supply transforms looked up at the sensor timestamp.
    """
    empty: dict[int, int] = {}
    if abs(int(mask_stamp_ns) - int(depth_stamp_ns)) > int(max_sync_skew_ns):
        return ProjectedFrame("WAITING_FOR_SYNC", empty)
    if rgb_from_depth is None or map_from_depth is None:
        return ProjectedFrame("WAITING_FOR_TF", empty)
    if mask.ndim != 2 or mask.shape != (rgb_intrinsics.height, rgb_intrinsics.width):
        raise ValueError("semantic mask shape must match RGB CameraInfo dimensions")
    if confidence_image is not None and confidence_image.shape != mask.shape:
        raise ValueError("confidence image shape must match RGB CameraInfo dimensions")
    if depth_m.ndim != 2 or depth_m.shape != (depth_intrinsics.height, depth_intrinsics.width):
        raise ValueError("depth image shape must match depth CameraInfo dimensions")
    if resolution <= 0 or grid_width <= 0 or grid_height <= 0 or sample_stride <= 0:
        raise ValueError("grid geometry and sample stride must be positive")
    if not 0 < min_depth < max_depth:
        raise ValueError("depth limits must satisfy 0 < min_depth < max_depth")

    height, width = depth_m.shape
    v_all, u_all = np.indices((height, width), dtype=np.float64)
    z_all = np.asarray(depth_m, dtype=np.float64)
    valid = np.isfinite(z_all) & (z_all >= min_depth) & (z_all <= max_depth)
    v, u, z = v_all[valid], u_all[valid], z_all[valid]
    if z.size == 0:
        return ProjectedFrame("NO_VALID_DEPTH", empty)

    depth_points = np.column_stack(((u - depth_intrinsics.cx) * z / depth_intrinsics.fx,
                                    (v - depth_intrinsics.cy) * z / depth_intrinsics.fy,
                                    z))
    rgb_points = rgb_from_depth.apply(depth_points)
    rgb_z = rgb_points[:, 2]
    projected_u = np.floor(rgb_intrinsics.fx * rgb_points[:, 0] / np.maximum(rgb_z, 1e-12)
                           + rgb_intrinsics.cx + 0.5).astype(np.int64)
    projected_v = np.floor(rgb_intrinsics.fy * rgb_points[:, 1] / np.maximum(rgb_z, 1e-12)
                           + rgb_intrinsics.cy + 0.5).astype(np.int64)
    in_rgb = ((rgb_z > 0) & (projected_u >= 0) & (projected_u < rgb_intrinsics.width)
              & (projected_v >= 0) & (projected_v < rgb_intrinsics.height))
    if not np.any(in_rgb):
        return ProjectedFrame("NO_RGB_PROJECTIONS", empty, int(z.size))

    # Z-buffer all valid depth points in the RGB image before downsampling map updates.
    z_buffer = np.full((rgb_intrinsics.height, rgb_intrinsics.width), np.inf, dtype=np.float64)
    np.minimum.at(z_buffer, (projected_v[in_rgb], projected_u[in_rgb]), rgb_z[in_rgb])
    visible = in_rgb & (rgb_z <= z_buffer[projected_v.clip(0, rgb_intrinsics.height - 1),
                                          projected_u.clip(0, rgb_intrinsics.width - 1)] + 1e-6)
    sampled = ((u.astype(np.int64) % sample_stride == 0)
               & (v.astype(np.int64) % sample_stride == 0))
    candidate = visible & sampled
    if not np.any(candidate):
        return ProjectedFrame("NO_VISIBLE_SAMPLES", empty, int(z.size),
                              int(np.count_nonzero(in_rgb)), int(np.count_nonzero(in_rgb & ~visible)))

    labels = np.asarray(mask, dtype=np.uint8)[projected_v[candidate], projected_u[candidate]]
    known = (labels == TERRAIN) | (labels == OBSTACLE)
    if not np.any(known):
        return ProjectedFrame("NO_LABELED_SAMPLES", empty, int(z.size),
                              int(np.count_nonzero(in_rgb)), int(np.count_nonzero(in_rgb & ~visible)),
                              int(labels.size))

    map_points = map_from_depth.apply(depth_points[candidate][known])
    gx = np.floor((map_points[:, 0] - origin_x) / resolution).astype(np.int64)
    gy = np.floor((map_points[:, 1] - origin_y) / resolution).astype(np.int64)
    in_grid = ((gx >= 0) & (gx < grid_width) & (gy >= 0) & (gy < grid_height))
    cells: dict[int, int] = {}
    confidences = (np.ones(labels.shape, dtype=np.float32) if confidence_image is None
                   else np.asarray(confidence_image, dtype=np.float32)[
                       projected_v[candidate], projected_u[candidate]][known])
    confidence_cells: dict[int, float] = {}
    elevation_samples: dict[int, list[float]] = {}
    for x_cell, y_cell, label, confidence, elevation in zip(
            gx[in_grid], gy[in_grid], labels[known][in_grid], confidences[in_grid],
            map_points[:, 2][in_grid]):
        index = int(y_cell * grid_width + x_cell)
        value = int(label)
        conf = float(confidence)
        elevation_samples.setdefault(index, []).append(float(elevation))
        if value == OBSTACLE or index not in cells or conf > confidence_cells[index]:
            cells[index] = value
            confidence_cells[index] = conf
    elevation_cells = {index: float(np.median(samples))
                       for index, samples in elevation_samples.items()}
    out_of_grid = int(np.count_nonzero(~in_grid))
    return ProjectedFrame("PASS" if cells else "NO_IN_GRID_SAMPLES", cells,
                          int(z.size), int(np.count_nonzero(in_rgb)),
                          int(np.count_nonzero(in_rgb & ~visible)),
                          int(np.count_nonzero(~known)), out_of_grid,
                          confidence_cells, elevation_cells)
