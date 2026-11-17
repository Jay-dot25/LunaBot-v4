"""Derive aligned slope, roughness, crater-risk, and uncertainty cost layers."""
from __future__ import annotations

import math

import numpy as np


def derive_terrain_layers(elevation: np.ndarray, elevation_seen_ns: np.ndarray,
                          labels: np.ndarray, confidence: np.ndarray,
                          last_seen_ns: np.ndarray, *, now_ns: int,
                          resolution: float, confidence_half_life_s: float = 10.0,
                          observation_ttl_s: float = 30.0,
                          crater_radius_cells: int = 3,
                          crater_min_depth_m: float = 0.04,
                          crater_full_depth_m: float = 0.25,
                          slope_full_degrees: float = 25.0,
                          roughness_full_m: float = 0.15) -> dict[str, np.ndarray]:
    """Return int8 0..100 risk layers; -1 means insufficient observations.

    Slope is the fitted local plane angle; roughness is local plane residual
    spread. Crater risk is a center depression relative to its surrounding
    annulus. Uncertainty combines effective semantic confidence and observation
    age. These are risk scores, not physical occupancy probabilities.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    seen_z = np.asarray(elevation_seen_ns, dtype=np.int64)
    labels = np.asarray(labels, dtype=np.uint8)
    confidence = np.asarray(confidence, dtype=np.float64)
    seen = np.asarray(last_seen_ns, dtype=np.int64)
    if (elevation.ndim != 2 or any(array.shape != elevation.shape for array in
                                   (seen_z, labels, confidence, seen))):
        raise ValueError("terrain layer inputs must share a 2D grid geometry")
    if (resolution <= 0 or confidence_half_life_s <= 0 or observation_ttl_s <= 0
            or crater_radius_cells < 2 or crater_full_depth_m <= crater_min_depth_m
            or slope_full_degrees <= 0 or roughness_full_m <= 0):
        raise ValueError("terrain layer geometry and thresholds must be positive")

    height, width = elevation.shape
    z_valid = np.isfinite(elevation) & (seen_z >= 0)
    slope = np.full((height, width), -1, dtype=np.int8)
    roughness = np.full_like(slope, -1)
    crater = np.full_like(slope, -1)

    # Fit a local plane over a 1 m window. Adjacent-cell differences amplify
    # RGB-D height noise into large slope spikes on otherwise smooth terrain.
    fit_radius = max(1, int(round(0.5 / resolution)))
    count = np.zeros((height, width), dtype=np.float64)
    sum_x = np.zeros_like(elevation)
    sum_y = np.zeros_like(elevation)
    sum_z = np.zeros_like(elevation)
    sum_xx = np.zeros_like(elevation)
    sum_xy = np.zeros_like(elevation)
    sum_yy = np.zeros_like(elevation)
    sum_xz = np.zeros_like(elevation)
    sum_yz = np.zeros_like(elevation)
    for dy in range(-fit_radius, fit_radius + 1):
        src_y0, src_y1 = max(0, -dy), min(height, height - dy)
        if src_y0 >= src_y1:
            continue
        dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
        for dx in range(-fit_radius, fit_radius + 1):
            src_x0, src_x1 = max(0, -dx), min(width, width - dx)
            if src_x0 >= src_x1:
                continue
            dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
            valid = z_valid[src_y0:src_y1, src_x0:src_x1]
            z = elevation[src_y0:src_y1, src_x0:src_x1]
            target = np.s_[dst_y0:dst_y1, dst_x0:dst_x1]
            weight = valid.astype(np.float64)
            x, y = dx * resolution, dy * resolution
            count[target] += weight
            sum_x[target] += weight * x
            sum_y[target] += weight * y
            sum_z[target] += np.where(valid, z, 0.0)
            sum_xx[target] += weight * x * x
            sum_xy[target] += weight * x * y
            sum_yy[target] += weight * y * y
            sum_xz[target] += np.where(valid, z * x, 0.0)
            sum_yz[target] += np.where(valid, z * y, 0.0)

    safe_count = np.maximum(count, 1.0)
    cov_xx = sum_xx - sum_x * sum_x / safe_count
    cov_xy = sum_xy - sum_x * sum_y / safe_count
    cov_yy = sum_yy - sum_y * sum_y / safe_count
    cov_xz = sum_xz - sum_x * sum_z / safe_count
    cov_yz = sum_yz - sum_y * sum_z / safe_count
    determinant = cov_xx * cov_yy - cov_xy * cov_xy
    slope_valid = z_valid & (count >= 6) & (determinant > 1e-12)
    gx = np.zeros_like(elevation)
    gy = np.zeros_like(elevation)
    gx[slope_valid] = ((cov_xz * cov_yy - cov_yz * cov_xy)[slope_valid]
                       / determinant[slope_valid])
    gy[slope_valid] = ((cov_yz * cov_xx - cov_xz * cov_xy)[slope_valid]
                       / determinant[slope_valid])
    angle = np.degrees(np.arctan(np.hypot(gx, gy)))
    slope[slope_valid] = np.clip(
        np.rint(angle[slope_valid] / slope_full_degrees * 100), 0, 100).astype(np.int8)

    # Local detrended roughness: center deviation from the mean of known
    # neighboring elevations. A plane has zero residual; ledges and rough
    # surface variation raise the score.
    padded_z = np.pad(np.where(z_valid, elevation, 0.0), 1)
    padded_valid = np.pad(z_valid, 1)
    neighbor_sum = np.zeros_like(elevation)
    neighbor_count = np.zeros(elevation.shape, dtype=np.uint8)
    for dy in range(3):
        for dx in range(3):
            if dx == 1 and dy == 1:
                continue
            neighbor_sum += padded_z[dy:dy + height, dx:dx + width]
            neighbor_count += padded_valid[dy:dy + height, dx:dx + width]
    rough_valid = z_valid & (neighbor_count >= 5)
    residual = np.zeros_like(elevation)
    residual[rough_valid] = np.abs(
        elevation[rough_valid] - neighbor_sum[rough_valid] / neighbor_count[rough_valid])
    roughness[rough_valid] = np.clip(
        np.rint(residual[rough_valid] / roughness_full_m * 100), 0, 100).astype(np.int8)

    # A negative bowl is a crater candidate only if a surrounding annulus has
    # enough height returns. The layer is risk, not a confirmed crater label.
    radius = crater_radius_cells
    pad_z = np.pad(np.where(z_valid, elevation, np.nan), radius,
                   constant_values=np.nan)
    pad_valid = np.pad(z_valid, radius, constant_values=False)
    z_windows = np.lib.stride_tricks.sliding_window_view(
        pad_z, (2 * radius + 1, 2 * radius + 1))
    valid_windows = np.lib.stride_tricks.sliding_window_view(
        pad_valid, (2 * radius + 1, 2 * radius + 1))
    yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    annulus = (np.hypot(xx, yy) >= 1.5) & (np.hypot(xx, yy) <= radius)
    annulus_windows = valid_windows[..., annulus]
    counts = np.count_nonzero(annulus_windows, axis=-1)
    annulus_values = np.where(annulus_windows, z_windows[..., annulus], np.inf)
    annulus_values.sort(axis=-1)
    lower = np.maximum(0, (counts - 1) // 2)
    upper = np.maximum(0, counts // 2)
    rows, cols = np.indices((height, width))
    median_rim = (annulus_values[rows, cols, lower] + annulus_values[rows, cols, upper]) / 2.0
    crater_valid = z_valid & (counts >= 8)
    depression = median_rim - elevation
    crater_score = ((depression - crater_min_depth_m)
                    / (crater_full_depth_m - crater_min_depth_m) * 100.0)
    crater[crater_valid] = np.clip(
        np.rint(crater_score[crater_valid]), 0, 100).astype(np.int8)

    # Also compare against a sparse outer rim. A broad crater can have a flat
    # floor several metres across, so the local 0.3 m neighborhood correctly
    # reports zero at its center. Sampling a 3 m ring lets the center express
    # the depression while bounding the calculation to a few dozen cells per
    # output cell instead of materializing a large square window.
    wide_radius_cells = max(radius + 1, int(round(3.0 / resolution)))
    if wide_radius_cells > radius:
        ring_offsets = sorted({
            (int(round(wide_radius_cells * math.sin(2.0 * math.pi * i / 32))),
             int(round(wide_radius_cells * math.cos(2.0 * math.pi * i / 32))))
            for i in range(32)
        })
        ring_values = np.full((len(ring_offsets), height, width), np.inf,
                              dtype=np.float64)
        ring_count = np.zeros((height, width), dtype=np.uint8)
        for index, (dy, dx) in enumerate(ring_offsets):
            src_y0, src_y1 = max(0, -dy), min(height, height - dy)
            src_x0, src_x1 = max(0, -dx), min(width, width - dx)
            if src_y0 >= src_y1 or src_x0 >= src_x1:
                continue
            dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
            dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
            source_valid = z_valid[src_y0:src_y1, src_x0:src_x1]
            target = ring_values[index, dst_y0:dst_y1, dst_x0:dst_x1]
            target[source_valid] = elevation[src_y0:src_y1, src_x0:src_x1][source_valid]
            ring_count[dst_y0:dst_y1, dst_x0:dst_x1] += source_valid
        ring_values.sort(axis=0)
        low = np.maximum(0, (ring_count.astype(np.int16) - 1) // 2)
        high = np.maximum(0, ring_count.astype(np.int16) // 2)
        rows, cols = np.indices((height, width))
        wide_rim = (ring_values[low, rows, cols] + ring_values[high, rows, cols]) / 2.0
        wide_valid = z_valid & (ring_count >= 8)
        wide_depression = wide_rim - elevation
        wide_score = ((wide_depression - crater_min_depth_m)
                      / (crater_full_depth_m - crater_min_depth_m) * 100.0)
        wide_risk = np.clip(np.rint(wide_score), 0, 100).astype(np.int8)
        current_risk = np.maximum(crater, 0)
        crater[wide_valid] = np.maximum(current_risk[wide_valid], wide_risk[wide_valid])

    semantic_valid = (labels != 0) & (seen >= 0)
    uncertainty = np.full((height, width), -1, dtype=np.int8)
    age = np.maximum(0, int(now_ns) - seen).astype(np.float64) / 1e9
    effective_conf = confidence * np.exp2(-age / confidence_half_life_s)
    confidence_uncertainty = 1.0 - np.clip(effective_conf, 0.0, 1.0)
    age_uncertainty = np.clip(age / observation_ttl_s, 0.0, 1.0)
    geometry_support = ((slope >= 0).astype(np.float64)
                        + (roughness >= 0).astype(np.float64)
                        + (crater >= 0).astype(np.float64)) / 3.0
    geometry_uncertainty = 1.0 - geometry_support
    uncertainty[semantic_valid] = np.rint(
        100 * np.maximum.reduce((confidence_uncertainty, age_uncertainty,
                                 geometry_uncertainty))[semantic_valid]
    ).astype(np.int8)
    return {"slope": slope, "roughness": roughness,
            "crater_risk": crater, "uncertainty": uncertainty}
