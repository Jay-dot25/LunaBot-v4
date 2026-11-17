"""Bounded, confidence- and age-aware semantic map layers."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


UNKNOWN = 0
TERRAIN = 1
OBSTACLE = 2


def three_class_to_map_labels(labels: np.ndarray) -> np.ndarray:
    """Map P5-02 IDs to the traversability grid's terrain/obstacle vocabulary.

    Regolith is traversable terrain, rock is an obstacle, and shadow/unknown
    remain unobserved because appearance alone cannot establish traversability.
    """
    source = np.asarray(labels)
    result = np.zeros(source.shape, dtype=np.uint8)
    result[source == 0] = TERRAIN
    result[source == 1] = OBSTACLE
    return result


def decision_strength(*, roi_margin: float, appearance_margin: float) -> float:
    """Return a conservative heuristic confidence for the rule segmenter.

    This is a decision margin, not a calibrated probability. Keeping a nonzero
    floor for known labels lets the mapper separately apply its confidence gate.
    """
    values = (float(roi_margin), float(appearance_margin))
    if not all(math.isfinite(value) for value in values):
        return 0.0
    margin = min(1.0, max(0.0, max(abs(values[0]), abs(values[1]))))
    return 0.55 + 0.45 * margin


@dataclass(frozen=True)
class GridUpdate:
    accepted: int
    rejected_low_confidence: int
    cleared_stale: int


class SemanticGrid:
    """Fixed-size rolling semantic grid with confidence decay and TTL clearing."""

    def __init__(self, width: int, height: int, resolution: float,
                 origin_x: float, origin_y: float, *,
                 half_life_s: float = 10.0, ttl_s: float = 30.0,
                 min_confidence: float = 0.35) -> None:
        if width < 1 or height < 1 or resolution <= 0:
            raise ValueError("grid dimensions and resolution must be positive")
        if half_life_s <= 0 or ttl_s <= 0 or not 0 <= min_confidence <= 1:
            raise ValueError("invalid confidence or aging policy")
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin_x), float(origin_y)
        self.half_life_ns = int(half_life_s * 1e9)
        self.ttl_ns = int(ttl_s * 1e9)
        self.min_confidence = float(min_confidence)
        self.labels = np.zeros((height, width), dtype=np.uint8)
        self.confidence = np.zeros((height, width), dtype=np.float32)
        self.last_seen_ns = np.full((height, width), -1, dtype=np.int64)
        self.elevation = np.full((height, width), np.nan, dtype=np.float32)
        self.elevation_seen_ns = np.full((height, width), -1, dtype=np.int64)

    @property
    def memory_bytes(self) -> int:
        return int(self.labels.nbytes + self.confidence.nbytes + self.last_seen_ns.nbytes
                   + self.elevation.nbytes + self.elevation_seen_ns.nbytes)

    def recenter(self, x: float, y: float) -> int:
        """Move the bounded window around a map-frame point, preserving overlap."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("recenter point must be finite")
        span_x, span_y = self.width * self.resolution, self.height * self.resolution
        new_x = math.floor((x - span_x / 2) / self.resolution) * self.resolution
        new_y = math.floor((y - span_y / 2) / self.resolution) * self.resolution
        shift_x = int(round((self.origin_x - new_x) / self.resolution))
        shift_y = int(round((self.origin_y - new_y) / self.resolution))
        if shift_x == 0 and shift_y == 0:
            return 0
        labels = np.zeros_like(self.labels)
        confidence = np.zeros_like(self.confidence)
        seen = np.full_like(self.last_seen_ns, -1)
        elevation = np.full_like(self.elevation, np.nan)
        elevation_seen = np.full_like(self.elevation_seen_ns, -1)
        src_x0, src_y0 = max(0, -shift_x), max(0, -shift_y)
        dst_x0, dst_y0 = max(0, shift_x), max(0, shift_y)
        copy_w = max(0, min(self.width - src_x0, self.width - dst_x0))
        copy_h = max(0, min(self.height - src_y0, self.height - dst_y0))
        if copy_w > 0 and copy_h > 0:
            src = np.s_[src_y0:src_y0 + copy_h, src_x0:src_x0 + copy_w]
            dst = np.s_[dst_y0:dst_y0 + copy_h, dst_x0:dst_x0 + copy_w]
            labels[dst] = self.labels[src]
            confidence[dst] = self.confidence[src]
            seen[dst] = self.last_seen_ns[src]
            elevation[dst] = self.elevation[src]
            elevation_seen[dst] = self.elevation_seen_ns[src]
        self.labels, self.confidence, self.last_seen_ns = labels, confidence, seen
        self.elevation, self.elevation_seen_ns = elevation, elevation_seen
        self.origin_x, self.origin_y = new_x, new_y
        return self.width * self.height - copy_w * copy_h

    def _decay_in_place(self, now_ns: int) -> int:
        observed = self.last_seen_ns >= 0
        age = np.maximum(0, int(now_ns) - self.last_seen_ns)
        effective = self.confidence * np.exp2(-age / self.half_life_ns)
        clear = observed & ((age >= self.ttl_ns) | (effective < self.min_confidence))
        count = int(np.count_nonzero(clear))
        self.labels[clear] = UNKNOWN
        self.confidence[clear] = 0.0
        self.last_seen_ns[clear] = -1
        stale_z = ((self.elevation_seen_ns >= 0)
                   & ((int(now_ns) - self.elevation_seen_ns) >= self.ttl_ns))
        self.elevation[stale_z] = np.nan
        self.elevation_seen_ns[stale_z] = -1
        return count

    def decay(self, now_ns: int) -> int:
        return self._decay_in_place(now_ns)

    def update(self, observations: dict[int, tuple[int, float]], now_ns: int,
               elevations: dict[int, float] | None = None) -> GridUpdate:
        cleared = self._decay_in_place(now_ns)
        accepted = rejected = 0
        for flat_index, pair in observations.items():
            label, raw_confidence = pair
            confidence = float(raw_confidence)
            if label not in (TERRAIN, OBSTACLE) or not math.isfinite(confidence):
                continue
            confidence = min(1.0, max(0.0, confidence))
            if confidence < self.min_confidence:
                rejected += 1
                continue
            if not 0 <= int(flat_index) < self.width * self.height:
                continue
            row, col = divmod(int(flat_index), self.width)
            old_label = int(self.labels[row, col])
            old_conf = float(self.confidence[row, col])
            if old_label == UNKNOWN or old_label == label or confidence >= old_conf:
                if old_label == label:
                    confidence = 1.0 - (1.0 - old_conf) * (1.0 - confidence)
                self.labels[row, col] = label
                self.confidence[row, col] = min(1.0, confidence)
                self.last_seen_ns[row, col] = int(now_ns)
                if elevations is not None and int(flat_index) in elevations:
                    z = float(elevations[int(flat_index)])
                    if math.isfinite(z):
                        self.elevation[row, col] = z
                        self.elevation_seen_ns[row, col] = int(now_ns)
                accepted += 1
        return GridUpdate(accepted, rejected, cleared)

    def semantic_data(self, now_ns: int) -> list[int]:
        self._decay_in_place(now_ns)
        values = np.full(self.labels.shape, -1, dtype=np.int8)
        values[self.labels == TERRAIN] = 25
        values[self.labels == OBSTACLE] = 100
        return values.ravel().tolist()

    def confidence_data(self, now_ns: int) -> list[int]:
        self._decay_in_place(now_ns)
        age = np.maximum(0, int(now_ns) - self.last_seen_ns)
        effective = self.confidence * np.exp2(-age / self.half_life_ns)
        values = np.rint(effective * 100).astype(np.int8)
        values[self.labels == UNKNOWN] = -1
        return values.ravel().tolist()

    def age_data(self, now_ns: int) -> list[int]:
        self._decay_in_place(now_ns)
        observed = self.last_seen_ns >= 0
        age_s = np.maximum(0, int(now_ns) - self.last_seen_ns) / 1e9
        values = np.full(self.labels.shape, -1, dtype=np.int8)
        values[observed] = np.minimum(100, np.floor(age_s[observed])).astype(np.int8)
        return values.ravel().tolist()

    def counts(self, now_ns: int) -> tuple[int, int, int]:
        self._decay_in_place(now_ns)
        return (int(np.count_nonzero(self.labels == TERRAIN)),
                int(np.count_nonzero(self.labels == OBSTACLE)),
                int(np.count_nonzero(self.labels == UNKNOWN)))
