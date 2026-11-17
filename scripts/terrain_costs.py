"""Pure alignment checks and terrain-risk fusion for the traversability grid."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GridGeometry:
    frame_id: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float = 0.0
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def compatible(self, other: "GridGeometry", tolerance: float = 1e-6) -> bool:
        left_norm = math.sqrt(sum(value * value for value in self.orientation_xyzw))
        right_norm = math.sqrt(sum(value * value for value in other.orientation_xyzw))
        if left_norm <= 1e-12 or right_norm <= 1e-12:
            return False
        orientation_dot = abs(sum(
            (a / left_norm) * (b / right_norm)
            for a, b in zip(self.orientation_xyzw, other.orientation_xyzw)))
        return (
            self.frame_id == other.frame_id
            and self.width == other.width and self.height == other.height
            and math.isfinite(self.resolution) and math.isfinite(other.resolution)
            and abs(self.resolution - other.resolution) <= tolerance
            and abs(self.origin_x - other.origin_x) <= tolerance
            and abs(self.origin_y - other.origin_y) <= tolerance
            and abs(self.origin_z - other.origin_z) <= tolerance
            and orientation_dot >= 1.0 - tolerance
        )


def fuse_terrain_costs(semantic: list[int] | tuple[int, ...], *,
                       slope: list[int] | tuple[int, ...],
                       roughness: list[int] | tuple[int, ...],
                       crater_risk: list[int] | tuple[int, ...],
                       uncertainty: list[int] | tuple[int, ...],
                       terrain_cost: int = 20, unknown_cost: int = 80,
                       obstacle_cost: int = 100,
                       weights: tuple[float, float, float, float] =
                       (0.25, 0.20, 0.35, 0.20)) -> list[int]:
    """Fuse aligned risk scores; semantic unknown/shadow never becomes obstacle."""
    size = len(semantic)
    layers = (slope, roughness, crater_risk, uncertainty)
    if any(len(layer) != size for layer in layers):
        raise ValueError("semantic and terrain-risk arrays must have equal lengths")
    if (not 0 <= terrain_cost < obstacle_cost <= 100
            or not 0 <= unknown_cost <= 100 or len(weights) != 4
            or any(not math.isfinite(w) or w < 0 for w in weights)
            or sum(weights) <= 0):
        raise ValueError("cost bounds and fusion weights are invalid")
    total_weight = sum(weights)
    normalized = tuple(w / total_weight for w in weights)
    output: list[int] = []
    for index, value in enumerate(semantic):
        value = int(value)
        if value >= 100:
            output.append(int(obstacle_cost))
            continue
        if value <= 0:
            # -1 and 0 (unknown/shadow) are caution, never hard obstacles.
            output.append(int(unknown_cost))
            continue
        risks = [max(0, min(100, int(layer[index]))) for layer in layers]
        risk = sum(weight * risk_value for weight, risk_value in zip(normalized, risks))
        cost = terrain_cost + (obstacle_cost - terrain_cost) * risk / 100.0
        output.append(min(int(obstacle_cost), max(int(terrain_cost), int(round(cost)))))
    return output
