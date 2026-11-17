"""Revision hashing and occupancy-constrained weighted A* helpers."""
from __future__ import annotations

import hashlib
import heapq
import json
import math


def compute_map_revision(*, frame_id: str, width: int, height: int,
                         resolution: float, origin: tuple[float, ...],
                         data: list[int] | tuple[int, ...]) -> str:
    """Hash cost-grid geometry and contents; timestamps do not create revisions."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("cost map dimensions and data length are invalid")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("cost map resolution must be finite and positive")
    if len(origin) != 7 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError("map origin must contain seven finite pose values")
    if math.sqrt(sum(float(value) ** 2 for value in origin[3:])) < 1e-12:
        raise ValueError("map origin orientation quaternion is invalid")
    if any(int(value) < -1 or int(value) > 100 for value in data):
        raise ValueError("cost map values must be -1..100")
    quaternion = [float(value) for value in origin[3:]]
    norm = math.sqrt(sum(value * value for value in quaternion))
    quaternion = [value / norm for value in quaternion]
    first_nonzero = next((value for value in quaternion if abs(value) > 1e-12), 1.0)
    if first_nonzero < 0:
        quaternion = [-value for value in quaternion]
    quaternion = [0.0 if abs(value) < 1e-15 else value for value in quaternion]
    position = [0.0 if abs(float(value)) < 1e-15 else float(value)
                for value in origin[:3]]
    canonical_origin = position + quaternion
    header = json.dumps(
        [frame_id, int(width), int(height), float(resolution),
         canonical_origin],
        separators=(",", ":"), ensure_ascii=True).encode("ascii")
    digest = hashlib.sha256()
    digest.update(header)
    digest.update(bytes(int(value) & 0xFF for value in data))
    return digest.hexdigest()


def world_to_grid(x: float, y: float, *, origin_x: float, origin_y: float,
                  origin_yaw: float, resolution: float) -> tuple[int, int]:
    if not all(math.isfinite(value) for value in
               (x, y, origin_x, origin_y, origin_yaw, resolution)) or resolution <= 0:
        raise ValueError("world/grid geometry is invalid")
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    return (int(math.floor((c * dx + s * dy) / resolution)),
            int(math.floor((-s * dx + c * dy) / resolution)))


def grid_to_world(cell: tuple[int, int], *, origin_x: float, origin_y: float,
                  origin_yaw: float, resolution: float) -> tuple[float, float]:
    if not all(math.isfinite(value) for value in
               (origin_x, origin_y, origin_yaw, resolution)) or resolution <= 0:
        raise ValueError("world/grid geometry is invalid")
    mx, my = (cell[0] + 0.5) * resolution, (cell[1] + 0.5) * resolution
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    return origin_x + c * mx - s * my, origin_y + s * mx + c * my


def weighted_astar(costs: list[int] | tuple[int, ...], width: int, height: int,
                   start: tuple[int, int], goal: tuple[int, int], *,
                   cost_weight: float = 2.0, blocked_cost: int = 100,
                   unknown_cost: int = 80,
                   unknown_is_blocked: bool = False) -> tuple[list[tuple[int, int]], float]:
    """Return a minimum weighted-cost 8-connected path without corner cutting."""
    if width <= 0 or height <= 0 or len(costs) != width * height:
        raise ValueError("cost grid dimensions and data length are invalid")
    if (not math.isfinite(cost_weight) or cost_weight < 0
            or not 1 <= blocked_cost <= 100 or not 0 <= unknown_cost < blocked_cost):
        raise ValueError("planner cost policy is invalid")
    if any(int(value) < -1 or int(value) > 100 for value in costs):
        raise ValueError("cost grid values must be -1..100")

    def cell_cost(cell: tuple[int, int]) -> int | None:
        x, y = cell
        if x < 0 or y < 0 or x >= width or y >= height:
            return None
        value = int(costs[y * width + x])
        if value < 0:
            return None if unknown_is_blocked else unknown_cost
        if value >= blocked_cost:
            return None
        return value

    if cell_cost(start) is None or cell_cost(goal) is None:
        return [], math.inf

    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                 (1, 1), (1, -1), (-1, 1), (-1, -1))
    scores = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    open_set = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), start)]
    while open_set:
        _, current = heapq.heappop(open_set)
        current_score = scores.get(current)
        if current_score is None:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path)), scores[goal]
        for dx, dy in neighbors:
            nxt = current[0] + dx, current[1] + dy
            next_cost = cell_cost(nxt)
            if next_cost is None:
                continue
            if dx and dy:
                # A diagonal step is valid only when both adjacent orthogonal
                # cells are traversable; this prevents squeezing through walls.
                if (cell_cost((current[0] + dx, current[1])) is None
                        or cell_cost((current[0], current[1] + dy)) is None):
                    continue
            distance = math.sqrt(2.0) if dx and dy else 1.0
            step = distance * (1.0 + cost_weight * next_cost / 100.0)
            tentative = current_score + step
            if tentative >= scores.get(nxt, math.inf):
                continue
            came_from[nxt] = current
            scores[nxt] = tentative
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(open_set, (tentative + heuristic, nxt))
    return [], math.inf


def path_is_traversable(costs: list[int] | tuple[int, ...], width: int,
                        height: int, path: list[tuple[int, int]] | tuple[tuple[int, int], ...],
                        *, blocked_cost: int = 100,
                        unknown_is_blocked: bool = False) -> bool:
    """Check a prior path against a changed cost grid, including diagonal corners."""
    if width <= 0 or height <= 0 or len(costs) != width * height:
        raise ValueError("cost grid dimensions and data length are invalid")
    if not 1 <= blocked_cost <= 100:
        raise ValueError("blocked_cost must be in 1..100")
    if any(int(value) < -1 or int(value) > 100 for value in costs):
        raise ValueError("cost map values must be -1..100")

    def clear(cell: tuple[int, int]) -> bool:
        x, y = cell
        if x < 0 or y < 0 or x >= width or y >= height:
            return False
        value = int(costs[y * width + x])
        return not (value < 0 and unknown_is_blocked) and value < blocked_cost

    if not path or any(not clear(cell) for cell in path):
        return False
    for first, second in zip(path, path[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        if abs(dx) > 1 or abs(dy) > 1:
            return False
        if dx and dy and (not clear((first[0] + dx, first[1]))
                          or not clear((first[0], first[1] + dy))):
            return False
    return True


def merge_obstacle_grid(costs: list[int] | tuple[int, ...], width: int,
                        height: int, *, origin_x: float, origin_y: float,
                        origin_yaw: float, resolution: float,
                        obstacles: list[int] | tuple[int, ...],
                        obstacle_width: int, obstacle_height: int,
                        obstacle_origin_x: float, obstacle_origin_y: float,
                        obstacle_origin_yaw: float, obstacle_resolution: float,
                        obstacle_threshold: int = 65,
                        inflation_radius: float = 0.35) -> list[int]:
    """Overlay detected obstacles on a terrain grid with conservative inflation."""
    if (width <= 0 or height <= 0 or len(costs) != width * height
            or obstacle_width <= 0 or obstacle_height <= 0
            or len(obstacles) != obstacle_width * obstacle_height):
        raise ValueError("terrain and obstacle grid dimensions are invalid")
    if (not 1 <= obstacle_threshold <= 100 or not math.isfinite(inflation_radius)
            or inflation_radius < 0):
        raise ValueError("obstacle threshold or inflation radius is invalid")
    for value in costs:
        if int(value) < -1 or int(value) > 100:
            raise ValueError("terrain costs must be -1..100")
    for value in obstacles:
        if int(value) < -1 or int(value) > 100:
            raise ValueError("obstacle costs must be -1..100")
    if (not math.isfinite(resolution) or resolution <= 0
            or not math.isfinite(obstacle_resolution) or obstacle_resolution <= 0):
        raise ValueError("grid resolutions must be finite and positive")

    merged = [int(value) for value in costs]
    margin = (inflation_radius + math.sqrt(2.0) *
              (resolution + obstacle_resolution) / 2.0)
    radius = int(math.ceil(margin / resolution))
    margin_sq = margin * margin
    for obstacle_y in range(obstacle_height):
        for obstacle_x in range(obstacle_width):
            if int(obstacles[obstacle_y * obstacle_width + obstacle_x]) < obstacle_threshold:
                continue
            world_x, world_y = grid_to_world(
                (obstacle_x, obstacle_y), origin_x=obstacle_origin_x,
                origin_y=obstacle_origin_y, origin_yaw=obstacle_origin_yaw,
                resolution=obstacle_resolution)
            center_x, center_y = world_to_grid(
                world_x, world_y, origin_x=origin_x, origin_y=origin_y,
                origin_yaw=origin_yaw, resolution=resolution)
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if (dx * dx + dy * dy) * resolution * resolution > margin_sq:
                        continue
                    x, y = center_x + dx, center_y + dy
                    if 0 <= x < width and 0 <= y < height:
                        merged[y * width + x] = 100
    return merged
