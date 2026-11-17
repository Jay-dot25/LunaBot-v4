"""Incremental D* Lite search over an 8-connected weighted occupancy grid.

This is a benchmark candidate only. The production terrain planner continues
to use weighted A* unless matched measurements justify an adoption.
"""
from __future__ import annotations

import heapq
import math
from collections import defaultdict
from typing import Iterable

Cell = tuple[int, int]


class DStarLite:
    def __init__(self, costs: list[int], width: int, height: int,
                 start: Cell, goal: Cell, *, cost_weight: float = 2.0,
                 blocked_cost: int = 100, unknown_cost: int = 80,
                 unknown_is_blocked: bool = False):
        if width <= 0 or height <= 0 or len(costs) != width * height:
            raise ValueError("cost grid dimensions and data length are invalid")
        if (not math.isfinite(cost_weight) or cost_weight < 0 or
                not 1 <= blocked_cost <= 100 or
                not 0 <= unknown_cost < blocked_cost):
            raise ValueError("planner cost policy is invalid")
        if any(int(value) < -1 or int(value) > 100 for value in costs):
            raise ValueError("cost grid values must be -1..100")
        self.width, self.height = width, height
        self.costs = [int(value) for value in costs]
        self.start, self.last_start, self.goal = start, start, goal
        self.cost_weight = cost_weight
        self.blocked_cost = blocked_cost
        self.unknown_cost = unknown_cost
        self.unknown_is_blocked = unknown_is_blocked
        self.km = 0.0
        self.g: defaultdict[Cell, float] = defaultdict(lambda: math.inf)
        self.rhs: defaultdict[Cell, float] = defaultdict(lambda: math.inf)
        self._queue: list[tuple[float, float, int, Cell, int]] = []
        self._versions: defaultdict[Cell, int] = defaultdict(int)
        self._sequence = 0
        self.expanded = 0
        self.rhs[goal] = 0.0
        self._enqueue(goal)

    def _cell_cost(self, cell: Cell) -> float | None:
        x, y = cell
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return None
        value = self.costs[y * self.width + x]
        if value < 0:
            if self.unknown_is_blocked:
                return None
            value = self.unknown_cost
        if value >= self.blocked_cost:
            return None
        return float(value)

    def _neighbors(self, cell: Cell) -> Iterable[Cell]:
        x, y = cell
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = x + dx, y + dy
            if self._cell_cost(nxt) is None:
                continue
            if dx and dy and (
                    self._cell_cost((x + dx, y)) is None or
                    self._cell_cost((x, y + dy)) is None):
                continue
            yield nxt

    def _edge_cost(self, source: Cell, target: Cell) -> float:
        cost = self._cell_cost(target)
        if cost is None:
            return math.inf
        distance = math.sqrt(2.0) if source[0] != target[0] and source[1] != target[1] else 1.0
        return distance * (1.0 + self.cost_weight * cost / 100.0)

    @staticmethod
    def _heuristic(first: Cell, second: Cell) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _key(self, cell: Cell) -> tuple[float, float]:
        best = min(self.g[cell], self.rhs[cell])
        return (best + self._heuristic(self.start, cell) + self.km, best)

    def _enqueue(self, cell: Cell) -> None:
        self._versions[cell] += 1
        if self.g[cell] != self.rhs[cell]:
            k1, k2 = self._key(cell)
            self._sequence += 1
            heapq.heappush(self._queue,
                           (k1, k2, self._sequence, cell,
                            self._versions[cell]))

    def _discard_stale(self) -> None:
        while self._queue:
            *_, cell, version = self._queue[0]
            if (version == self._versions[cell] and
                    self.g[cell] != self.rhs[cell]):
                return
            heapq.heappop(self._queue)

    def _top_key(self) -> tuple[float, float]:
        self._discard_stale()
        if not self._queue:
            return math.inf, math.inf
        return self._queue[0][0], self._queue[0][1]

    def _pop(self) -> tuple[tuple[float, float], Cell] | None:
        self._discard_stale()
        if not self._queue:
            return None
        k1, k2, _, cell, _ = heapq.heappop(self._queue)
        self._versions[cell] += 1
        return (k1, k2), cell

    def _update_vertex(self, cell: Cell) -> None:
        if cell != self.goal:
            self.rhs[cell] = min(
                (self._edge_cost(cell, nxt) + self.g[nxt]
                 for nxt in self._neighbors(cell)), default=math.inf)
        self._enqueue(cell)

    def compute_shortest_path(self) -> None:
        # The guard catches malformed state instead of spinning indefinitely.
        limit = self.width * self.height * 100
        steps = 0
        while (self._top_key() < self._key(self.start) or
               self.rhs[self.start] != self.g[self.start]):
            if steps >= limit:
                raise RuntimeError("D* Lite search exceeded its safety bound")
            steps += 1
            popped = self._pop()
            if popped is None:
                break
            old_key, cell = popped
            new_key = self._key(cell)
            if old_key < new_key:
                self._enqueue(cell)
            elif self.g[cell] > self.rhs[cell]:
                self.g[cell] = self.rhs[cell]
                self.expanded += 1
                for predecessor in self._neighbors(cell):
                    self._update_vertex(predecessor)
            else:
                self.g[cell] = math.inf
                self._update_vertex(cell)
                for predecessor in self._neighbors(cell):
                    self._update_vertex(predecessor)

    def update_costs(self, costs: list[int], changed_cells: Iterable[Cell]) -> None:
        if len(costs) != self.width * self.height:
            raise ValueError("updated cost grid dimensions do not match")
        if any(int(value) < -1 or int(value) > 100 for value in costs):
            raise ValueError("cost grid values must be -1..100")
        old_costs = self.costs
        self.costs = [int(value) for value in costs]
        affected: set[Cell] = set()
        for x, y in changed_cells:
            for cy in range(y - 1, y + 2):
                for cx in range(x - 1, x + 2):
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        affected.add((cx, cy))
        for cell in sorted(affected):
            self._update_vertex(cell)
        if not affected and self.costs != old_costs:
            raise ValueError("changed_cells must include changed map cells")

    def move_start(self, start: Cell) -> None:
        self.km += self._heuristic(self.last_start, start)
        self.last_start = self.start = start

    def extract_path(self) -> tuple[list[Cell], float]:
        self.compute_shortest_path()
        if self._cell_cost(self.start) is None or not math.isfinite(self.g[self.start]):
            return [], math.inf
        path = [self.start]
        seen = {self.start}
        current = self.start
        total = 0.0
        for _ in range(self.width * self.height):
            if current == self.goal:
                return path, total
            options = [
                (self._edge_cost(current, nxt) + self.g[nxt], nxt)
                for nxt in self._neighbors(current)
            ]
            if not options:
                return [], math.inf
            value, nxt = min(options, key=lambda row: (row[0], row[1]))
            if not math.isfinite(value) or nxt in seen:
                return [], math.inf
            total += self._edge_cost(current, nxt)
            path.append(nxt)
            seen.add(nxt)
            current = nxt
        return [], math.inf
