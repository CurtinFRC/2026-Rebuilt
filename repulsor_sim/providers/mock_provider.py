# repulsor_sim/providers/mock_provider.py
from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

from repulsor_sim.config import Config
from repulsor_sim.types import (
    CLASS_FUEL,
    CLASS_ROBOT_BLUE,
    CLASS_ROBOT_RED,
    ProviderFrame,
    VisionObstacle,
    WorldObject,
)
from repulsor_sim.providers.base import RepulsorProvider

def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def _box_muller(rng: random.Random) -> float:
    u1 = max(1e-12, rng.random())
    u2 = rng.random()
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

@dataclass
class _FuelObj:
    oid: str
    x: float
    y: float
    z: float

@dataclass
class _RobotObj:
    oid: str
    class_id: int
    x: float
    y: float
    sx: float
    sy: float

class MockProvider(RepulsorProvider):
    def __init__(self):
        self.cfg: Config | None = None
        self.rng: random.Random | None = None
        self.fuel: Dict[str, _FuelObj] = {}
        self.robots: List[_RobotObj] = []
        self.cluster_centers: List[Tuple[float, float, float]] = []
        self.cx = 0.0
        self.cy = 0.0
        self.x0 = 0.0
        self.x1 = 0.0
        self.y0 = 0.0
        self.y1 = 0.0
        self.nx = 0
        self.ny = 0
        self._frozen = False

    def reset(self, cfg: Config) -> None:
        self.cfg = cfg
        self.rng = random.Random()

        self.cx = cfg.field_length_m * 0.5
        self.cy = cfg.field_width_m * 0.5
        self.x0 = self.cx - cfg.grid_region_half_x_m
        self.x1 = self.cx + cfg.grid_region_half_x_m
        self.y0 = self.cy - cfg.grid_region_half_y_m
        self.y1 = self.cy + cfg.grid_region_half_y_m

        self.nx = int(math.floor((self.x1 - self.x0) / cfg.grid_cell_m))
        self.ny = int(math.floor((self.y1 - self.y0) / cfg.grid_cell_m))

        self.cluster_centers = self._make_cluster_centers()
        self._regen_fuel()
        self._init_robots()
        self._frozen = True

    def _make_cluster_centers(self) -> List[Tuple[float, float, float]]:
        assert self.rng is not None
        out = []
        for _ in range(6):
            x = self.rng.uniform(self.x0, self.x1)
            y = self.rng.uniform(self.y0, self.y1)
            w = self.rng.uniform(0.6, 1.8)
            out.append((x, y, w))
        return out

    def _cell_center(self, ix: int, iy: int) -> Tuple[float, float]:
        assert self.cfg is not None
        x = self.x0 + ix * self.cfg.grid_cell_m + self.cfg.grid_cell_m * 0.5
        y = self.y0 + iy * self.cfg.grid_cell_m + self.cfg.grid_cell_m * 0.5
        return x, y

    def _score_cell(self, x: float, y: float) -> float:
        s = 0.0
        for cx, cy, w in self.cluster_centers:
            dx = x - cx
            dy = y - cy
            d2 = dx * dx + dy * dy
            s += math.exp(-d2 / (2.0 * (w * w)))
        return s

    def _pick_counts_per_cell(self) -> Dict[Tuple[int, int], int]:
        assert self.cfg is not None
        assert self.rng is not None

        scores = []
        for ix in range(self.nx + 1):
            for iy in range(self.ny + 1):
                x, y = self._cell_center(ix, iy)
                sc = self._score_cell(x, y)
                scores.append((sc, ix, iy))
        scores.sort(reverse=True)

        budget = min(self.cfg.max_objects, 1200)
        out: Dict[Tuple[int, int], int] = {}

        top = scores[: max(1, len(scores) // 6)]
        mid = scores[len(scores) // 6 : len(scores) // 2]
        low = scores[len(scores) // 2 :]

        def alloc(bucket, lo, hi, frac):
            nonlocal budget
            want = int(min(self.cfg.max_objects, 300) * frac)
            want = min(want, budget)
            if want <= 0 or not bucket:
                return
            for _ in range(want):
                _, ix, iy = bucket[self.rng.randrange(0, len(bucket))]
                n = out.get((ix, iy), 0)
                if n >= hi:
                    continue
                out[(ix, iy)] = lo if n < lo else n + 1
                budget -= 1 if n >= lo else lo
                if budget <= 0:
                    break

        alloc(top, 4, 14, 0.60)
        if budget > 0:
            alloc(mid, 1, 8, 0.30)
        if budget > 0:
            alloc(low, 0, 3, 0.10)
        return out

    def _regen_fuel(self) -> None:
        assert self.cfg is not None
        assert self.rng is not None

        self.fuel.clear()
        counts = self._pick_counts_per_cell()
        oid_num = 0
        for (ix, iy), n in counts.items():
            cx, cy = self._cell_center(ix, iy)
            for _ in range(n):
                dx = _box_muller(self.rng) * (self.cfg.grid_cell_m * 0.20)
                dy = _box_muller(self.rng) * (self.cfg.grid_cell_m * 0.20)
                x = _clamp(cx + dx, self.x0, self.x1)
                y = _clamp(cy + dy, self.y0, self.y1)
                oid = f"fuel_{oid_num}"
                oid_num += 1
                self.fuel[oid] = _FuelObj(oid=oid, x=x, y=y, z=self.cfg.fuel_z_m)

    def _init_robots(self) -> None:
        assert self.cfg is not None
        assert self.rng is not None

        self.robots = []
        for i in range(min(self.cfg.max_obstacles, 24)):
            x = self.rng.uniform(self.cfg.field_length_m * 0.25, self.cfg.field_length_m * 0.75)
            y = self.rng.uniform(self.cfg.field_width_m * 0.20, self.cfg.field_width_m * 0.80)
            sx = self.rng.uniform(0.55, 0.95)
            sy = self.rng.uniform(0.55, 0.95)
            class_id = CLASS_ROBOT_BLUE if (i % 2 == 0) else CLASS_ROBOT_RED
            oid = f"robot_{'b' if class_id == CLASS_ROBOT_BLUE else 'r'}_{i}"
            self.robots.append(_RobotObj(oid=oid, class_id=class_id, x=x, y=y, sx=sx, sy=sy))

    def step(self, now_s: float) -> ProviderFrame:
        assert self.cfg is not None

        objs: List[WorldObject] = []
        for fo in self.fuel.values():
            objs.append(WorldObject(oid=fo.oid, class_id=CLASS_FUEL, x=fo.x, y=fo.y, z=fo.z))

        if len(objs) > self.cfg.max_objects:
            objs = objs[: self.cfg.max_objects]

        obstacles: List[VisionObstacle] = []
        for ro in self.robots[: self.cfg.max_obstacles]:
            obstacles.append(VisionObstacle(oid=ro.oid, kind="robot", x=ro.x, y=ro.y, sx=ro.sx, sy=ro.sy))

        return ProviderFrame(objects=objs, obstacles=obstacles, extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
