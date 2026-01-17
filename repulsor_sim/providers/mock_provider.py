# repulsor_sim/providers/mock_provider.py
from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d

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

        self.piece_count = 0
        self._picked_up: Dict[str, _FuelObj] = {}
        self._nt_inst = None
        self._piece_pub = None
        self._piece_sub = None

        self._forbidden: List[Tuple[float, float, float, float]] = []

    def reset(self, cfg: Config) -> None:
        self.cfg = cfg
        self.rng = random.Random()

        self.cx = cfg.field_length_m * 0.5
        self.cy = cfg.field_width_m * 0.5

        self.x0 = 0.0
        self.x1 = float(cfg.field_length_m)
        self.y0 = 0.0
        self.y1 = float(cfg.field_width_m)

        self.nx = int(math.floor((self.x1 - self.x0) / cfg.grid_cell_m))
        self.ny = int(math.floor((self.y1 - self.y0) / cfg.grid_cell_m))

        self._forbidden = self._build_forbidden_regions()

        self.cluster_centers = self._make_cluster_centers()
        self._regen_fuel()
        self._init_robots()

        self._nt_inst = NetworkTableInstance.getDefault()
        self._piece_pub = self._nt_inst.getIntegerTopic("/PieceCount").publish()
        self._piece_sub = self._nt_inst.getIntegerTopic("/PieceCount").subscribe(0)

        self.piece_count = 0
        self.last_piece_count = 0
        self._picked_up.clear()
        self._piece_pub.set(0)

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
    
    def _build_forbidden_regions(self) -> List[Tuple[float, float, float, float]]:
        assert self.cfg is not None
        L = float(self.cfg.field_length_m)
        W = float(self.cfg.field_width_m)
        m = float(max(0.0, self.cfg.fuel_exclude_margin_m))

        out: List[Tuple[float, float, float, float]] = []

        s = 1.1938
        hs = 0.5 * s + m

        sx1 = 4.625594
        sy1 = W * 0.5
        out.append((sx1 - hs, sx1 + hs, sy1 - hs, sy1 + hs))

        sx2 = L - 4.625594
        sy2 = W * 0.5
        out.append((sx2 - hs, sx2 + hs, sy2 - hs, sy2 + hs))

        rw = 1.1938
        rh = 0.255336
        hrw = 0.5 * rw + m
        hrh = 0.5 * rh + m

        rxL = (L * 0.5) - 3.63982
        rxR = (L * 0.5) + 3.63982

        ryB = 1.4224
        ryT = W - 1.4224

        out.append((rxL - hrw, rxL + hrw, ryB - hrh, ryB + hrh))
        out.append((rxL - hrw, rxL + hrw, ryT - hrh, ryT + hrh))
        out.append((rxR - hrw, rxR + hrw, ryB - hrh, ryB + hrh))
        out.append((rxR - hrw, rxR + hrw, ryT - hrh, ryT + hrh))

        return out

    def _in_forbidden(self, x: float, y: float) -> bool:
        for x0, x1, y0, y1 in self._forbidden:
            if x >= x0 and x <= x1 and y >= y0 and y <= y1:
                return True
        return False

    def _pick_counts_per_cell(self) -> Dict[Tuple[int, int], int]:
        assert self.cfg is not None
        assert self.rng is not None

        scores = []
        for ix in range(self.nx + 1):
            for iy in range(self.ny + 1):
                x, y = self._cell_center(ix, iy)
                if self._in_forbidden(x, y):
                    continue
                sc = self._score_cell(x, y)
                scores.append((sc, ix, iy))
        scores.sort(reverse=True)

        budget = min(self.cfg.max_objects, 1200)
        out: Dict[Tuple[int, int], int] = {}

        top = scores[: max(1, len(scores) // 6)]
        mid = scores[len(scores) // 6 : len(scores) // 2]
        low = scores[len(scores) // 2 :]

        def alloc(bucket, lo, hi, frac):
            assert self.cfg is not None
            assert self.rng is not None
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
        self._picked_up.clear()
        counts = self._pick_counts_per_cell()
        oid_num = 0
        cell = float(self.cfg.grid_cell_m)
        sigma = cell * 0.20
        span = cell * 0.45

        for (ix, iy), n in counts.items():
            cx, cy = self._cell_center(ix, iy)
            for _ in range(n):
                placed = False
                x = cx
                y = cy

                for _k in range(24):
                    dx = _box_muller(self.rng) * sigma
                    dy = _box_muller(self.rng) * sigma
                    x = _clamp(cx + dx, self.x0, self.x1)
                    y = _clamp(cy + dy, self.y0, self.y1)
                    if not self._in_forbidden(x, y):
                        placed = True
                        break

                if not placed:
                    for _k in range(32):
                        x = _clamp(cx + self.rng.uniform(-span, span), self.x0, self.x1)
                        y = _clamp(cy + self.rng.uniform(-span, span), self.y0, self.y1)
                        if not self._in_forbidden(x, y):
                            placed = True
                            break

                if not placed:
                    continue

                oid = f"fuel_{oid_num}"
                oid_num += 1
                self.fuel[oid] = _FuelObj(oid=oid, x=x, y=y, z=self.cfg.fuel_z_m)

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

    def _pickup_radius2(self) -> float:
        assert self.cfg is not None
        r = max(0.25, max(0.705, 0.730) * 0.5)
        return r * r

    def _maybe_respawn_on_reset(self) -> None:
        assert self.cfg is not None
        assert self.rng is not None
        ext = int(self._piece_sub.get())
        if self.piece_count > 0 and ext == 0:
            if self._picked_up:
                min_d2 = (max(self.cfg.grid_cell_m, 0.6) * 3.0) ** 2
                for oid, fo in list(self._picked_up.items()):
                    x = fo.x
                    y = fo.y
                    for _ in range(40):
                        nx = self.rng.uniform(self.x0, self.x1)
                        ny = self.rng.uniform(self.y0, self.y1)
                        dx = nx - fo.x
                        dy = ny - fo.y
                        if dx * dx + dy * dy >= min_d2:
                            x, y = nx, ny
                            break
                    self.fuel[oid] = _FuelObj(oid=oid, x=x, y=y, z=fo.z)
                self._picked_up.clear()
            self.piece_count = 0

    def _pickup_half_extents(self) -> tuple[float, float]:
        assert self.cfg is not None
        hx = float(getattr(self.cfg, "pickup_half_extent_x_m", 0.705 / 2))
        hy = float(getattr(self.cfg, "pickup_half_extent_y_m", 0.73 / 2))
        hx = max(0.01, hx)
        hy = max(0.01, hy)
        return hx, hy

    def _maybe_pickup(self, pose: Pose2d) -> None:
        assert self.cfg is not None
        if not self.fuel:
            return

        px = float(pose.x)
        py = float(pose.y)
        hx, hy = self._pickup_half_extents()

        to_remove: List[str] = []
        for oid, fo in self.fuel.items():
            if abs(fo.x - px) <= hx and abs(fo.y - py) <= hy:
                to_remove.append(oid)

        if not to_remove:
            return

        for oid in to_remove:
            fo = self.fuel.pop(oid, None)
            if fo is None:
                continue
            self._picked_up[oid] = fo
            self.piece_count += 1

    def step(self, now_s: float, pose: Pose2d) -> ProviderFrame:
        assert self.cfg is not None

        self._maybe_respawn_on_reset()
        if pose is not None:
            self._maybe_pickup(pose)

        if (self.piece_count != self.last_piece_count):
            self.last_piece_count = self.piece_count
            self._piece_pub.set(int(self.piece_count))

        objs: List[WorldObject] = []
        for fo in self.fuel.values():
            objs.append(WorldObject(oid=fo.oid, class_id=CLASS_FUEL, x=fo.x, y=fo.y, z=fo.z))

        if len(objs) > self.cfg.max_objects:
            objs = objs[: self.cfg.max_objects]

        obstacles: List[VisionObstacle] = []
        for ro in self.robots[: self.cfg.max_obstacles]:
            obstacles.append(VisionObstacle(oid=ro.oid, kind="robot", x=ro.x, y=ro.y, sx=ro.sx, sy=ro.sy))

        return ProviderFrame(objects=objs, obstacles=obstacles, extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
