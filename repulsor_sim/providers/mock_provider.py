# repulsor_sim/providers/mock_provider.py
from __future__ import annotations
import math
import os
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

def _wrap_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0

@dataclass
class _Camera:
    name: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    hfov_rad: float
    vfov_rad: float
    min_range: float
    max_range: float
    update_hz: float
    noise_xy: float
    noise_z: float
    dropout: float
    next_update_s: float = 0.0

@dataclass
class _Detected:
    oid: str
    class_id: int
    x: float
    y: float
    z: float
    score: float

@dataclass
class _TrackedWorld:
    obj: WorldObject
    last_seen_s: float
    last_update_s: float

@dataclass
class _TrackedObstacle:
    obs: VisionObstacle
    last_seen_s: float
    last_update_s: float

@dataclass(frozen=True)
class CameraConfig:
    # ROBOT-RELATIVE camera pose (meters, degrees).
    name: str
    x: float
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float = 0.0
    hfov_deg: float = 95.0
    vfov_deg: float = 60.0
    min_range: float = 0.2
    max_range: float = 9.0
    update_hz: float = 15.0
    noise_xy: float = 0.05
    noise_z: float = 0.03
    dropout: float = 0.04

CAMERAS: List[CameraConfig] = [ # 0.705, 0.730
    CameraConfig(name="cam_front", x=0.35, y=0.00, z=1.35, yaw_deg=0.0,   pitch_deg=-12.0),
    CameraConfig(name="cam_back",  x=-0.35,y=0.00, z=1.35, yaw_deg=180.0, pitch_deg=-12.0),
    CameraConfig(name="cam_left",  x=0.00, y=0.36, z=1.35, yaw_deg=90.0,  pitch_deg=-12.0),
    CameraConfig(name="cam_right", x=0.00, y=-0.36,z=1.35, yaw_deg=-90.0, pitch_deg=-12.0),
]

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
        self._cams: List[_Camera] = []
        self._tracked_objects: Dict[str, _TrackedWorld] = {}
        self._tracked_obstacles: Dict[str, _TrackedObstacle] = {}
        self._obj_publish_hz = 10.0
        self._obs_publish_hz = 10.0
        self._obj_publish_period_s = 0.1
        self._obs_publish_period_s = 0.1
        self._visibility_timeout_s = 0.45
        self.last_piece_count = 0

    def reset(self, cfg: Config) -> None:
        self.cfg = cfg
        seed = int(getattr(cfg, "seed", 1337))
        self.rng = random.Random(seed)

        self.cx = cfg.field_length_m * 0.5
        self.cy = cfg.field_width_m * 0.5

        self.x0 = 0.0
        self.x1 = float(cfg.field_length_m)
        self.y0 = 0.0
        self.y1 = float(cfg.field_width_m)

        self.nx = int(math.floor((self.x1 - self.x0) / cfg.grid_cell_m))
        self.ny = int(math.floor((self.y1 - self.y0) / cfg.grid_cell_m))

        self._forbidden = self._build_forbidden_regions()
        self._cams = self._build_cameras()
        self._tracked_objects.clear()
        self._tracked_obstacles.clear()
        self._obj_publish_hz = float(os.getenv("MOCK_VISION_OBJ_HZ", "10"))
        self._obs_publish_hz = float(os.getenv("MOCK_VISION_OBS_HZ", "10"))
        self._obj_publish_period_s = 1.0 / max(1e-6, self._obj_publish_hz)
        self._obs_publish_period_s = 1.0 / max(1e-6, self._obs_publish_hz)
        self._visibility_timeout_s = float(os.getenv("MOCK_VISION_TIMEOUT_S", "0.45"))

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

    def _build_cameras(self) -> List[_Camera]:
        assert self.cfg is not None
        cams: List[_Camera] = []

        for c in CAMERAS:
            cams.append(
                _Camera(
                    name=c.name,
                    x=c.x,
                    y=c.y,
                    z=c.z,
                    roll=_deg2rad(c.roll_deg),
                    pitch=_deg2rad(c.pitch_deg),
                    yaw=_deg2rad(c.yaw_deg),
                    hfov_rad=_deg2rad(c.hfov_deg),
                    vfov_rad=_deg2rad(c.vfov_deg),
                    min_range=max(0.0, c.min_range),
                    max_range=max(0.05, c.max_range),
                    update_hz=max(1e-6, c.update_hz),
                    noise_xy=max(0.0, c.noise_xy),
                    noise_z=max(0.0, c.noise_z),
                    dropout=_clamp(c.dropout, 0.0, 1.0),
                )
            )

        if cams:
            return cams

        # Fallback if list is empty (robot-relative)
        return [
            _Camera(
                name="cam_front",
                x=0.35,
                y=0.0,
                z=1.35,
                roll=0.0,
                pitch=_deg2rad(-12.0),
                yaw=_deg2rad(0.0),
                hfov_rad=_deg2rad(95.0),
                vfov_rad=_deg2rad(60.0),
                min_range=0.2,
                max_range=9.0,
                update_hz=15.0,
                noise_xy=0.05,
                noise_z=0.03,
                dropout=0.04,
            ),
            _Camera(
                name="cam_back",
                x=-0.35,
                y=0.0,
                z=1.35,
                roll=0.0,
                pitch=_deg2rad(-12.0),
                yaw=_deg2rad(180.0),
                hfov_rad=_deg2rad(95.0),
                vfov_rad=_deg2rad(60.0),
                min_range=0.2,
                max_range=9.0,
                update_hz=15.0,
                noise_xy=0.05,
                noise_z=0.03,
                dropout=0.04,
            ),
        ]

    def _detect_object(
        self,
        cam: _Camera,
        obj: WorldObject,
        cam_x: float,
        cam_y: float,
        cam_z: float,
        cam_yaw: float,
    ) -> _Detected | None:
        assert self.rng is not None
        dx = obj.x - cam_x
        dy = obj.y - cam_y
        dz = obj.z - cam_z
        dist2 = dx * dx + dy * dy + dz * dz
        if dist2 < (cam.min_range * cam.min_range) or dist2 > (cam.max_range * cam.max_range):
            return None

        horiz = math.hypot(dx, dy)
        yaw_to = math.atan2(dy, dx)
        pitch_to = math.atan2(dz, max(1e-9, horiz))
        dyaw = _wrap_rad(yaw_to - cam_yaw)
        dpitch = pitch_to - cam.pitch

        if abs(dyaw) > cam.hfov_rad * 0.5 or abs(dpitch) > cam.vfov_rad * 0.5:
            return None

        if self.rng.random() < cam.dropout:
            return None

        dist = math.sqrt(dist2)
        off_yaw = abs(dyaw) / max(1e-6, cam.hfov_rad * 0.5)
        off_pitch = abs(dpitch) / max(1e-6, cam.vfov_rad * 0.5)
        p = 1.0 - 0.45 * (dist / max(1e-6, cam.max_range)) - 0.20 * off_yaw - 0.20 * off_pitch
        if self.rng.random() > _clamp(p, 0.0, 1.0):
            return None

        nx = obj.x + _box_muller(self.rng) * cam.noise_xy
        ny = obj.y + _box_muller(self.rng) * cam.noise_xy
        nz = obj.z + _box_muller(self.rng) * cam.noise_z

        score = dist2 + (off_yaw * off_yaw + off_pitch * off_pitch) * dist2
        return _Detected(oid=obj.oid, class_id=obj.class_id, x=nx, y=ny, z=nz, score=score)

    def _simulate_vision(
        self,
        now_s: float,
        objects: List[WorldObject],
        robot_sizes: Dict[str, Tuple[float, float]],
        robot_pose: Pose2d | None,
    ) -> Tuple[List[WorldObject], List[VisionObstacle]]:
        assert self.rng is not None

        detections: Dict[str, _Detected] = {}
        if robot_pose is None:
            robot_x = 0.0
            robot_y = 0.0
            robot_yaw = 0.0
        else:
            robot_x = float(robot_pose.x)
            robot_y = float(robot_pose.y)
            robot_yaw = float(robot_pose.rotation().radians())
        cos_r = math.cos(robot_yaw)
        sin_r = math.sin(robot_yaw)

        for cam in self._cams:
            if now_s < cam.next_update_s:
                continue
            cam.next_update_s = now_s + (1.0 / max(1e-6, cam.update_hz))
            cam_x = robot_x + cam.x * cos_r - cam.y * sin_r
            cam_y = robot_y + cam.x * sin_r + cam.y * cos_r
            cam_z = cam.z
            cam_yaw = _wrap_rad(robot_yaw + cam.yaw)
            for obj in objects:
                det = self._detect_object(cam, obj, cam_x, cam_y, cam_z, cam_yaw)
                if det is None:
                    continue
                prev = detections.get(det.oid)
                if prev is None or det.score < prev.score:
                    detections[det.oid] = det

        # Update tracked world objects
        for oid, det in detections.items():
            tr = self._tracked_objects.get(oid)
            if tr is None:
                obj = WorldObject(oid=oid, class_id=det.class_id, x=det.x, y=det.y, z=det.z)
                self._tracked_objects[oid] = _TrackedWorld(
                    obj=obj, last_seen_s=now_s, last_update_s=now_s
                )
            else:
                tr.last_seen_s = now_s
                if now_s - tr.last_update_s >= self._obj_publish_period_s:
                    tr.obj = WorldObject(
                        oid=oid, class_id=det.class_id, x=det.x, y=det.y, z=det.z
                    )
                    tr.last_update_s = now_s

        # Drop stale world objects
        for oid in list(self._tracked_objects.keys()):
            tr = self._tracked_objects[oid]
            if now_s - tr.last_seen_s > self._visibility_timeout_s:
                del self._tracked_objects[oid]

        # Update tracked obstacles (robots only)
        for oid, det in detections.items():
            if det.class_id not in (CLASS_ROBOT_BLUE, CLASS_ROBOT_RED):
                continue
            sx, sy = robot_sizes.get(oid, (0.75, 0.75))
            tr = self._tracked_obstacles.get(oid)
            if tr is None:
                obs = VisionObstacle(oid=oid, kind="robot", x=det.x, y=det.y, sx=sx, sy=sy)
                self._tracked_obstacles[oid] = _TrackedObstacle(
                    obs=obs, last_seen_s=now_s, last_update_s=now_s
                )
            else:
                tr.last_seen_s = now_s
                if now_s - tr.last_update_s >= self._obs_publish_period_s:
                    tr.obs = VisionObstacle(oid=oid, kind="robot", x=det.x, y=det.y, sx=sx, sy=sy)
                    tr.last_update_s = now_s

        for oid in list(self._tracked_obstacles.keys()):
            tr = self._tracked_obstacles[oid]
            if now_s - tr.last_seen_s > self._visibility_timeout_s:
                del self._tracked_obstacles[oid]

        objs_out = [tr.obj for tr in self._tracked_objects.values()]
        objs_out.sort(key=lambda o: o.oid)
        if len(objs_out) > self.cfg.max_objects:
            objs_out = objs_out[: self.cfg.max_objects]

        obs_out = [tr.obs for tr in self._tracked_obstacles.values()]
        obs_out.sort(key=lambda o: o.oid)
        if len(obs_out) > self.cfg.max_obstacles:
            obs_out = obs_out[: self.cfg.max_obstacles]

        return objs_out, obs_out

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
        if not self.fuel or pose is None:
            return

        px = float(pose.x)
        py = float(pose.y)
        hx, hy = self._pickup_half_extents()

        r = float(getattr(self.cfg, "fuel_radius_m", 0.075))
        r2 = r * r

        to_remove: List[str] = []
        for oid, fo in self.fuel.items():
            dx = max(abs(fo.x - px) - hx, 0.0)
            dy = max(abs(fo.y - py) - hy, 0.0)
            if dx * dx + dy * dy <= r2:
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

        # include robots as world objects for vision dedupe and camera visibility
        robot_sizes: Dict[str, Tuple[float, float]] = {}
        for ro in self.robots:
            objs.append(
                WorldObject(oid=ro.oid, class_id=ro.class_id, x=ro.x, y=ro.y, z=0.0)
            )
            robot_sizes[ro.oid] = (ro.sx, ro.sy)

        vis_objects, vis_obstacles = self._simulate_vision(now_s, objs, robot_sizes, pose)

        return ProviderFrame(
            objects=vis_objects,
            obstacles=vis_obstacles,
            extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
