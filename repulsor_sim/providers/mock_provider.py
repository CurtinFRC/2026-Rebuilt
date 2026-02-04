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
    CameraInfo,
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
    hfov_deg: float = 71
    vfov_deg: float = 48
    min_range: float = 0
    max_range: float = 5.0
    update_hz: float = 15.0
    noise_xy: float = 0#.01
    noise_z: float = 0#.03
    dropout: float = 0

CAMERAS: List[CameraConfig] = [
    CameraConfig(name="cam_left_rear",  x=0, y=-0.25, z=0.5, yaw_deg=-90 + 45,  pitch_deg=0),
    CameraConfig(name="cam_left_front", x= 0, y=0.25, z=0.5, yaw_deg=90 - 45,   pitch_deg=0),

    CameraConfig(name="cam_right_rear",  x=-0.25, y=-0.25, z=0.5, yaw_deg=-135, pitch_deg=0),
    CameraConfig(name="cam_right_front", x=-0.25, y=0.25, z=0.5, yaw_deg=135,  pitch_deg=0),
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

@dataclass
class _ShotBall:
    oid: str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    landed: bool
    age_s: float = 0.0
    roll_decel_scale: float = 1.0

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
        self._camera_infos: List[CameraInfo] = []
        self._fuel_target_count = 400
        self._pickup_enabled = True
        self._pickup_front_only = True
        self._pickup_use_body = True
        self._shots: List[_ShotBall] = []
        self._shot_oid = 0
        self._last_step_s = None
        self._last_ext_piece = 0
        self._shot_speed = 5.5
        self._shot_pitch = _deg2rad(25.0)
        self._shot_spread = _deg2rad(6.0)
        self._shot_speed_jitter = 0.15
        self._shot_pitch_jitter = _deg2rad(3.0)
        self._shot_offset = (0.45, 0.0, 0.55)
        self._shot_gravity = 9.81
        self._shot_roll_decel = 1.4
        self._shot_stop_speed = 0.15
        self._shot_air_drag = 0.12
        self._shot_substep = 0.02
        self._shot_hub_roll_speed = 1.8
        self._shot_hub_roll_jitter = 0.15
        self._shot_hub_roll_decel_scale = 0.45
        self._shot_bump_strength = 0.08
        self._shot_bounce = 0.35
        self._shot_bump_transfer = 0.55
        self._shot_bump_max = 2.2
        self._shot_bump_max_per_step = 6
        self._hub_rects: List[Tuple[float, float, float, float]] = []

    def reset(self, cfg: Config) -> None:
        self.cfg = cfg
        seed = int(getattr(cfg, "seed", 1337))
        self.rng = random.Random(seed)
        self._fuel_target_count = int(os.getenv("MOCK_FUEL_COUNT", "400"))
        self._pickup_enabled = os.getenv("MOCK_PICKUP_ENABLED", "1").strip() != "0"
        self._pickup_front_only = os.getenv("MOCK_PICKUP_FRONT_ONLY", "1").strip() != "0"
        self._pickup_use_body = os.getenv("MOCK_PICKUP_USE_BODY", "1").strip() != "0"
        self._shot_speed = float(os.getenv("MOCK_SHOT_SPEED", "5.5"))
        self._shot_pitch = _deg2rad(float(os.getenv("MOCK_SHOT_PITCH_DEG", "25.0")))
        self._shot_spread = _deg2rad(float(os.getenv("MOCK_SHOT_SPREAD_DEG", "6.0")))
        self._shot_speed_jitter = float(os.getenv("MOCK_SHOT_SPEED_JITTER", "0.15"))
        self._shot_pitch_jitter = _deg2rad(float(os.getenv("MOCK_SHOT_PITCH_JITTER_DEG", "3.0")))
        self._shot_gravity = float(os.getenv("MOCK_SHOT_GRAVITY", "9.81"))
        self._shot_roll_decel = float(os.getenv("MOCK_SHOT_ROLL_DECEL", "1.4"))
        self._shot_stop_speed = float(os.getenv("MOCK_SHOT_STOP_SPEED", "0.15"))
        self._shot_air_drag = float(os.getenv("MOCK_SHOT_AIR_DRAG", "0.12"))
        self._shot_substep = float(os.getenv("MOCK_SHOT_SUBSTEP", "0.02"))
        self._shot_hub_roll_speed = float(os.getenv("MOCK_SHOT_HUB_ROLL_SPEED", "1.8"))
        self._shot_hub_roll_jitter = float(os.getenv("MOCK_SHOT_HUB_ROLL_JITTER", "0.15"))
        self._shot_hub_roll_decel_scale = float(os.getenv("MOCK_SHOT_HUB_ROLL_DECEL_SCALE", "0.45"))
        self._shot_bump_strength = float(os.getenv("MOCK_SHOT_BUMP_STRENGTH", "0.08"))
        self._shot_bounce = float(os.getenv("MOCK_SHOT_BOUNCE", "0.35"))
        self._shot_bump_transfer = float(os.getenv("MOCK_SHOT_BUMP_TRANSFER", "0.55"))
        self._shot_bump_max = float(os.getenv("MOCK_SHOT_BUMP_MAX", "2.2"))
        self._shot_bump_max_per_step = int(os.getenv("MOCK_SHOT_BUMP_MAX_PER_STEP", "6"))
        off_x = float(os.getenv("MOCK_SHOT_OFFSET_X", "0.45"))
        off_y = float(os.getenv("MOCK_SHOT_OFFSET_Y", "0.0"))
        off_z = float(os.getenv("MOCK_SHOT_OFFSET_Z", "0.55"))
        self._shot_offset = (off_x, off_y, off_z)
        self._shots.clear()
        self._last_step_s = None

        self.cx = cfg.field_length_m * 0.5
        self.cy = cfg.field_width_m * 0.5

        self.x0 = 0.0
        self.x1 = float(cfg.field_length_m)
        self.y0 = 0.0
        self.y1 = float(cfg.field_width_m)

        self.nx = int(math.floor((self.x1 - self.x0) / cfg.grid_cell_m))
        self.ny = int(math.floor((self.y1 - self.y0) / cfg.grid_cell_m))

        self._forbidden = self._build_forbidden_regions()
        rect_width = float(os.getenv("MOCK_HUB_RECT_W", "0.5929315"))
        rect_height = float(os.getenv("MOCK_HUB_RECT_H", "5.711800"))
        rect_half_x = rect_width * 0.5
        rect_half_y = rect_height * 0.5
        hub_dx = float(os.getenv("MOCK_HUB_RECT_DX", "3.648981"))
        rect_cy = float(os.getenv("MOCK_HUB_RECT_CY", str(self.cy)))
        left_rect_x = (self.x1 * 0.5) - hub_dx
        right_rect_x = (self.x1 * 0.5) + hub_dx
        self._hub_rects = [
            (left_rect_x, rect_cy, rect_half_x, rect_half_y),
            (right_rect_x, rect_cy, rect_half_x, rect_half_y),
        ]
        self._cams = self._build_cameras()
        self._camera_infos = [
            CameraInfo(
                name=c.name,
                x=float(c.x),
                y=float(c.y),
                z=float(c.z),
                yaw_deg=float(c.yaw_deg),
                pitch_deg=float(c.pitch_deg),
                roll_deg=float(c.roll_deg),
                hfov_deg=float(c.hfov_deg),
                vfov_deg=float(c.vfov_deg),
                max_range=float(c.max_range),
            )
            for c in CAMERAS
        ]
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
        self._last_ext_piece = int(self._piece_sub.get())

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
        r00: float,
        r01: float,
        r02: float,
        r10: float,
        r11: float,
        r12: float,
        r20: float,
        r21: float,
        r22: float,
    ) -> _Detected | None:
        assert self.rng is not None
        dx = obj.x - cam_x
        dy = obj.y - cam_y
        dz = obj.z - cam_z
        dist2 = dx * dx + dy * dy + dz * dz
        if dist2 < (cam.min_range * cam.min_range) or dist2 > (cam.max_range * cam.max_range):
            return None

        # # world -> camera frame (R^T * v_world)
        x_cam = r00 * dx + r10 * dy + r20 * dz
        y_cam = r01 * dx + r11 * dy + r21 * dz
        z_cam = r02 * dx + r12 * dy + r22 * dz

        if x_cam <= 1e-6:
            return None

        yaw_to = math.atan2(y_cam, x_cam)
        pitch_to = math.atan2(z_cam, x_cam)
        dyaw = _wrap_rad(yaw_to)
        dpitch = pitch_to

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
            # Pitch in config uses negative = down; flip to match math convention here.
            cam_pitch = -cam.pitch
            cam_roll = cam.roll

            cyaw = math.cos(cam_yaw)
            syaw = math.sin(cam_yaw)
            cp = math.cos(cam_pitch)
            sp = math.sin(cam_pitch)
            cr = math.cos(cam_roll)
            sr = math.sin(cam_roll)

            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            r00 = cyaw * cp
            r01 = cyaw * sp * sr - syaw * cr
            r02 = cyaw * sp * cr + syaw * sr
            r10 = syaw * cp
            r11 = syaw * sp * sr + cyaw * cr
            r12 = syaw * sp * cr - cyaw * sr
            r20 = -sp
            r21 = cp * sr
            r22 = cp * cr
            for obj in objects:
                det = self._detect_object(
                    cam,
                    obj,
                    cam_x,
                    cam_y,
                    cam_z,
                    r00,
                    r01,
                    r02,
                    r10,
                    r11,
                    r12,
                    r20,
                    r21,
                    r22,
                )
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

        budget = max(0, self._fuel_target_count)
        out: Dict[Tuple[int, int], int] = {}

        top = scores[: max(1, len(scores) // 6)]
        mid = scores[len(scores) // 6 : len(scores) // 2]
        low = scores[len(scores) // 2 :]

        def alloc(bucket, lo, hi, frac):
            assert self.cfg is not None
            assert self.rng is not None
            nonlocal budget
            want = int(self._fuel_target_count * frac)
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
        if budget > 0 and scores:
            max_per_cell = 18
            while budget > 0:
                _, ix, iy = scores[self.rng.randrange(0, len(scores))]
                n = out.get((ix, iy), 0)
                if n >= max_per_cell:
                    continue
                out[(ix, iy)] = n + 1
                budget -= 1
        return out

    def _regen_fuel(self) -> None:
        assert self.cfg is not None
        assert self.rng is not None

        self.fuel.clear()
        self._picked_up.clear()
        self._shots.clear()
        oid_num = 0
        total = max(0, int(self._fuel_target_count))
        if total <= 0:
            return

        desired_spacing = float(os.getenv("MOCK_FUEL_GRID_SPACING", "0.22"))
        cols = int(math.ceil(math.sqrt(total)))
        rows = int(math.ceil(total / max(1, cols)))
        cols = max(1, cols)
        rows = max(1, rows)

        max_span_x = max(0.5, (self.x1 - self.x0) - 1.0)
        max_span_y = max(0.5, (self.y1 - self.y0) - 1.0)
        spacing_x = max_span_x if cols <= 1 else max_span_x / max(1, cols - 1)
        spacing_y = max_span_y if rows <= 1 else max_span_y / max(1, rows - 1)
        spacing = max(0.06, min(desired_spacing, spacing_x, spacing_y))

        width = spacing * (cols - 1)
        height = spacing * (rows - 1)
        start_x = self.cx - width * 0.5
        start_y = self.cy - height * 0.5

        jitter = float(os.getenv("MOCK_FUEL_GRID_JITTER", "0.02"))
        jitter = max(0.0, jitter)

        idx = 0
        while idx < total and idx < cols * rows:
            ix = idx % cols
            iy = idx // cols
            x = _clamp(start_x + ix * spacing + self.rng.uniform(-jitter, jitter), self.x0, self.x1)
            y = _clamp(start_y + iy * spacing + self.rng.uniform(-jitter, jitter), self.y0, self.y1)
            if not self._in_forbidden(x, y):
                oid = f"fuel_{oid_num}"
                oid_num += 1
                self.fuel[oid] = _FuelObj(oid=oid, x=x, y=y, z=self.cfg.fuel_z_m)
            idx += 1

        while len(self.fuel) < total:
            x = _clamp(
                self.cx + self.rng.uniform(-width * 0.55, width * 0.55), self.x0, self.x1
            )
            y = _clamp(
                self.cy + self.rng.uniform(-height * 0.55, height * 0.55), self.y0, self.y1
            )
            if self._in_forbidden(x, y):
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
        r = max(0.25, max(0.85) * 0.5)
        return r * r

    def _maybe_shoot(self, pose: Pose2d | None) -> None:
        assert self.cfg is not None
        assert self.rng is not None
        if self._piece_sub is None:
            return
        ext = int(self._piece_sub.get())
        if ext < 0:
            ext = 0
        if ext == 0 and self._last_ext_piece > 0 and self.piece_count > 0:
            shots = max(0, self.piece_count)
            for _ in range(shots):
                self._spawn_shot(pose)
            self.piece_count = 0
        self._last_ext_piece = ext

    def _spawn_shot(self, pose: Pose2d | None) -> None:
        assert self.cfg is not None
        assert self.rng is not None

        if self._picked_up:
            oid, fo = self._picked_up.popitem()
            oid_use = oid
        else:
            oid_use = f"shot_{self._shot_oid}"
            self._shot_oid += 1

        if pose is None:
            rx = self.cx
            ry = self.cy
            yaw = 0.0
        else:
            rx = float(pose.x)
            ry = float(pose.y)
            yaw = float(pose.rotation().radians())

        ox, oy, oz = self._shot_offset
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        sx = rx + ox * cy - oy * sy
        syw = ry + ox * sy + oy * cy
        sz = max(self.cfg.fuel_z_m, oz)

        yaw_spread = yaw + (self.rng.uniform(-1.0, 1.0) * self._shot_spread)
        pitch = self._shot_pitch + (self.rng.uniform(-1.0, 1.0) * self._shot_pitch_jitter)
        speed = self._shot_speed * (1.0 + self.rng.uniform(-self._shot_speed_jitter, self._shot_speed_jitter))
        speed = max(0.1, speed)
        vxy = speed * math.cos(pitch)
        vx = vxy * math.cos(yaw_spread)
        vy = vxy * math.sin(yaw_spread)
        vz = speed * math.sin(pitch)

        self._shots.append(
            _ShotBall(
                oid=oid_use,
                x=sx,
                y=syw,
                z=sz,
                vx=vx,
                vy=vy,
                vz=vz,
                landed=False,
            )
        )

    def _update_shots(self, dt: float) -> None:
        if not self._shots or dt <= 0.0:
            return
        assert self.cfg is not None
        g = self._shot_gravity
        ground_z = float(self.cfg.fuel_z_m)
        roll_decel = max(0.0, self._shot_roll_decel)
        stop_speed = max(0.01, self._shot_stop_speed)
        drag = max(0.0, self._shot_air_drag)
        substep = max(0.004, self._shot_substep)

        steps = int(math.ceil(dt / substep))
        steps = max(1, steps)
        step_dt = dt / steps

        shots = self._shots
        for _ in range(steps):
            if not shots:
                break
            remaining: List[_ShotBall] = []
            for b in shots:
                b.age_s += step_dt
                if not b.landed:
                    if drag > 1e-9:
                        d = math.exp(-drag * step_dt)
                        b.vx *= d
                        b.vy *= d
                    b.vz -= g * step_dt
                    b.x += b.vx * step_dt
                    b.y += b.vy * step_dt
                    b.z += b.vz * step_dt
                    if self._snap_to_hub(b, ground_z):
                        pass
                    elif b.z <= ground_z:
                        b.z = ground_z
                        b.vz = 0.0
                        b.landed = True
                if b.landed:
                    b.x += b.vx * step_dt
                    b.y += b.vy * step_dt
                    speed = math.hypot(b.vx, b.vy)
                    if speed > 1e-6:
                        decel = roll_decel * max(0.05, b.roll_decel_scale)
                        new_speed = max(0.0, speed - decel * step_dt)
                        if new_speed <= stop_speed:
                            self.fuel[b.oid] = _FuelObj(oid=b.oid, x=b.x, y=b.y, z=ground_z)
                            continue
                        scale = new_speed / speed
                        b.vx *= scale
                        b.vy *= scale
                    else:
                        self.fuel[b.oid] = _FuelObj(oid=b.oid, x=b.x, y=b.y, z=ground_z)
                        continue
                    self._bump_fuel_nearby(b, step_dt)

                if b.x < self.x0 or b.x > self.x1:
                    b.x = _clamp(b.x, self.x0, self.x1)
                    b.vx = 0.0
                if b.y < self.y0 or b.y > self.y1:
                    b.y = _clamp(b.y, self.y0, self.y1)
                    b.vy = 0.0
                remaining.append(b)
            shots = remaining

        self._shots = shots

    def _snap_to_hub(self, b: _ShotBall, ground_z: float) -> bool:
        if not self._hub_rects:
            return False
        for cx, cy, hx, hy in self._hub_rects:
            if abs(b.x - cx) <= hx and abs(b.y - cy) <= hy:
                b.x = cx
                b.y = cy
                b.z = ground_z
                b.vz = 0.0
                b.landed = True
                dx = self.cx - cx
                dy = self.cy - cy
                mag = math.hypot(dx, dy)
                if mag < 1e-6:
                    dx, dy = 1.0, 0.0
                    mag = 1.0
                jitter = 1.0 + self.rng.uniform(-self._shot_hub_roll_jitter, self._shot_hub_roll_jitter)
                speed = max(0.05, self._shot_hub_roll_speed * jitter)
                b.vx = dx / mag * speed
                b.vy = dy / mag * speed
                b.roll_decel_scale = self._shot_hub_roll_decel_scale
                return True
        return False

    def _bump_fuel_nearby(self, b: _ShotBall, dt: float) -> None:
        if not self.fuel or dt <= 0.0:
            return
        assert self.cfg is not None
        r = float(getattr(self.cfg, "fuel_radius_m", 0.075))
        bump_r = max(0.04, r * 2.1)
        bump_r2 = bump_r * bump_r
        strength = max(0.0, self._shot_bump_strength)
        bounce = max(0.0, self._shot_bounce)
        transfer = max(0.0, self._shot_bump_transfer)
        max_speed = max(0.05, self._shot_bump_max)
        max_per_step = max(1, self._shot_bump_max_per_step)
        if strength <= 1e-9:
            return
        bx = b.x
        by = b.y
        new_shots: List[_ShotBall] = []
        to_remove: List[str] = []
        hits = 0
        for oid, fo in list(self.fuel.items()):
            dx = fo.x - bx
            dy = fo.y - by
            d2 = dx * dx + dy * dy
            if d2 > bump_r2:
                continue
            d = math.sqrt(d2) if d2 > 1e-9 else 1e-6
            nx = dx / d
            ny = dy / d
            push = (bump_r - d) * strength
            if push > 0.0:
                fo.x = _clamp(fo.x + nx * push, self.x0, self.x1)
                fo.y = _clamp(fo.y + ny * push, self.y0, self.y1)

            vn = b.vx * nx + b.vy * ny
            if vn > 0.0:
                b.vx -= (1.0 + bounce) * vn * nx
                b.vy -= (1.0 + bounce) * vn * ny
            else:
                b.vx += nx * strength * 0.02
                b.vy += ny * strength * 0.02

            if transfer > 1e-6:
                speed = min(max_speed, max(0.12, abs(vn) * transfer))
                new_shots.append(
                    _ShotBall(
                        oid=oid,
                        x=fo.x,
                        y=fo.y,
                        z=fo.z,
                        vx=nx * speed,
                        vy=ny * speed,
                        vz=0.0,
                        landed=True,
                        roll_decel_scale=0.85,
                    )
                )
                to_remove.append(oid)
                hits += 1
                if hits >= max_per_step:
                    break

        if to_remove:
            for oid in to_remove:
                self.fuel.pop(oid, None)
        if new_shots:
            self._shots.extend(new_shots)

    def _pickup_half_extents(self) -> tuple[float, float]:
        assert self.cfg is not None
        hx = float(
            os.getenv("MOCK_PICKUP_HALF_X", getattr(self.cfg, "pickup_half_extent_x_m", 0.85 / 2))
        )
        hy = float(
            os.getenv("MOCK_PICKUP_HALF_Y", getattr(self.cfg, "pickup_half_extent_y_m", 0.85 / 2))
        )
        hx = max(0.01, hx)
        hy = max(0.01, hy)
        return hx, hy

    def _pickup_offsets(self, hx: float, hy: float) -> tuple[float, float]:
        ox = os.getenv("MOCK_PICKUP_OFFSET_X", "")
        oy = os.getenv("MOCK_PICKUP_OFFSET_Y", "")
        if ox != "":
            try:
                return float(ox), float(oy) if oy != "" else 0.0
            except ValueError:
                return 0.0, 0.0
        # default: slightly forward
        return 0.6 * hx, 0.0

    def _pickup_body_half_extents(self, hx: float, hy: float) -> tuple[float, float]:
        bx = os.getenv("MOCK_PICKUP_BODY_HALF_X", "")
        by = os.getenv("MOCK_PICKUP_BODY_HALF_Y", "")
        if bx != "":
            try:
                return max(0.01, float(bx)), max(0.01, float(by)) if by != "" else hy
            except ValueError:
                return hx, hy
        return hx, hy

    def _maybe_pickup(self, pose: Pose2d) -> None:
        assert self.cfg is not None
        if not self._pickup_enabled:
            return
        if not self.fuel or pose is None:
            return

        px = float(pose.x)
        py = float(pose.y)
        hx, hy = self._pickup_half_extents()
        ox, oy = self._pickup_offsets(hx, hy)
        bx, by = self._pickup_body_half_extents(hx, hy)
        yaw = float(pose.rotation().radians())
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        r = float(getattr(self.cfg, "fuel_radius_m", 0.075))
        r2 = r * r

        to_remove: List[str] = []
        for oid, fo in self.fuel.items():
            dxw = fo.x - px
            dyw = fo.y - py
            # world -> robot frame
            x_r = dxw * cy + dyw * sy
            y_r = -dxw * sy + dyw * cy

            # body intersection (centered)
            if self._pickup_use_body:
                dx_b = max(abs(x_r) - bx, 0.0)
                dy_b = max(abs(y_r) - by, 0.0)
                if dx_b * dx_b + dy_b * dy_b <= r2:
                    to_remove.append(oid)
                    continue

            x_r -= ox
            y_r -= oy
            if self._pickup_front_only and x_r < 0.0:
                continue
            dx = max(abs(x_r) - hx, 0.0)
            dy = max(abs(y_r) - hy, 0.0)
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

        if self._last_step_s is None:
            self._last_step_s = now_s
        dt = max(0.0, float(now_s - self._last_step_s))
        self._last_step_s = now_s

        self._maybe_shoot(pose)
        self._update_shots(dt)
        if pose is not None:
            self._maybe_pickup(pose)

        if (self.piece_count != self.last_piece_count):
            self.last_piece_count = self.piece_count
            self._piece_pub.set(int(self.piece_count))

        objs: List[WorldObject] = []
        truth_objs: List[WorldObject] = []
        for fo in self.fuel.values():
            o = WorldObject(oid=fo.oid, class_id=CLASS_FUEL, x=fo.x, y=fo.y, z=fo.z)
            objs.append(o)
            truth_objs.append(o)
        for b in self._shots:
            o = WorldObject(oid=b.oid, class_id=CLASS_FUEL, x=b.x, y=b.y, z=b.z)
            objs.append(o)
            truth_objs.append(o)

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
            cameras=self._camera_infos,
            truth_objects=truth_objs,
            extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
