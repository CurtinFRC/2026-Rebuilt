from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import Optional, Tuple

import pyglet
from pyglet.window import key, mouse

from repulsor_3d_sim.config import ViewerConfig
from repulsor_3d_sim.model import WorldSnapshot
from repulsor_3d_sim.nt4 import NT4Reader
from repulsor_3d_sim.render.camera import OrbitCamera
from repulsor_3d_sim.render.scene import SceneRenderer
from repulsor_3d_sim.truth_socket import TruthSocketReceiver


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _smooth_damp(
    current: float,
    target: float,
    current_velocity: float,
    smooth_time: float,
    dt: float,
    max_speed: float = float("inf"),
) -> tuple[float, float]:
    smooth_time = max(1e-4, float(smooth_time))
    dt = max(0.0, float(dt))

    omega = 2.0 / smooth_time
    x = omega * dt
    exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)

    change = current - target
    original_target = target

    if max_speed != float("inf"):
        max_change = max_speed * smooth_time
        change = _clamp(change, -max_change, max_change)

    target = current - change

    temp = (current_velocity + omega * change) * dt
    new_velocity = (current_velocity - omega * temp) * exp
    new_value = target + (change + temp) * exp

    if (original_target - current > 0.0) == (new_value > original_target):
        new_value = original_target
        new_velocity = 0.0

    return new_value, new_velocity


def _wrap_pi(rad: float) -> float:
    import math
    r = (rad + math.pi) % (2.0 * math.pi) - math.pi
    return r


def _smooth_damp_angle(
    current: float,
    target: float,
    current_velocity: float,
    smooth_time: float,
    dt: float,
    max_speed: float = float("inf"),
) -> tuple[float, float]:
    import math
    delta = _wrap_pi(target - current)
    unwrapped_target = current + delta
    v, vv = _smooth_damp(current, unwrapped_target, current_velocity, smooth_time, dt, max_speed)
    return _wrap_pi(v), vv


def _get_heading_rad(pose_obj) -> float:
    if pose_obj is None:
        return 0.0

    for name in ("theta", "heading", "yaw", "rot"):
        if hasattr(pose_obj, name):
            return float(getattr(pose_obj, name))

    if hasattr(pose_obj, "rotation"):
        r = getattr(pose_obj, "rotation")
        if callable(r):
            r = r()

        if hasattr(r, "radians") and callable(getattr(r, "radians")):
            return float(r.radians())
        if hasattr(r, "rad") and callable(getattr(r, "rad")):
            return float(r.rad())
        if hasattr(r, "degrees") and callable(getattr(r, "degrees")):
            import math
            return math.radians(float(r.degrees()))
        if hasattr(r, "value"):
            return float(getattr(r, "value"))

    return 0.0


def _replace_pose_fields(pose_obj, x: float, y: float, heading_rad: float):
    if pose_obj is None:
        return None

    try:
        from wpimath.geometry import Pose2d as WpiPose2d, Rotation2d
        if isinstance(pose_obj, WpiPose2d):
            return WpiPose2d(float(x), float(y), Rotation2d(float(heading_rad)))
    except Exception:
        pass

    for hname in ("theta", "heading", "yaw", "rot"):
        if hasattr(pose_obj, hname):
            try:
                return replace(pose_obj, x=float(x), y=float(y), **{hname: float(heading_rad)})
            except Exception:
                try:
                    setattr(pose_obj, "x", float(x))
                    setattr(pose_obj, "y", float(y))
                    setattr(pose_obj, hname, float(heading_rad))
                    return pose_obj
                except Exception:
                    break

    try:
        return replace(pose_obj, x=float(x), y=float(y))
    except Exception:
        try:
            setattr(pose_obj, "x", float(x))
            setattr(pose_obj, "y", float(y))
        except Exception:
            pass
        return pose_obj


class _SnapshotWorker:
    def __init__(self, reader: NT4Reader, hz: float):
        self.reader = reader
        self.period_s = 1.0 / max(1.0, float(hz))
        self._stop = threading.Event()
        self._latest_pair: Tuple[Optional[WorldSnapshot], bool] = (None, False)
        self._t = threading.Thread(target=self._run, name="nt4-snapshot-worker", daemon=True)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=0.5)

    def latest(self) -> Tuple[Optional[WorldSnapshot], bool]:
        return self._latest_pair

    def _run(self):
        next_t = time.perf_counter()
        wait = self._stop.wait
        perf = time.perf_counter
        period = self.period_s
        reader = self.reader

        while not self._stop.is_set():
            now = perf()
            dt = next_t - now
            if dt > 0:
                wait(timeout=dt)
                continue

            self._latest_pair = (reader.snapshot(), reader.is_connected())
            next_t = max(next_t + period, perf() + 0.0005)


class ViewerApp:
    def __init__(self, cfg: ViewerConfig):
        self.cfg = cfg
        self.reader = NT4Reader(cfg)

        self.window = pyglet.window.Window(
            width=cfg.window_w,
            height=cfg.window_h,
            caption="Repulsor 3D NT4 Viewer",
            resizable=True,
        )
        self.window.push_handlers(self)

        self._field_target = (float(cfg.field_length_m) * 0.5, float(cfg.field_width_m) * 0.5, 0.0)

        self.camera = OrbitCamera(
            distance=cfg.camera_distance_m,
            yaw_deg=cfg.camera_yaw_deg,
            pitch_deg=cfg.camera_pitch_deg,
            target=self._field_target,
        )

        self.renderer = SceneRenderer(cfg)
        self._show_camera_debug = bool(getattr(cfg, "show_camera_debug", True))
        self.renderer.show_camera_debug = self._show_camera_debug
        self._show_truth_fuel = bool(getattr(cfg, "show_truth_fuel", True))
        self.renderer.show_truth_fuel = self._show_truth_fuel
        self._show_age_filtered_fuel = bool(getattr(cfg, "show_age_filtered_fuel", False))
        self.renderer.show_age_filtered_fuel = self._show_age_filtered_fuel

        self._keys = key.KeyStateHandler()
        self.window.push_handlers(self._keys)

        self._dragging = False
        self._last_mx = 0
        self._last_my = 0

        self._worker = _SnapshotWorker(self.reader, hz=float(cfg.fps) * 2.0)
        self._worker.start()
        self._truth_receiver = None
        if cfg.truth_socket_enabled:
            self._truth_receiver = TruthSocketReceiver(cfg.truth_socket_host, cfg.truth_socket_port)
            self._truth_receiver.start()

        self.last_snapshot: Optional[WorldSnapshot] = None
        self._render_snapshot: Optional[WorldSnapshot] = None
        self._last_connected: bool = False

        self._fps_inv = 1.0 / float(cfg.fps)
        pyglet.clock.schedule_interval(self._tick, self._fps_inv)

        self._follow_target = self.camera.target
        self._follow_vel = (0.0, 0.0, 0.0)
        self._follow_smooth_time = float(getattr(cfg, "follow_smooth_time_s", 0.18))
        self._follow_max_speed = float(getattr(cfg, "follow_max_speed_mps", 12.0))

        # Robot smoothing (render pose)
        self._robot_smooth_time = float(getattr(cfg, "robot_smooth_time_s", 0.10))
        self._robot_max_speed = float(getattr(cfg, "robot_max_speed_mps", 20.0))
        self._robot_x = None
        self._robot_y = None
        self._robot_h = None
        self._robot_vx = 0.0
        self._robot_vy = 0.0
        self._robot_vh = 0.0

    def run(self):
        try:
            pyglet.app.run()
        finally:
            self._worker.stop()
            if self._truth_receiver is not None:
                self._truth_receiver.stop()

    def _tick(self, dt: float):
        snap, conn = self._worker.latest()
        if snap is not None and self._truth_receiver is not None:
            try:
                snap.truth = self._truth_receiver.latest()
            except Exception:
                pass
        self.last_snapshot = snap
        self._last_connected = conn

        # Smooth robot pose for rendering
        self._render_snapshot = snap
        if snap is not None and snap.pose is not None:
            px = float(snap.pose.x)
            py = float(snap.pose.y)
            ph = float(_get_heading_rad(snap.pose))

            if self._robot_x is None or self._robot_y is None or self._robot_h is None:
                self._robot_x, self._robot_y, self._robot_h = px, py, ph
                self._robot_vx = self._robot_vy = self._robot_vh = 0.0
            else:
                self._robot_x, self._robot_vx = _smooth_damp(
                    float(self._robot_x), px, self._robot_vx, self._robot_smooth_time, dt, self._robot_max_speed
                )
                self._robot_y, self._robot_vy = _smooth_damp(
                    float(self._robot_y), py, self._robot_vy, self._robot_smooth_time, dt, self._robot_max_speed
                )
                self._robot_h, self._robot_vh = _smooth_damp_angle(
                    float(self._robot_h), ph, self._robot_vh, self._robot_smooth_time, dt, max_speed=50.0
                )

            smoothed_pose = _replace_pose_fields(snap.pose, self._robot_x, self._robot_y, self._robot_h)
            if smoothed_pose is not None:
                try:
                    self._render_snapshot = replace(snap, pose=smoothed_pose)
                except Exception:
                    try:
                        snap.pose = smoothed_pose
                        self._render_snapshot = snap
                    except Exception:
                        self._render_snapshot = snap

        if self.cfg.follow_robot and self._render_snapshot is not None and self._render_snapshot.pose is not None:
            desired = (float(self._render_snapshot.pose.x), float(self._render_snapshot.pose.y), 0.0)

            cx, cy, cz = self._follow_target
            vx, vy, vz = self._follow_vel

            nx, vx = _smooth_damp(cx, desired[0], vx, self._follow_smooth_time, dt, self._follow_max_speed)
            ny, vy = _smooth_damp(cy, desired[1], vy, self._follow_smooth_time, dt, self._follow_max_speed)
            nz, vz = _smooth_damp(cz, desired[2], vz, self._follow_smooth_time, dt, self._follow_max_speed)

            self._follow_target = (nx, ny, nz)
            self._follow_vel = (vx, vy, vz)
            self.camera.target = self._follow_target

        self.window.invalid = True

    def on_draw(self):
        self.window.clear()
        self.renderer.draw(self.window, self.camera, self._render_snapshot, self._last_connected)

    def on_resize(self, width: int, height: int):
        self.renderer.resize(width, height)

    def on_mouse_press(self, x: int, y: int, button: int, modifiers: int):
        if button == mouse.LEFT:
            self._dragging = True
            self._last_mx = x
            self._last_my = y

    def on_mouse_release(self, x: int, y: int, button: int, modifiers: int):
        if button == mouse.LEFT:
            self._dragging = False

    def on_mouse_drag(self, x: int, y: int, dx: int, dy: int, buttons: int, modifiers: int):
        if not self._dragging:
            return
        self.camera.yaw_deg = (self.camera.yaw_deg + float(dx) * 0.35) % 360.0
        p = self.camera.pitch_deg + float(dy) * 0.35
        if p < -89.0:
            p = -89.0
        elif p > 89.0:
            p = 89.0
        self.camera.pitch_deg = p

    def on_mouse_scroll(self, x: int, y: int, scroll_x: int, scroll_y: int):
        s = 1.0 - float(scroll_y) * 0.08
        d = self.camera.distance * s
        if d < 1.5:
            d = 1.5
        elif d > 80.0:
            d = 80.0
        self.camera.distance = d

    def on_key_press(self, symbol: int, modifiers: int):
        if symbol == key.R:
            self.camera.target = self._field_target
            self._follow_target = self._field_target
            self._follow_vel = (0.0, 0.0, 0.0)

            self._robot_x = None
            self._robot_y = None
            self._robot_h = None
            self._robot_vx = self._robot_vy = self._robot_vh = 0.0

            self.camera.distance = self.cfg.camera_distance_m
            self.camera.pitch_deg = self.cfg.camera_pitch_deg
            self.camera.yaw_deg = self.cfg.camera_yaw_deg
        elif symbol == key.C:
            self._show_camera_debug = not self._show_camera_debug
            self.renderer.show_camera_debug = self._show_camera_debug
        elif symbol == key.T:
            self._show_truth_fuel = not self._show_truth_fuel
            self.renderer.show_truth_fuel = self._show_truth_fuel
        elif symbol == key.A:
            self._show_age_filtered_fuel = not self._show_age_filtered_fuel
            self.renderer.show_age_filtered_fuel = self._show_age_filtered_fuel
