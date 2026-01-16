# repulsor_3d_sim/viewer.py
from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import pyglet
from pyglet.window import key, mouse

from repulsor_3d_sim.config import ViewerConfig
from repulsor_3d_sim.model import WorldSnapshot
from repulsor_3d_sim.nt4 import NT4Reader
from repulsor_3d_sim.render.camera import OrbitCamera
from repulsor_3d_sim.render.scene import SceneRenderer


class _SnapshotWorker:
    def __init__(self, reader: NT4Reader, hz: float):
        self.reader = reader
        self.period_s = 1.0 / max(1.0, float(hz))
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[WorldSnapshot] = None
        self._connected: bool = False
        self._t = threading.Thread(target=self._run, name="nt4-snapshot-worker", daemon=True)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=0.5)

    def latest(self) -> Tuple[Optional[WorldSnapshot], bool]:
        with self._lock:
            return self._latest, self._connected

    def _run(self):
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            if now < next_t:
                self._stop.wait(timeout=next_t - now)
                continue

            snap = self.reader.snapshot()
            conn = self.reader.is_connected()

            with self._lock:
                self._latest = snap
                self._connected = conn

            next_t = max(next_t + self.period_s, time.time() + 0.0005)


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

        self.camera = OrbitCamera(
            distance=cfg.camera_distance_m,
            yaw_deg=cfg.camera_yaw_deg,
            pitch_deg=cfg.camera_pitch_deg,
            target=(cfg.field_length_m * 0.5, cfg.field_width_m * 0.5, 0.0),
        )

        self.renderer = SceneRenderer(cfg)

        self._keys = key.KeyStateHandler()
        self.window.push_handlers(self._keys)

        self._dragging = False
        self._last_mx = 0
        self._last_my = 0

        self._worker = _SnapshotWorker(self.reader, hz=float(cfg.fps) * 2.0)
        self._worker.start()

        self.last_snapshot: Optional[WorldSnapshot] = None
        self._last_connected: bool = False

        pyglet.clock.schedule_interval(self._tick, 1.0 / float(cfg.fps))

    def run(self):
        try:
            pyglet.app.run()
        finally:
            self._worker.stop()

    def _tick(self, dt: float):
        snap, conn = self._worker.latest()
        self.last_snapshot = snap
        self._last_connected = conn

        if self.cfg.follow_robot and snap is not None and snap.pose is not None:
            px = float(snap.pose.x)
            py = float(snap.pose.y)
            self.camera.target = (px, py, 0.0)

        self.window.invalid = True

    def on_draw(self):
        self.window.clear()
        self.renderer.draw(self.window, self.camera, self.last_snapshot, self._last_connected)

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
        self.camera.pitch_deg = max(-89.0, min(89.0, self.camera.pitch_deg + float(dy) * 0.35))

    def on_mouse_scroll(self, x: int, y: int, scroll_x: int, scroll_y: int):
        s = 1.0 - float(scroll_y) * 0.08
        self.camera.distance = max(1.5, min(80.0, self.camera.distance * s))

    def on_key_press(self, symbol: int, modifiers: int):
        if symbol == key.R:
            self.camera.target = (self.cfg.field_length_m * 0.5, self.cfg.field_width_m * 0.5, 0.0)
            self.camera.distance = self.cfg.camera_distance_m
            self.camera.pitch_deg = self.cfg.camera_pitch_deg
            self.camera.yaw_deg = self.cfg.camera_yaw_deg
