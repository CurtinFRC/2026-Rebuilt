from __future__ import annotations

import math
import time
from typing import Optional

import pyglet
from pyglet.gl import (
    GL_COLOR_BUFFER_BIT,
    GL_CULL_FACE,
    GL_DEPTH_BUFFER_BIT,
    glClear,
    glDisable,
    glEnable,
    glPopMatrix,
    glPushMatrix,
    glScalef,
    glTranslatef,
)

from repulsor_3d_sim.config import ViewerConfig
from repulsor_3d_sim.model import WorldSnapshot
from repulsor_3d_sim.render.camera import OrbitCamera
from repulsor_3d_sim.render.glutil import init_gl, set_matrices, set_viewport
from repulsor_3d_sim.render.primitives import draw_axes, draw_box, draw_grid, draw_mesh, make_sphere_mesh
import trimesh

class SceneRenderer:
    def __init__(self, cfg: ViewerConfig):
        self.cfg = cfg
        init_gl()

        self.sphere = make_sphere_mesh(segments=24, rings=16)

        self._w = int(cfg.window_w)
        self._h = int(cfg.window_h)

        self._field_length = float(cfg.field_length_m)
        self._field_width = float(cfg.field_width_m)
        self._field_z = float(cfg.field_z_m)

        self._fuel_r = float(cfg.ball_radius_m)
        self._other_side = float(cfg.obs_box_side_m)
        self._other_hz = self._field_z + (self._other_side * 0.5)

        self._robot_l = float(cfg.robot_box_l_m)
        self._robot_w = float(cfg.robot_box_w_m)
        self._robot_h = float(cfg.robot_box_h_m)

        self._col_fuel = (1.0, 0.95, 0.15, 0.95)
        self._col_other = (1.0, 0.15, 0.15, 0.85)
        self._col_us = (0.35, 0.35, 0.38, 0.75)

        self._last_cap_t = 0.0
        self._last_cap_s = ""

    def resize(self, w: int, h: int):
        self._w = int(max(1, w))
        self._h = int(max(1, h))
        set_viewport(self._w, self._h)

    def draw(self, window: pyglet.window.Window, camera: OrbitCamera, snap: Optional[WorldSnapshot], connected: bool):
        w = int(max(1, window.width))
        h = int(max(1, window.height))
        if w != self._w or h != self._h:
            self.resize(w, h)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_CULL_FACE)

        aspect = float(w) / float(h)
        proj = camera.projection_matrix(aspect)
        view = camera.view_matrix()
        set_matrices(proj, view)

        draw_grid(self._field_length, self._field_width, self._field_z, step=1.0)
        draw_axes((0.0, 0.0, self._field_z), scale=1.0)

        if snap is not None:
            self._draw_fuel(snap)
            self._draw_other_robots(snap)
            self._draw_us_robot(snap)

        glDisable(GL_CULL_FACE)
        self._update_caption(window, snap, connected)

    def _draw_fuel(self, snap: WorldSnapshot):
        fv = snap.fieldvision
        if not fv:
            return

        base_z = self._field_z
        r = self._fuel_r
        sphere = self.sphere
        col = self._col_fuel

        for o in fv:
            glPushMatrix()
            glTranslatef(float(o.x), float(o.y), base_z + float(o.z))
            glScalef(r, r, r)
            draw_mesh(sphere, col)
            glPopMatrix()

    def _draw_other_robots(self, snap: WorldSnapshot):
        rv = snap.repulsorvision
        if not rv:
            return

        hz = self._other_hz
        side = self._other_side
        col = self._col_other

        for o in rv:
            draw_box((float(o.x), float(o.y), hz), (side, side, side), 0.0, col)

    def _draw_us_robot(self, snap: WorldSnapshot):
        pose = snap.pose
        if pose is None:
            return

        cx = float(pose.x)
        cy = float(pose.y)
        cz = self._field_z + (self._robot_h * 0.5)
        yaw_deg = float(pose.rotation().degrees())
        draw_box((cx, cy, cz), (self._robot_l, self._robot_w, self._robot_h), yaw_deg, self._col_us)

    def _update_caption(self, window: pyglet.window.Window, snap: Optional[WorldSnapshot], connected: bool):
        now = time.monotonic()
        if now - self._last_cap_t < 0.30:
            return

        fv_n = 0
        rv_n = 0
        pose_s = "none"
        ex = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        if snap is not None:
            fv_n = len(snap.fieldvision)
            rv_n = len(snap.repulsorvision)
            if snap.pose is not None:
                pose_s = f"x={snap.pose.x:.3f} y={snap.pose.y:.3f} th={snap.pose.rotation().degrees():.3f}"
            ex = snap.extrinsics

        st = "connected" if connected else "disconnected"
        cap = (
            f"Ex={ex[0]:.2f},{ex[1]:.2f},{ex[2]:.2f},{ex[3]:.2f},{ex[4]:.2f},{ex[5]:.2f}"
        )

        if cap != self._last_cap_s:
            window.set_caption(cap)
            self._last_cap_s = cap
        self._last_cap_t = now
