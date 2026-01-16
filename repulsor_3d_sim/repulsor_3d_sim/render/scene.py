# repulsor_3d_sim/render/scene.py
from __future__ import annotations

import math
from typing import Optional

import pyglet
from pyglet.gl import (
    GL_COLOR_BUFFER_BIT,
    GL_CULL_FACE,
    GL_DEPTH_BUFFER_BIT,
    glClear,
    glDisable,
    glEnable,
)

from repulsor_3d_sim.config import ViewerConfig
from repulsor_3d_sim.model import WorldSnapshot
from repulsor_3d_sim.render.camera import OrbitCamera
from repulsor_3d_sim.render.glutil import init_gl, set_matrices, set_viewport
from repulsor_3d_sim.render.primitives import (
    draw_axes,
    draw_box,
    draw_grid,
    draw_mesh,
    make_sphere_mesh,
)


class SceneRenderer:
    def __init__(self, cfg: ViewerConfig):
        self.cfg = cfg
        init_gl()
        self.sphere = make_sphere_mesh(segments=24, rings=16)
        self._w = int(cfg.window_w)
        self._h = int(cfg.window_h)

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

        draw_grid(self.cfg.field_length_m, self.cfg.field_width_m, self.cfg.field_z_m, step=1.0)
        draw_axes((0.0, 0.0, self.cfg.field_z_m), scale=1.0)

        if snap is not None:
            self._draw_objects(snap)
            self._draw_pose_box(snap)
            self._draw_robot_box(snap)

        glDisable(GL_CULL_FACE)
        self._update_caption(window, snap, connected)

    def _draw_objects(self, snap: WorldSnapshot):
        r = float(self.cfg.ball_radius_m)
        base_z = float(self.cfg.field_z_m)

        import pyglet.gl as gl

        for o in snap.fieldvision:
            cx, cy, cz = float(o.x), float(o.y), float(o.z)
            z = base_z + cz
            gl.glPushMatrix()
            gl.glTranslatef(cx, cy, z)
            gl.glScalef(r, r, r)
            draw_mesh(self.sphere, (1.0, 0.95, 0.15, 0.95))
            gl.glPopMatrix()

        side = float(self.cfg.obs_box_side_m)
        hz = base_z + (side * 0.5)
        for o in snap.repulsorvision:
            cx, cy = float(o.x), float(o.y)
            draw_box((cx, cy, hz), (side, side, side), 0.0, (1.0, 0.15, 0.15, 0.85))

    def _draw_pose_box(self, snap: WorldSnapshot):
        if snap.pose is None:
            return

        side = float(self.cfg.pose_box_side_m)
        h = float(self.cfg.pose_box_h_m)
        cx = float(snap.pose.x)
        cy = float(snap.pose.y)
        cz = float(self.cfg.field_z_m) + (h * 0.5)

        yaw_deg = float(snap.pose.rotation().degrees())
        yaw_rad = float(snap.pose.rotation().radians())

        draw_box((cx, cy, cz), (side, side, h), yaw_deg, (0.95, 0.95, 0.95, 0.9))

        hx = cx + math.cos(yaw_rad) * (side * 0.7)
        hy = cy + math.sin(yaw_rad) * (side * 0.7)
        draw_box((hx, hy, cz + h * 0.1), (side * 0.15, side * 0.15, h * 0.35), yaw_deg, (0.2, 0.9, 1.0, 0.95))

    def _draw_robot_box(self, snap: WorldSnapshot):
        if snap.pose is None:
            return

        l = float(self.cfg.robot_box_l_m)
        w = float(self.cfg.robot_box_w_m)
        h = float(self.cfg.robot_box_h_m)
        cx = float(snap.pose.x)
        cy = float(snap.pose.y)
        cz = float(self.cfg.field_z_m) + (h * 0.5)
        yaw_deg = float(snap.pose.rotation().degrees())
        draw_box((cx, cy, cz), (l, w, h), yaw_deg, (0.35, 0.35, 0.38, 0.75))

    def _update_caption(self, window: pyglet.window.Window, snap: Optional[WorldSnapshot], connected: bool):
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
        window.set_caption(
            f"Repulsor 3D NT4 Viewer | NT4 {st} | FV={fv_n} RV={rv_n} | Pose={pose_s} | Ex={ex[0]:.2f},{ex[1]:.2f},{ex[2]:.2f},{ex[3]:.2f},{ex[4]:.2f},{ex[5]:.2f}"
        )
