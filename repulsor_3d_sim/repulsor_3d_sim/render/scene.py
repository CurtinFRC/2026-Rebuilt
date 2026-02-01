from __future__ import annotations

import math
import os
import time
from typing import Optional

import pyglet
from pyglet.gl import (
    GL_COLOR_BUFFER_BIT,
    GL_CULL_FACE,
    GL_DEPTH_BUFFER_BIT,
    GL_LINES,
    GL_QUADS,
    glClear,
    glDisable,
    glEnable,
    glBegin,
    glColor4f,
    glEnd,
    glPopMatrix,
    glPushMatrix,
    glScalef,
    glTranslatef,
    glTexCoord2f,
    glTexParameteri,
    glBindTexture,
    glVertex3f,
    GL_LINEAR,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
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
        self._col_truth_fuel = (0.15, 0.95, 0.35, 0.70)
        self._col_other = (1.0, 0.15, 0.15, 0.85)
        self._col_us = (1, 0.27, 0, 0.75)
        self._col_cam = (0.20, 0.75, 1.0, 0.85)
        self._col_cam_fov = (0.20, 0.75, 1.0, 0.55)
        self._col_cam_ray_ok = (0.20, 0.95, 0.35, 0.55)
        self._col_cam_ray_bad = (1.00, 0.25, 0.25, 0.40)

        self.show_camera_debug = bool(getattr(cfg, "show_camera_debug", True))
        self.show_truth_fuel = bool(getattr(cfg, "show_truth_fuel", True))
        self.show_field_image = bool(getattr(cfg, "show_field_image", True))

        self._field_image_path = str(getattr(cfg, "field_image_path", "") or "")
        self._field_image_alpha = float(getattr(cfg, "field_image_alpha", 0.92))
        self._field_image_flip_x = bool(getattr(cfg, "field_image_flip_x", False))
        self._field_image_flip_y = bool(getattr(cfg, "field_image_flip_y", False))
        self._field_tex = self._load_field_texture(self._field_image_path)
        self._field_uvs = self._build_tex_uvs(self._field_tex)

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
        if self.show_field_image and self._field_tex is not None:
            self._draw_field_image()
        draw_axes((0.0, 0.0, self._field_z), scale=1.0)

        if snap is not None:
            if self.show_truth_fuel:
                self._draw_truth_fuel(snap)
            self._draw_fuel(snap)
            self._draw_other_robots(snap)
            self._draw_us_robot(snap)
            if self.show_camera_debug:
                self._draw_cameras(snap)

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

    def _load_field_texture(self, path: str):
        if not path:
            return None
        candidates = [path]
        if not os.path.isabs(path):
            base_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            base_root = os.path.abspath(os.path.join(base_pkg, ".."))
            candidates.append(os.path.join(base_pkg, path))
            candidates.append(os.path.join(base_root, path))
        for p in candidates:
            if os.path.isfile(p):
                try:
                    img = pyglet.image.load(p)
                    tex = img.get_texture()
                    glBindTexture(tex.target, tex.id)
                    glTexParameteri(tex.target, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
                    glTexParameteri(tex.target, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
                    glBindTexture(tex.target, 0)
                    return tex
                except Exception:
                    return None
        return None

    def _build_tex_uvs(self, tex):
        if tex is None:
            return [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        tc = getattr(tex, "tex_coords", None)
        if not tc or len(tc) < 8:
            return [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        uvs = [(tc[i], tc[i + 1]) for i in range(0, 12, 3)]
        if len(uvs) != 4:
            return [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        if self._field_image_flip_x or self._field_image_flip_y:
            us = [u for u, _v in uvs]
            vs = [_v for _u, _v in uvs]
            umin, umax = min(us), max(us)
            vmin, vmax = min(vs), max(vs)
            out = []
            for u, v in uvs:
                if self._field_image_flip_x:
                    u = umin + (umax - u)
                if self._field_image_flip_y:
                    v = vmin + (vmax - v)
                out.append((u, v))
            return out
        return uvs

    def _draw_field_image(self):
        tex = self._field_tex
        if tex is None:
            return
        z = self._field_z + 0.001
        fl = self._field_length
        fw = self._field_width
        alpha = max(0.0, min(1.0, self._field_image_alpha))
        glEnable(GL_TEXTURE_2D)
        glBindTexture(tex.target, tex.id)
        glColor4f(1.0, 1.0, 1.0, alpha)
        glBegin(GL_QUADS)
        uvs = self._field_uvs
        # (0,0) -> (fl,0) -> (fl,fw) -> (0,fw)
        glTexCoord2f(uvs[0][0], uvs[0][1])
        glVertex3f(0.0, 0.0, z)
        glTexCoord2f(uvs[1][0], uvs[1][1])
        glVertex3f(fl, 0.0, z)
        glTexCoord2f(uvs[2][0], uvs[2][1])
        glVertex3f(fl, fw, z)
        glTexCoord2f(uvs[3][0], uvs[3][1])
        glVertex3f(0.0, fw, z)
        glEnd()
        glBindTexture(tex.target, 0)
        glDisable(GL_TEXTURE_2D)

    def _draw_truth_fuel(self, snap: WorldSnapshot):
        truth = snap.truth
        if not truth:
            return

        base_z = self._field_z
        r = self._fuel_r * 0.85
        sphere = self.sphere
        col = self._col_truth_fuel

        for o in truth:
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

    def _draw_cameras(self, snap: WorldSnapshot):
        pose = snap.pose
        cams = snap.cameras if snap is not None else []
        if pose is None or not cams:
            return

        rx = float(pose.x)
        ry = float(pose.y)
        rz = float(self._field_z)
        yaw = float(pose.rotation().radians())
        ry_cos = math.cos(yaw)
        ry_sin = math.sin(yaw)

        fv = snap.fieldvision

        for c in cams:
            # robot-relative -> world
            cx = rx + c.x * ry_cos - c.y * ry_sin
            cyw = ry + c.x * ry_sin + c.y * ry_cos
            cz = rz + c.z
            yaw_world = yaw + math.radians(c.yaw_deg)
            pitch_world = math.radians(c.pitch_deg)
            roll_world = math.radians(c.roll_deg)
            hfov = math.radians(max(1e-6, c.hfov_deg))
            vfov = math.radians(max(1e-6, c.vfov_deg))
            depth = max(0.2, float(c.max_range))

            # camera marker
            draw_box((cx, cyw, cz), (0.06, 0.06, 0.06), math.degrees(yaw_world), self._col_cam)

            # draw frustum edges
            ch = math.cos(yaw_world)
            sh = math.sin(yaw_world)
            cp = math.cos(pitch_world)
            sp = math.sin(pitch_world)

            forward = (cp * ch, cp * sh, sp)
            right = (-sh, ch, 0.0)
            up = (-sp * ch, -sp * sh, cp)

            tan_h = math.tan(hfov * 0.5)
            tan_v = math.tan(vfov * 0.5)

            far_cx = cx + forward[0] * depth
            far_cy = cyw + forward[1] * depth
            far_cz = cz + forward[2] * depth

            wx = right[0] * depth * tan_h
            wy = right[1] * depth * tan_h
            wz = right[2] * depth * tan_h
            hx = up[0] * depth * tan_v
            hy = up[1] * depth * tan_v
            hz = up[2] * depth * tan_v

            corners = [
                (far_cx + wx + hx, far_cy + wy + hy, far_cz + wz + hz),
                (far_cx + wx - hx, far_cy + wy - hy, far_cz + wz - hz),
                (far_cx - wx - hx, far_cy - wy - hy, far_cz - wz - hz),
                (far_cx - wx + hx, far_cy - wy + hy, far_cz - wz + hz),
            ]

            glColor4f(*self._col_cam_fov)
            glBegin(GL_LINES)
            for p in corners:
                glVertex3f(cx, cyw, cz)
                glVertex3f(p[0], p[1], p[2])
            # far plane outline
            for i in range(4):
                a = corners[i]
                b = corners[(i + 1) % 4]
                glVertex3f(a[0], a[1], a[2])
                glVertex3f(b[0], b[1], b[2])
            glEnd()

            if not fv:
                continue

            # camera -> world object rays (debug)
            cr = math.cos(roll_world)
            sr = math.sin(roll_world)
            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            r00 = ch * cp
            r01 = ch * sp * sr - sh * cr
            r02 = ch * sp * cr + sh * sr
            r10 = sh * cp
            r11 = sh * sp * sr + ch * cr
            r12 = sh * sp * cr - ch * sr
            r20 = -sp
            r21 = cp * sr
            r22 = cp * cr

            glBegin(GL_LINES)
            for o in fv:
                ox = float(o.x)
                oy = float(o.y)
                oz = rz + float(o.z)
                dx = ox - cx
                dy = oy - cyw
                dz = oz - cz

                x_cam = r00 * dx + r10 * dy + r20 * dz
                y_cam = r01 * dx + r11 * dy + r21 * dz
                z_cam = r02 * dx + r12 * dy + r22 * dz

                if x_cam <= 1e-6:
                    glColor4f(*self._col_cam_ray_bad)
                else:
                    dyaw = math.atan2(y_cam, x_cam)
                    dpitch = math.atan2(z_cam, x_cam)
                    if abs(dyaw) <= hfov * 0.5 and abs(dpitch) <= vfov * 0.5:
                        glColor4f(*self._col_cam_ray_ok)
                    else:
                        glColor4f(*self._col_cam_ray_bad)

                glVertex3f(cx, cyw, cz)
                glVertex3f(ox, oy, oz)
            glEnd()

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
