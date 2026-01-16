from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from pyglet.gl import (
    GL_ARRAY_BUFFER,
    GL_COMPILE,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_FLOAT,
    GL_LINES,
    GL_NORMAL_ARRAY,
    GL_POINT_SMOOTH,
    GL_POINTS,
    GL_STATIC_DRAW,
    GL_TRIANGLES,
    GL_UNSIGNED_INT,
    GL_VERTEX_ARRAY,
    glBegin,
    glBindBuffer,
    glBufferData,
    glCallList,
    glColor4f,
    glDisable,
    glDisableClientState,
    glDrawArrays,
    glDrawElements,
    glEnable,
    glEnableClientState,
    glEnd,
    glEndList,
    glGenBuffers,
    glGenLists,
    glNewList,
    glNormalPointer,
    glPointSize,
    glPopMatrix,
    glPushMatrix,
    glRotatef,
    glScalef,
    glTranslatef,
    glVertex3f,
    glVertexPointer,
    glNormal3f,
    GL_QUADS,
)

_LAST_COLOR = (None, None, None, None)
_LAST_VBO_BIND = (0, 0)
_LAST_ARRAY_STATE = (False, False)
_LAST_PTR_STATE = (0, 0, 0)
_LAST_POINT_STATE = (False, 0.0)

def _set_color(rgba: Tuple[float, float, float, float]):
    global _LAST_COLOR
    r, g, b, a = rgba
    if _LAST_COLOR != (r, g, b, a):
        glColor4f(float(r), float(g), float(b), float(a))
        _LAST_COLOR = (r, g, b, a)

def _bind_buffers(vbo_v: int, vbo_i: int):
    global _LAST_VBO_BIND
    if _LAST_VBO_BIND[0] != vbo_v:
        glBindBuffer(GL_ARRAY_BUFFER, int(vbo_v))
        _LAST_VBO_BIND = (vbo_v, _LAST_VBO_BIND[1])
    if _LAST_VBO_BIND[1] != vbo_i:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, int(vbo_i))
        _LAST_VBO_BIND = (_LAST_VBO_BIND[0], vbo_i)

def _ensure_arrays_enabled():
    global _LAST_ARRAY_STATE
    v_on, n_on = _LAST_ARRAY_STATE
    if not v_on:
        glEnableClientState(GL_VERTEX_ARRAY)
        v_on = True
    if not n_on:
        glEnableClientState(GL_NORMAL_ARRAY)
        n_on = True
    _LAST_ARRAY_STATE = (v_on, n_on)

def _ensure_vertex_only_enabled():
    global _LAST_ARRAY_STATE
    v_on, n_on = _LAST_ARRAY_STATE
    if n_on:
        glDisableClientState(GL_NORMAL_ARRAY)
        n_on = False
    if not v_on:
        glEnableClientState(GL_VERTEX_ARRAY)
        v_on = True
    _LAST_ARRAY_STATE = (v_on, n_on)

def _disable_arrays():
    global _LAST_ARRAY_STATE
    v_on, n_on = _LAST_ARRAY_STATE
    if n_on:
        glDisableClientState(GL_NORMAL_ARRAY)
        n_on = False
    if v_on:
        glDisableClientState(GL_VERTEX_ARRAY)
        v_on = False
    _LAST_ARRAY_STATE = (v_on, n_on)

def _maybe_set_pointers(vbo_v: int, vbo_n: int, stride: int = 0):
    global _LAST_PTR_STATE
    if _LAST_PTR_STATE != (vbo_v, vbo_n, stride):
        glBindBuffer(GL_ARRAY_BUFFER, int(vbo_v))
        glVertexPointer(3, GL_FLOAT, int(stride), ctypes.c_void_p(0))
        glBindBuffer(GL_ARRAY_BUFFER, int(vbo_n))
        glNormalPointer(GL_FLOAT, int(stride), ctypes.c_void_p(0))
        _LAST_PTR_STATE = (vbo_v, vbo_n, stride)

def _set_points_enabled(enabled: bool, size: float):
    global _LAST_POINT_STATE
    en, sz = _LAST_POINT_STATE
    if enabled and not en:
        glEnable(GL_POINT_SMOOTH)
        en = True
    if not enabled and en:
        glDisable(GL_POINT_SMOOTH)
        en = False
    if enabled:
        if sz != size:
            glPointSize(float(size))
            sz = size
    _LAST_POINT_STATE = (en, sz)

@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    normals: np.ndarray
    indices: np.ndarray
    display_list: int = 0
    vbo_v: int = 0
    vbo_n: int = 0
    vbo_i: int = 0

def _upload_vbo(target: int, arr: np.ndarray) -> int:
    if glGenBuffers is None:
        return 0
    bid = ctypes.c_uint(0)
    glGenBuffers(1, ctypes.byref(bid))
    if int(bid.value) == 0:
        return 0
    glBindBuffer(target, int(bid.value))
    glBufferData(target, int(arr.nbytes), arr.ctypes.data_as(ctypes.c_void_p), GL_STATIC_DRAW)
    return int(bid.value)

def make_sphere_mesh(segments: int = 18, rings: int = 12) -> Mesh:
    segments = int(max(3, segments))
    rings = int(max(2, rings))

    verts: List[Tuple[float, float, float]] = []
    norms: List[Tuple[float, float, float]] = []
    idx: List[int] = []

    for r in range(rings + 1):
        v = float(r) / float(rings)
        phi = v * math.pi
        sp = math.sin(phi)
        cp = math.cos(phi)
        for s in range(segments + 1):
            u = float(s) / float(segments)
            theta = u * (2.0 * math.pi)
            ct = math.cos(theta)
            st = math.sin(theta)
            x = sp * ct
            y = sp * st
            z = cp
            verts.append((x, y, z))
            norms.append((x, y, z))

    stride = segments + 1
    for r in range(rings):
        base0 = r * stride
        base1 = (r + 1) * stride
        for s in range(segments):
            a = base0 + s
            b = base1 + s
            c = base1 + s + 1
            d = base0 + s + 1
            idx.extend([a, b, c, a, c, d])

    v = np.asarray(verts, dtype=np.float32)
    n = np.asarray(norms, dtype=np.float32)
    i = np.asarray(idx, dtype=np.uint32)

    vbo_v = _upload_vbo(GL_ARRAY_BUFFER, v)
    vbo_n = _upload_vbo(GL_ARRAY_BUFFER, n)
    vbo_i = _upload_vbo(GL_ELEMENT_ARRAY_BUFFER, i)

    dl = 0
    if not (vbo_v and vbo_n and vbo_i):
        dl = glGenLists(1)
        if dl:
            glNewList(dl, GL_COMPILE)
            glEnableClientState(GL_VERTEX_ARRAY)
            glEnableClientState(GL_NORMAL_ARRAY)
            glVertexPointer(3, GL_FLOAT, 0, v.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
            glNormalPointer(GL_FLOAT, 0, n.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
            glDrawElements(
                GL_TRIANGLES,
                int(i.shape[0]),
                GL_UNSIGNED_INT,
                i.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)),
            )
            glDisableClientState(GL_NORMAL_ARRAY)
            glDisableClientState(GL_VERTEX_ARRAY)
            glEndList()
        else:
            dl = 0
        vbo_v = vbo_n = vbo_i = 0

    return Mesh(v, n, i, int(dl), int(vbo_v), int(vbo_n), int(vbo_i))

def draw_mesh(mesh: Mesh, rgba: Tuple[float, float, float, float] | None):
    if rgba is not None:
        _set_color(tuple(map(float, rgba)))

    _set_points_enabled(False, 0.0)

    if mesh.vbo_v and mesh.vbo_n and mesh.vbo_i:
        _ensure_arrays_enabled()
        _maybe_set_pointers(mesh.vbo_v, mesh.vbo_n, 0)
        _bind_buffers(mesh.vbo_v, mesh.vbo_i)
        glDrawElements(GL_TRIANGLES, int(mesh.indices.shape[0]), GL_UNSIGNED_INT, ctypes.c_void_p(0))
        return

    if getattr(mesh, "display_list", 0):
        glCallList(int(mesh.display_list))
        return

    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)
    glVertexPointer(3, GL_FLOAT, 0, mesh.vertices.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
    glNormalPointer(GL_FLOAT, 0, mesh.normals.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
    ii = mesh.indices.astype(np.uint32, copy=False)
    glDrawElements(GL_TRIANGLES, int(ii.shape[0]), GL_UNSIGNED_INT, ii.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)))
    glDisableClientState(GL_NORMAL_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)

def draw_points(positions_xyz: np.ndarray, rgba: Tuple[float, float, float, float], size_px: float, smooth: bool = True):
    if positions_xyz is None:
        return
    if positions_xyz.size == 0:
        return
    if positions_xyz.dtype != np.float32:
        positions_xyz = positions_xyz.astype(np.float32, copy=False)

    _set_color(tuple(map(float, rgba)))
    _ensure_vertex_only_enabled()
    _bind_buffers(0, 0)
    _set_points_enabled(bool(smooth), float(size_px))

    glVertexPointer(3, GL_FLOAT, 0, positions_xyz.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
    glDrawArrays(GL_POINTS, 0, int(positions_xyz.shape[0]))

_BOX_VBO_V = 0
_BOX_VBO_N = 0
_BOX_VBO_I = 0
_BOX_IDX_COUNT = 0
_BOX_DL = 0

def _ensure_box_geometry():
    global _BOX_VBO_V, _BOX_VBO_N, _BOX_VBO_I, _BOX_IDX_COUNT, _BOX_DL

    if _BOX_VBO_V or _BOX_DL:
        return

    verts = np.array(
        [
            -0.5, -0.5,  0.5,  0.5, -0.5,  0.5,  0.5,  0.5,  0.5, -0.5,  0.5,  0.5,
            -0.5,  0.5, -0.5,  0.5,  0.5, -0.5,  0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
             0.5, -0.5, -0.5,  0.5,  0.5, -0.5,  0.5,  0.5,  0.5,  0.5, -0.5,  0.5,
            -0.5, -0.5,  0.5, -0.5,  0.5,  0.5, -0.5,  0.5, -0.5, -0.5, -0.5, -0.5,
            -0.5,  0.5, -0.5, -0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5, -0.5,
            -0.5, -0.5,  0.5, -0.5, -0.5, -0.5,  0.5, -0.5, -0.5,  0.5, -0.5,  0.5,
        ],
        dtype=np.float32,
    )

    norms = np.array(
        [
             0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,
             0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,
             1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,
            -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0,
             0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,
             0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,  0.0, -1.0,  0.0,
        ],
        dtype=np.float32,
    )

    idx = np.array(
        [
            0, 1, 2, 0, 2, 3,
            4, 5, 6, 4, 6, 7,
            8, 9, 10, 8, 10, 11,
            12, 13, 14, 12, 14, 15,
            16, 17, 18, 16, 18, 19,
            20, 21, 22, 20, 22, 23,
        ],
        dtype=np.uint32,
    )

    vbo_v = _upload_vbo(GL_ARRAY_BUFFER, verts)
    vbo_n = _upload_vbo(GL_ARRAY_BUFFER, norms)
    vbo_i = _upload_vbo(GL_ELEMENT_ARRAY_BUFFER, idx)

    if vbo_v and vbo_n and vbo_i:
        _BOX_VBO_V = int(vbo_v)
        _BOX_VBO_N = int(vbo_n)
        _BOX_VBO_I = int(vbo_i)
        _BOX_IDX_COUNT = int(idx.shape[0])
        return

    dl = glGenLists(1)
    if dl:
        glNewList(dl, GL_COMPILE)
        glBegin(GL_QUADS)

        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)

        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, -0.5)

        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)

        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(-0.5, -0.5, -0.5)

        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, -0.5)

        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)

        glEnd()
        glEndList()
        _BOX_DL = int(dl)

def draw_box(
    center: Tuple[float, float, float],
    size: Tuple[float, float, float],
    yaw_deg: float,
    rgba: Tuple[float, float, float, float],
):
    _ensure_box_geometry()

    _set_points_enabled(False, 0.0)

    cx, cy, cz = center
    sx, sy, sz = size

    glPushMatrix()
    glTranslatef(float(cx), float(cy), float(cz))
    glRotatef(float(yaw_deg), 0.0, 0.0, 1.0)
    glScalef(float(sx), float(sy), float(sz))

    _set_color(tuple(map(float, rgba)))

    if _BOX_VBO_V and _BOX_VBO_N and _BOX_VBO_I:
        _ensure_arrays_enabled()
        _maybe_set_pointers(_BOX_VBO_V, _BOX_VBO_N, 0)
        _bind_buffers(_BOX_VBO_V, _BOX_VBO_I)
        glDrawElements(GL_TRIANGLES, int(_BOX_IDX_COUNT), GL_UNSIGNED_INT, ctypes.c_void_p(0))
        glPopMatrix()
        return

    if _BOX_DL:
        glCallList(int(_BOX_DL))
        glPopMatrix()
        return

    glBegin(GL_QUADS)

    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(-0.5, -0.5, 0.5)
    glVertex3f(0.5, -0.5, 0.5)
    glVertex3f(0.5, 0.5, 0.5)
    glVertex3f(-0.5, 0.5, 0.5)

    glNormal3f(0.0, 0.0, -1.0)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(0.5, 0.5, -0.5)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(-0.5, -0.5, -0.5)

    glNormal3f(1.0, 0.0, 0.0)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(0.5, 0.5, -0.5)
    glVertex3f(0.5, 0.5, 0.5)
    glVertex3f(0.5, -0.5, 0.5)

    glNormal3f(-1.0, 0.0, 0.0)
    glVertex3f(-0.5, -0.5, 0.5)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(-0.5, -0.5, -0.5)

    glNormal3f(0.0, 1.0, 0.0)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f(0.5, 0.5, 0.5)
    glVertex3f(0.5, 0.5, -0.5)

    glNormal3f(0.0, -1.0, 0.0)
    glVertex3f(-0.5, -0.5, 0.5)
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(0.5, -0.5, 0.5)

    glEnd()
    glPopMatrix()

_GRID_CACHE: Dict[Tuple[float, float, float, float, float, float, float, float], int] = {}
_AXES_CACHE: Dict[Tuple[float, float, float, float], int] = {}

def draw_grid(
    field_length: float,
    field_width: float,
    z: float,
    step: float = 1.0,
    rgba: Tuple[float, float, float, float] = (0.15, 0.15, 0.18, 1.0),
):
    fl = float(field_length)
    fw = float(field_width)
    zz = float(z)
    st = float(step)
    r, g, b, a = map(float, rgba)

    key = (fl, fw, zz, st, r, g, b, a)
    dl = _GRID_CACHE.get(key, 0)

    if not dl:
        dl = int(glGenLists(1))
        if dl:
            glNewList(dl, GL_COMPILE)
            _set_color((r, g, b, a))
            glBegin(GL_LINES)

            x = 0.0
            end_x = fl + 1e-6
            end_y = fw + 1e-6
            while x <= end_x:
                glVertex3f(float(x), 0.0, zz)
                glVertex3f(float(x), fw, zz)
                x += st

            y = 0.0
            while y <= end_y:
                glVertex3f(0.0, float(y), zz)
                glVertex3f(fl, float(y), zz)
                y += st

            glEnd()
            glEndList()
            _GRID_CACHE[key] = dl

    _set_points_enabled(False, 0.0)

    if dl:
        glCallList(dl)
        return

    _set_color((r, g, b, a))
    glBegin(GL_LINES)
    x = 0.0
    while x <= fl + 1e-6:
        glVertex3f(float(x), 0.0, zz)
        glVertex3f(float(x), fw, zz)
        x += st
    y = 0.0
    while y <= fw + 1e-6:
        glVertex3f(0.0, float(y), zz)
        glVertex3f(fl, float(y), zz)
        y += st
    glEnd()

def draw_axes(origin: Tuple[float, float, float], scale: float = 1.0):
    ox, oy, oz = map(float, origin)
    sc = float(scale)

    key = (ox, oy, oz, sc)
    dl = _AXES_CACHE.get(key, 0)

    if not dl:
        dl = int(glGenLists(1))
        if dl:
            glNewList(dl, GL_COMPILE)
            glBegin(GL_LINES)
            _set_color((1.0, 0.2, 0.2, 1.0))
            glVertex3f(ox, oy, oz)
            glVertex3f(ox + sc, oy, oz)
            _set_color((0.2, 1.0, 0.2, 1.0))
            glVertex3f(ox, oy, oz)
            glVertex3f(ox, oy + sc, oz)
            _set_color((0.2, 0.6, 1.0, 1.0))
            glVertex3f(ox, oy, oz)
            glVertex3f(ox, oy, oz + sc)
            glEnd()
            glEndList()
            _AXES_CACHE[key] = dl

    _set_points_enabled(False, 0.0)

    if dl:
        glCallList(dl)
        return

    glBegin(GL_LINES)
    _set_color((1.0, 0.2, 0.2, 1.0))
    glVertex3f(ox, oy, oz)
    glVertex3f(ox + sc, oy, oz)
    _set_color((0.2, 1.0, 0.2, 1.0))
    glVertex3f(ox, oy, oz)
    glVertex3f(ox, oy + sc, oz)
    _set_color((0.2, 0.6, 1.0, 1.0))
    glVertex3f(ox, oy, oz)
    glVertex3f(ox, oy, oz + sc)
    glEnd()
