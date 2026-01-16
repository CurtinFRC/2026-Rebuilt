from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
from pyglet.gl import (
    glBegin, glEnd, glVertex3f, glColor4f, glNormal3f,
    glPushMatrix, glPopMatrix, glTranslatef, glRotatef, glScalef,
    GL_TRIANGLES, GL_QUADS, GL_LINES,
)

@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    normals: np.ndarray
    indices: np.ndarray

def _norm(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=1)
    n[n < 1e-9] = 1.0
    return v / n[:, None]

def make_sphere_mesh(segments: int = 18, rings: int = 12) -> Mesh:
    verts: List[Tuple[float, float, float]] = []
    norms: List[Tuple[float, float, float]] = []
    idx: List[int] = []

    for r in range(rings + 1):
        v = float(r) / float(rings)
        phi = v * math.pi
        for s in range(segments + 1):
            u = float(s) / float(segments)
            theta = u * (2.0 * math.pi)
            x = math.sin(phi) * math.cos(theta)
            y = math.sin(phi) * math.sin(theta)
            z = math.cos(phi)
            verts.append((x, y, z))
            norms.append((x, y, z))

    def vid(r: int, s: int) -> int:
        return r * (segments + 1) + s

    for r in range(rings):
        for s in range(segments):
            a = vid(r, s)
            b = vid(r + 1, s)
            c = vid(r + 1, s + 1)
            d = vid(r, s + 1)
            idx.extend([a, b, c, a, c, d])

    v = np.array(verts, dtype=np.float32)
    n = np.array(norms, dtype=np.float32)
    i = np.array(idx, dtype=np.int32)
    return Mesh(v, n, i)

def draw_mesh(mesh: Mesh, rgba: Tuple[float, float, float, float]):
    r, g, b, a = rgba
    glColor4f(float(r), float(g), float(b), float(a))
    glBegin(GL_TRIANGLES)
    for k in range(mesh.indices.shape[0]):
        ii = int(mesh.indices[k])
        nx, ny, nz = mesh.normals[ii]
        vx, vy, vz = mesh.vertices[ii]
        glNormal3f(float(nx), float(ny), float(nz))
        glVertex3f(float(vx), float(vy), float(vz))
    glEnd()

def draw_box(center: Tuple[float, float, float], size: Tuple[float, float, float], yaw_deg: float, rgba: Tuple[float, float, float, float]):
    cx, cy, cz = center
    sx, sy, sz = size
    r, g, b, a = rgba

    glPushMatrix()
    glTranslatef(float(cx), float(cy), float(cz))
    glRotatef(float(yaw_deg), 0.0, 0.0, 1.0)
    glScalef(float(sx), float(sy), float(sz))
    glColor4f(float(r), float(g), float(b), float(a))

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

def draw_grid(field_length: float, field_width: float, z: float, step: float = 1.0, rgba: Tuple[float, float, float, float] = (0.15, 0.15, 0.18, 1.0)):
    r, g, b, a = rgba
    glColor4f(float(r), float(g), float(b), float(a))
    glBegin(GL_LINES)
    x = 0.0
    while x <= field_length + 1e-6:
        glVertex3f(float(x), 0.0, float(z))
        glVertex3f(float(x), float(field_width), float(z))
        x += float(step)
    y = 0.0
    while y <= field_width + 1e-6:
        glVertex3f(0.0, float(y), float(z))
        glVertex3f(float(field_length), float(y), float(z))
        y += float(step)
    glEnd()

def draw_axes(origin: Tuple[float, float, float], scale: float = 1.0):
    ox, oy, oz = origin
    glBegin(GL_LINES)
    glColor4f(1.0, 0.2, 0.2, 1.0)
    glVertex3f(float(ox), float(oy), float(oz))
    glVertex3f(float(ox + scale), float(oy), float(oz))
    glColor4f(0.2, 1.0, 0.2, 1.0)
    glVertex3f(float(ox), float(oy), float(oz))
    glVertex3f(float(ox), float(oy + scale), float(oz))
    glColor4f(0.2, 0.6, 1.0, 1.0)
    glVertex3f(float(ox), float(oy), float(oz))
    glVertex3f(float(ox), float(oy), float(oz + scale))
    glEnd()
