from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple
import numpy as np

@dataclass
class OrbitCamera:
    distance: float
    yaw_deg: float
    pitch_deg: float
    target: Tuple[float, float, float]

    def eye(self) -> Tuple[float, float, float]:
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        cx, cy, cz = self.target
        x = cx + self.distance * math.cos(pitch) * math.cos(yaw)
        y = cy + self.distance * math.cos(pitch) * math.sin(yaw)
        z = cz + self.distance * math.sin(pitch)
        return (float(x), float(y), float(z))

    def view_matrix(self) -> np.ndarray:
        ex, ey, ez = self.eye()
        tx, ty, tz = self.target
        up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        eye = np.array([ex, ey, ez], dtype=np.float32)
        target = np.array([tx, ty, tz], dtype=np.float32)

        f = target - eye
        fn = np.linalg.norm(f)
        if fn < 1e-6:
            f = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            fn = 1.0
        f = f / fn

        s = np.cross(f, up)
        sn = np.linalg.norm(s)
        if sn < 1e-6:
            s = np.array([0.0, 1.0, 0.0], dtype=np.float32)
            sn = 1.0
        s = s / sn

        u = np.cross(s, f)

        m = np.eye(4, dtype=np.float32)
        m[0, 0:3] = s
        m[1, 0:3] = u
        m[2, 0:3] = -f
        m[0, 3] = -float(np.dot(s, eye))
        m[1, 3] = -float(np.dot(u, eye))
        m[2, 3] = float(np.dot(f, eye))
        return m

    def projection_matrix(self, aspect: float, fov_deg: float = 60.0, near: float = 0.05, far: float = 500.0) -> np.ndarray:
        f = 1.0 / math.tan(math.radians(fov_deg) * 0.5)
        m = np.zeros((4, 4), dtype=np.float32)
        m[0, 0] = f / float(aspect)
        m[1, 1] = f
        m[2, 2] = (far + near) / (near - far)
        m[2, 3] = (2.0 * far * near) / (near - far)
        m[3, 2] = -1.0
        return m
