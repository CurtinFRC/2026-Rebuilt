from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import cv2


# -----------------------------
# Data structures
# -----------------------------

@dataclass(frozen=True)
class CameraCalibration:
    K: np.ndarray          # (3,3) camera matrix
    dist: np.ndarray       # (N,) distortion coefficients (e.g. 5, 8, 12, 14 depending model)
    # Optional: if you use a rectified/new camera matrix, set it. Else None -> use K.
    K_new: Optional[np.ndarray] = None


@dataclass(frozen=True)
class CameraPoseField:
    """
    Pose of the CAMERA expressed in FIELD coordinates.

    R_f_c: rotation matrix that rotates a vector from camera frame -> field frame. (3,3)
    p_f: camera position in field frame. (3,)
    """
    R_f_c: np.ndarray
    p_f: np.ndarray


@dataclass(frozen=True)
class YoloBBox:
    # pixel coords in the image
    x1: float
    y1: float
    x2: float
    y2: float


# -----------------------------
# Geometry helpers
# -----------------------------

def bbox_top_center_pixel(b: YoloBBox) -> np.ndarray:
    """
    Returns pixel (u, v) for top-center of bbox.
    """
    u = 0.5 * (b.x1 + b.x2)
    v = b.y1
    return np.array([u, v], dtype=np.float64)


def pixel_to_unit_ray_camera(calib: CameraCalibration, uv: np.ndarray) -> np.ndarray:
    """
    Converts a distorted pixel to a unit direction vector in the camera frame.

    Uses cv2.undistortPoints to get normalized (x, y) where:
      [x, y, 1] is the ray direction in camera coordinates (before normalization).

    Camera frame convention here is OpenCV:
      +x right, +y down, +z forward.
    """
    K = calib.K
    dist = calib.dist
    P = calib.K_new if calib.K_new is not None else calib.K

    pts = uv.reshape(1, 1, 2).astype(np.float64)
    und = cv2.undistortPoints(pts, K, dist, P=P)  # output is in pixel coords if P is set
    # If P is provided, und is in pixel coords of P; we want normalized coords.
    # Easiest: run without P to get normalized coords:
    und_norm = cv2.undistortPoints(pts, K, dist, P=None)  # normalized image coords
    x = float(und_norm[0, 0, 0])
    y = float(und_norm[0, 0, 1])

    ray = np.array([x, y, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)
    return ray


def ray_camera_to_field(pose: CameraPoseField, ray_c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Transforms a ray direction from camera frame to field frame.

    Returns (ray_origin_field, ray_dir_field_unit).
    """
    d_f = pose.R_f_c @ ray_c
    d_f = d_f / np.linalg.norm(d_f)
    o_f = pose.p_f
    return o_f, d_f


def intersect_ray_with_plane_z(o_f: np.ndarray, d_f: np.ndarray, z_plane: float) -> Optional[np.ndarray]:
    """
    Intersects ray: p(t) = o_f + t * d_f with plane z = z_plane (field frame).

    Returns intersection point or None if ray parallel or behind camera.
    """
    dz = float(d_f[2])
    if abs(dz) < 1e-9:
        return None  # parallel to plane

    t = (z_plane - float(o_f[2])) / dz
    if t <= 0.0:
        return None  # intersection behind camera

    return o_f + t * d_f


def estimate_fuel_center_field(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    ball_radius_m: float = 0.15,
) -> Optional[np.ndarray]:
    """
    Main estimator: bbox -> top-center pixel -> field ray -> intersect z=radius plane.
    """
    uv = bbox_top_center_pixel(bbox)
    ray_c = pixel_to_unit_ray_camera(calib, uv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)
    return intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius_m)

@dataclass
class EMA3D:
    alpha: float  # 0..1 (higher = follows new measurements more)
    x: Optional[np.ndarray] = None

    def update(self, meas: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if meas is None:
            return self.x
        meas = meas.astype(np.float64)
        if self.x is None:
            self.x = meas
        else:
            self.x = (1.0 - self.alpha) * self.x + self.alpha * meas
        return self.x

def estimate_fuel_center_field_debug(bbox, calib, pose, ball_radius_m=0.15):
    uv = bbox_top_center_pixel(bbox)
    ray_c = pixel_to_unit_ray_camera(calib, uv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)

    print("uv:", uv)
    print("ray_c:", ray_c)
    print("o_f:", o_f)
    print("d_f:", d_f)

    dz = float(d_f[2])
    print("d_f.z:", dz)
    t = (ball_radius_m - float(o_f[2])) / dz if abs(dz) > 1e-9 else None
    print("t:", t)

    return intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius_m)

K = np.array([[950, 0, 640],
              [0, 950, 360],
              [0,   0,   1]], dtype=np.float64)
dist = np.array([0.1, -0.05, 0.0, 0.0, 0.0], dtype=np.float64)

calib = CameraCalibration(K=K, dist=dist)

R_f_c = np.eye(3, dtype=np.float64)   
p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)  

bbox = YoloBBox(x1=600, y1=250, x2=680, y2=330)

def rot_x(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]], dtype=np.float64)

R_f_c = rot_x(-30)  # try -20 to -45
pose = CameraPoseField(R_f_c=R_f_c, p_f=p_f)

pt = estimate_fuel_center_field_debug(bbox, calib, pose, ball_radius_m=0.15)
print("pt:", pt)
