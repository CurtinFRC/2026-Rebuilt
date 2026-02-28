from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import cv2


@dataclass(frozen=True)
class CameraCalibration:
    K: np.ndarray
    dist: np.ndarray


@dataclass(frozen=True)
class CameraPoseField:
    R_f_c: np.ndarray
    p_f: np.ndarray


@dataclass(frozen=True)
class YoloBBox:
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class EMA3D:
    alpha: float
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


def fmt_xyz(v: Optional[np.ndarray], name: str = "pt", unit: str = "m") -> str:
    if v is None:
        return f"{name}: None"
    x, y, z = map(float, np.asarray(v, dtype=np.float64).reshape(-1)[:3])
    return f"{name}: x={x:.3f} {unit}, y={y:.3f} {unit}, z={z:.3f} {unit}"


def rot_x(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rot_y(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rot_z(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def R_field_from_cam_ypr(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    return rot_z(yaw_deg) @ rot_y(pitch_deg) @ rot_x(roll_deg)


def bbox_center_u(b: YoloBBox) -> float:
    return float(0.5 * (float(b.x1) + float(b.x2)))


def bbox_v_at_frac(b: YoloBBox, frac_from_top: float) -> float:
    f = float(np.clip(frac_from_top, 0.0, 1.0))
    return float(float(b.y1) + f * (float(b.y2) - float(b.y1)))


def bbox_top_center_pixel(b: YoloBBox, frac_from_top: float = 0.0) -> np.ndarray:
    u = bbox_center_u(b)
    v = bbox_v_at_frac(b, frac_from_top)
    return np.array([u, v], dtype=np.float64)


def pixel_to_unit_ray_camera_opencv(calib: CameraCalibration, uv: np.ndarray) -> np.ndarray:
    pts = np.asarray(uv, dtype=np.float64).reshape(1, 1, 2)
    und_norm = cv2.undistortPoints(pts, np.asarray(calib.K, dtype=np.float64), np.asarray(calib.dist, dtype=np.float64), P=None)
    x = float(und_norm[0, 0, 0])
    y = float(und_norm[0, 0, 1])
    ray = np.array([x, y, 1.0], dtype=np.float64)
    ray /= float(np.linalg.norm(ray))
    return ray


def apply_axis_conversion(ray_cv: np.ndarray, A_cam_from_cv: Optional[np.ndarray]) -> np.ndarray:
    r = np.asarray(ray_cv, dtype=np.float64).reshape(3)
    if A_cam_from_cv is None:
        out = r
    else:
        out = np.asarray(A_cam_from_cv, dtype=np.float64).reshape(3, 3) @ r
    out /= float(np.linalg.norm(out))
    return out


def ray_camera_to_field(pose: CameraPoseField, ray_c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R = np.asarray(pose.R_f_c, dtype=np.float64).reshape(3, 3)
    p = np.asarray(pose.p_f, dtype=np.float64).reshape(3)
    d_f = R.T @ np.asarray(ray_c, dtype=np.float64).reshape(3)
    d_f = d_f / float(np.linalg.norm(d_f))
    return p, d_f


def intersect_ray_with_plane(o: np.ndarray, d: np.ndarray, n: np.ndarray, p0: np.ndarray, eps: float = 1e-9) -> Optional[np.ndarray]:
    o = np.asarray(o, dtype=np.float64).reshape(3)
    d = np.asarray(d, dtype=np.float64).reshape(3)
    n = np.asarray(n, dtype=np.float64).reshape(3)
    p0 = np.asarray(p0, dtype=np.float64).reshape(3)

    denom = float(n @ d)
    if abs(denom) < float(eps):
        return None
    t = float(n @ (p0 - o)) / denom
    if t <= 0.0: # HERE
        return None
    return o + t * d


def intersect_ray_with_plane_z(o_f: np.ndarray, d_f: np.ndarray, z_plane: float, eps: float = 1e-9) -> Optional[np.ndarray]:
    n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    p0 = np.array([0.0, 0.0, float(z_plane)], dtype=np.float64)
    return intersect_ray_with_plane(o_f, d_f, n, p0, eps=eps)


def estimate_fuel_center_field(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    ball_radius_m: float = 0.15,
    A_cam_from_cv: Optional[np.ndarray] = None,
    frac_from_top: float = 0.0,
    min_abs_dz: float = 0.0,
    max_range_m: Optional[float] = None,
) -> Optional[np.ndarray]:
    uv = bbox_top_center_pixel(bbox, frac_from_top=frac_from_top)
    ray_cv = pixel_to_unit_ray_camera_opencv(calib, uv)
    ray_c = apply_axis_conversion(ray_cv, A_cam_from_cv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)

    dz = float(d_f[2])
    if float(min_abs_dz) > 0.0 and abs(dz) < float(min_abs_dz):
        # print("gate(min_abs_dz):", True)
        return None

    pt = intersect_ray_with_plane_z(o_f, d_f, z_plane=float(ball_radius_m))
    if pt is None:
        # print("pt:", None)
        return None

    if max_range_m is not None:
        if float(np.linalg.norm(np.asarray(pt, dtype=np.float64).reshape(3) - np.asarray(o_f, dtype=np.float64).reshape(3))) > float(max_range_m):
            # print("gate(max_range_m):", True)
            return None

    return pt


def estimate_fuel_center_field_debug(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    ball_radius_m: float = 0.15,
    A_cam_from_cv: Optional[np.ndarray] = None,
    frac_from_top: float = 0.0,
    min_abs_dz: float = 0.0,
    max_range_m: Optional[float] = None,
) -> Optional[np.ndarray]:
    uv = bbox_top_center_pixel(bbox, frac_from_top=frac_from_top)
    ray_cv = pixel_to_unit_ray_camera_opencv(calib, uv)
    ray_c = apply_axis_conversion(ray_cv, A_cam_from_cv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)

    print("uv:", uv)
    print("ray_cv:", ray_cv)
    print("ray_c:", ray_c)
    print("o_f:", o_f)
    print("d_f:", d_f)
    print("d_f.z:", float(d_f[2]))

    dz = float(d_f[2])
    if float(min_abs_dz) > 0.0 and abs(dz) < float(min_abs_dz):
        print("gate(min_abs_dz):", True)
        return None

    pt = intersect_ray_with_plane_z(o_f, d_f, z_plane=float(ball_radius_m))
    if pt is None:
        print("pt:", None)
        return None

    if max_range_m is not None:
        dist = float(np.linalg.norm(pt - o_f))
        print("range:", dist)
        if dist > float(max_range_m):
            print("gate(max_range_m):", True)
            return None

    print("pt:", pt)
    return pt
