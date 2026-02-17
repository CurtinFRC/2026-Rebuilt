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

def fmt_xyz(v: np.ndarray, name="pt", unit="m"):
    if v is None:
        return f"{name}: None"
    x, y, z = map(float, v.reshape(-1)[:3])
    return f"{name}: x={x:.3f} {unit}, y={y:.3f} {unit}, z={z:.3f} {unit}"

def rot_x(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def R_field_from_cam_ypr(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    return rot_z(yaw_deg) @ rot_y(pitch_deg) @ rot_x(roll_deg)


def bbox_top_center_pixel(b: YoloBBox) -> np.ndarray:
    return np.array([0.5 * (b.x1 + b.x2), b.y1], dtype=np.float64)


def pixel_to_unit_ray_camera_opencv(calib: CameraCalibration, uv: np.ndarray) -> np.ndarray:
    pts = uv.reshape(1, 1, 2).astype(np.float64)
    und_norm = cv2.undistortPoints(pts, calib.K, calib.dist, P=None)
    x = float(und_norm[0, 0, 0])
    y = float(und_norm[0, 0, 1])
    ray = np.array([x, y, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)
    return ray


def apply_axis_conversion(ray_cv: np.ndarray, A_cam_from_cv: Optional[np.ndarray]) -> np.ndarray:
    if A_cam_from_cv is None:
        return ray_cv
    out = A_cam_from_cv @ ray_cv
    out /= np.linalg.norm(out)
    return out


def ray_camera_to_field(pose: CameraPoseField, ray_c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    d_f = pose.R_f_c @ ray_c
    d_f = d_f / np.linalg.norm(d_f)
    return pose.p_f, d_f


def intersect_ray_with_plane(o: np.ndarray, d: np.ndarray, n: np.ndarray, p0: np.ndarray) -> Optional[np.ndarray]:
    denom = float(n @ d)
    if abs(denom) < 1e-9:
        return None
    t = float(n @ (p0 - o)) / denom
    if t <= 0.0:
        return None
    return o + t * d


def intersect_ray_with_plane_z(o_f: np.ndarray, d_f: np.ndarray, z_plane: float) -> Optional[np.ndarray]:
    n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    p0 = np.array([0.0, 0.0, z_plane], dtype=np.float64)
    return intersect_ray_with_plane(o_f, d_f, n, p0)


def estimate_fuel_center_field(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    ball_radius_m: float = 0.15,
    A_cam_from_cv: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    uv = bbox_top_center_pixel(bbox)
    ray_cv = pixel_to_unit_ray_camera_opencv(calib, uv)
    ray_c = apply_axis_conversion(ray_cv, A_cam_from_cv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)
    return intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius_m)


def estimate_fuel_center_field_debug(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    ball_radius_m: float = 0.15,
    A_cam_from_cv: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    uv = bbox_top_center_pixel(bbox)
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
    if abs(dz) < 1e-9:
        print("t:", None)
        print("pt:", None)
        return None

    t = (ball_radius_m - float(o_f[2])) / dz
    print("t:", t)

    pt = intersect_ray_with_plane_z(o_f, d_f, ball_radius_m)
    # print(fmt_xyz(pt, "pt"))
    return pt


if __name__ == "__main__":
    np.set_printoptions(
        precision=3,      # decimals
        suppress=True,    # no scientific notation for small numbers
        linewidth=120,    # wrap width
        edgeitems=3,      # show first/last items for long arrays
        threshold=1000    # how many items before it starts summarizing with "..."
    )

    K = np.array([[950, 0, 640], [0, 950, 360], [0, 0, 1]], dtype=np.float64)
    dist = np.array([0.1, -0.05, 0.0, 0.0, 0.0], dtype=np.float64)
    calib = CameraCalibration(K=K, dist=dist)

    bbox = YoloBBox(x1=600, y1=250, x2=680, y2=330)

    A_cam_from_cv = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)

    p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)

    yaw_deg = 0.0
    pitch_down_deg = 30.0
    roll_deg = 0.0

    R_f_c = R_field_from_cam_ypr(yaw_deg=yaw_deg, pitch_deg=pitch_down_deg, roll_deg=roll_deg)
    pose = CameraPoseField(R_f_c=R_f_c, p_f=p_f)

    estimate_fuel_center_field_debug(bbox, calib, pose, ball_radius_m=0.15, A_cam_from_cv=A_cam_from_cv)

    tracker = EMA3D(alpha=0.25)
    pt = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv)
