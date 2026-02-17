from __future__ import annotations

import unittest
from typing import Optional, Tuple

import numpy as np
import cv2

from lib.fuel_estimator import (
    CameraCalibration,
    CameraPoseField,
    YoloBBox,
    R_field_from_cam_ypr,
    bbox_top_center_pixel,
    pixel_to_unit_ray_camera_opencv,
    apply_axis_conversion,
    ray_camera_to_field,
    intersect_ray_with_plane_z,
    estimate_fuel_center_field,
)

def _rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(seed)


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(-1)
    n = float(np.linalg.norm(v))
    if n == 0.0:
        return v
    return v / n


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    a = _normalize(a)
    b = _normalize(b)
    return float(np.arccos(np.clip(float(a @ b), -1.0, 1.0)))


def _inv3(A: np.ndarray) -> np.ndarray:
    return np.linalg.inv(np.asarray(A, dtype=np.float64).reshape(3, 3))


def _project_point_cv(K: np.ndarray, dist: np.ndarray, p_cv: np.ndarray) -> np.ndarray:
    obj = np.asarray(p_cv, dtype=np.float64).reshape(1, 1, 3)
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)
    img, _ = cv2.projectPoints(
        obj,
        rvec,
        tvec,
        np.asarray(K, dtype=np.float64),
        np.asarray(dist, dtype=np.float64),
    )
    return img.reshape(2).astype(np.float64)


def _bbox_from_uv(uv: np.ndarray) -> YoloBBox:
    u = float(uv[0])
    v = float(uv[1])
    return YoloBBox(x1=u - 2.0, y1=v, x2=u + 2.0, y2=v + 4.0)


def _o_d_from_bbox(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    A_cam_from_cv: Optional[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    uv = bbox_top_center_pixel(bbox)
    ray_cv = pixel_to_unit_ray_camera_opencv(calib, uv)
    ray_c = apply_axis_conversion(ray_cv, A_cam_from_cv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)
    return uv, ray_cv, o_f, d_f


def _colinear(o: np.ndarray, d: np.ndarray, p: np.ndarray, atol: float) -> bool:
    v = np.asarray(p, dtype=np.float64).reshape(3) - np.asarray(o, dtype=np.float64).reshape(3)
    d = _normalize(np.asarray(d, dtype=np.float64).reshape(3))
    if float(v @ d) <= 0.0:
        return False
    c = np.cross(v, d)
    return float(np.linalg.norm(c)) <= float(atol)


class TestFuelEstimator(unittest.TestCase):
    def test_axis_conversion_is_orthonormal_if_given(self):
        A = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        I = A.T @ A
        self.assertTrue(np.allclose(I, np.eye(3), atol=1e-12))
        self.assertAlmostEqual(float(np.linalg.det(A)), 1.0, places=12)

    def test_ray_roundtrip_no_distortion_centerish(self):
        K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)
        rng = _rng(1)

        for _ in range(4000):
            x = float(rng.uniform(-0.6, 0.6))
            y = float(rng.uniform(-0.45, 0.45))
            ray_cv = _normalize(np.array([x, y, 1.0], dtype=np.float64))
            depth = float(rng.uniform(2.0, 12.0))
            uv = _project_point_cv(K, dist, ray_cv * depth)
            ray_cv_back = pixel_to_unit_ray_camera_opencv(calib, uv)
            self.assertLess(_angle(ray_cv, ray_cv_back), 5e-8)

    def test_ray_roundtrip_with_distortion_centerish(self):
        K = np.array([[920.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.12, -0.08, 0.001, -0.001, 0.02], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)
        rng = _rng(2)

        for _ in range(8000):
            x = float(rng.uniform(-0.3, 0.3))
            y = float(rng.uniform(-0.25, 0.25))
            ray_cv = _normalize(np.array([x, y, 1.0], dtype=np.float64))
            depth = float(rng.uniform(2.0, 10.0))
            uv = _project_point_cv(K, dist, ray_cv * depth)
            ray_cv_back = pixel_to_unit_ray_camera_opencv(calib, uv)
            self.assertLess(_angle(ray_cv, ray_cv_back), 2e-3)

    def test_estimator_properties_with_axis_no_dist(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        ball_radius = 0.15

        p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)
        R_f_c = R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0)
        pose = CameraPoseField(R_f_c=R_f_c, p_f=p_f)

        hits = 0
        for u in np.linspace(520.0, 760.0, 13):
            for v in np.linspace(220.0, 520.0, 11):
                bbox = _bbox_from_uv(np.array([u, v], dtype=np.float64))
                est = estimate_fuel_center_field(bbox, calib, pose, ball_radius, A_cam_from_cv=A_cam_from_cv)
                uv, ray_cv, o_f, d_f = _o_d_from_bbox(bbox, calib, pose, A_cam_from_cv)
                man = intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius)

                if man is None:
                    self.assertIsNone(est)
                    continue

                self.assertIsNotNone(est)
                hits += 1
                self.assertAlmostEqual(float(est[2]), float(ball_radius), places=9)
                self.assertTrue(_colinear(o_f, d_f, est, atol=1e-6))
                self.assertTrue(np.allclose(est, man, atol=1e-9))

        self.assertGreaterEqual(hits, 10)

    def test_estimator_properties_with_axis_with_dist(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.1, -0.05, 0.0, 0.0, 0.0], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        ball_radius = 0.15

        p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)
        R_f_c = R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0)
        pose = CameraPoseField(R_f_c=R_f_c, p_f=p_f)

        hits = 0
        for u in np.linspace(540.0, 740.0, 11):
            for v in np.linspace(240.0, 500.0, 9):
                bbox = _bbox_from_uv(np.array([u, v], dtype=np.float64))
                est = estimate_fuel_center_field(bbox, calib, pose, ball_radius, A_cam_from_cv=A_cam_from_cv)
                uv, ray_cv, o_f, d_f = _o_d_from_bbox(bbox, calib, pose, A_cam_from_cv)
                man = intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius)

                if man is None:
                    self.assertIsNone(est)
                    continue

                self.assertIsNotNone(est)
                hits += 1
                self.assertAlmostEqual(float(est[2]), float(ball_radius), places=7)
                self.assertTrue(_colinear(o_f, d_f, est, atol=2e-4))
                self.assertTrue(np.allclose(est, man, atol=1e-7))

        self.assertGreaterEqual(hits, 8)

    def test_without_axis_conversion_typical_pose_often_none(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        ball_radius = 0.15
        p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)
        R_f_c = R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0)
        pose = CameraPoseField(R_f_c=R_f_c, p_f=p_f)

        none_count = 0
        total = 0
        for u in np.linspace(520.0, 760.0, 13):
            for v in np.linspace(220.0, 520.0, 11):
                total += 1
                bbox = _bbox_from_uv(np.array([u, v], dtype=np.float64))
                est = estimate_fuel_center_field(bbox, calib, pose, ball_radius, A_cam_from_cv=None)
                if est is None:
                    none_count += 1

        self.assertGreaterEqual(none_count, int(0.7 * total))
