from __future__ import annotations

import unittest
from typing import Optional, Tuple

import numpy as np
import cv2

from lib.fuel_estimator import (
    CameraCalibration,
    CameraPoseField,
    YoloBBox,
    EMA3D,
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


def _bbox_from_uv(uv: np.ndarray, h: float = 80.0, w: float = 80.0) -> YoloBBox:
    u = float(uv[0])
    v = float(uv[1])
    return YoloBBox(x1=u - 0.5 * w, y1=v, x2=u + 0.5 * w, y2=v + h)


def _o_d_from_bbox(
    bbox: YoloBBox,
    calib: CameraCalibration,
    pose: CameraPoseField,
    A_cam_from_cv: Optional[np.ndarray],
    frac_from_top: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    uv = bbox_top_center_pixel(bbox, frac_from_top=frac_from_top)
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


def _rand_rot_ypr(rng: np.random.Generator) -> np.ndarray:
    yaw = float(rng.uniform(-180.0, 180.0))
    pitch = float(rng.uniform(-70.0, 70.0))
    roll = float(rng.uniform(-180.0, 180.0))
    return R_field_from_cam_ypr(yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll)


class TestFuelEstimator(unittest.TestCase):
    def test_bbox_top_center_pixel_frac_basic(self):
        b = YoloBBox(x1=10.0, y1=20.0, x2=30.0, y2=60.0)
        uv0 = bbox_top_center_pixel(b, frac_from_top=0.0)
        uv1 = bbox_top_center_pixel(b, frac_from_top=1.0)
        uv05 = bbox_top_center_pixel(b, frac_from_top=0.5)
        self.assertTrue(np.allclose(uv0, np.array([20.0, 20.0], dtype=np.float64), atol=0.0))
        self.assertTrue(np.allclose(uv1, np.array([20.0, 60.0], dtype=np.float64), atol=0.0))
        self.assertTrue(np.allclose(uv05, np.array([20.0, 40.0], dtype=np.float64), atol=0.0))

    def test_bbox_top_center_pixel_frac_clamps(self):
        b = YoloBBox(x1=10.0, y1=20.0, x2=30.0, y2=60.0)
        uv_neg = bbox_top_center_pixel(b, frac_from_top=-2.0)
        uv_big = bbox_top_center_pixel(b, frac_from_top=3.0)
        uv0 = bbox_top_center_pixel(b, frac_from_top=0.0)
        uv1 = bbox_top_center_pixel(b, frac_from_top=1.0)
        self.assertTrue(np.allclose(uv_neg, uv0, atol=0.0))
        self.assertTrue(np.allclose(uv_big, uv1, atol=0.0))

    def test_EMA3D_update_behavior(self):
        ema = EMA3D(alpha=0.25)
        self.assertIsNone(ema.update(None))
        a = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        b = np.array([5.0, 6.0, 7.0], dtype=np.float64)
        out1 = ema.update(a)
        assert out1 is not None
        self.assertTrue(np.allclose(out1, a, atol=0.0))
        out2 = ema.update(b)
        assert out2 is not None
        expected = (1.0 - 0.25) * a + 0.25 * b
        self.assertTrue(np.allclose(out2, expected, atol=0.0))
        out3 = ema.update(None)
        assert out3 is not None
        self.assertTrue(np.allclose(out3, expected, atol=0.0))

    def test_apply_axis_conversion_none_is_identity(self):
        rng = _rng(1)
        for _ in range(200):
            v = rng.normal(size=(3,))
            out = apply_axis_conversion(v, None)
            self.assertTrue(np.allclose(out, np.asarray(v, dtype=np.float64).reshape(3), atol=0.0))

    def test_apply_axis_conversion_preserves_norm_if_orthonormal(self):
        rng = _rng(2)
        A = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        for _ in range(2000):
            v = _normalize(rng.normal(size=(3,)))
            out = apply_axis_conversion(v, A)
            self.assertAlmostEqual(float(np.linalg.norm(out)), 1.0, places=12)

    def test_R_field_from_cam_ypr_orthonormal_many(self):
        rng = _rng(3)
        for _ in range(2000):
            R = _rand_rot_ypr(rng)
            I = R.T @ R
            self.assertTrue(np.allclose(I, np.eye(3), atol=5e-12))
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=10)

    def test_ray_camera_to_field_inverse_rotation_roundtrip(self):
        rng = _rng(4)
        for _ in range(800):
            R = _rand_rot_ypr(rng)
            p = rng.uniform(-5.0, 5.0, size=(3,)).astype(np.float64)
            pose = CameraPoseField(R_f_c=R, p_f=p)
            ray_c = _normalize(rng.normal(size=(3,)))
            o_f, d_f = ray_camera_to_field(pose, ray_c)
            self.assertTrue(np.allclose(o_f, p, atol=0.0))
            ray_c_back = _inv3(R) @ d_f
            self.assertLess(_angle(ray_c, ray_c_back), 2e-7)

    def test_intersect_ray_with_plane_z_parallel_returns_none(self):
        o = np.array([0.0, 0.0, 0.5], dtype=np.float64)
        d = _normalize(np.array([1.0, 0.0, 0.0], dtype=np.float64))
        p = intersect_ray_with_plane_z(o, d, z_plane=0.2)
        self.assertIsNone(p)

    def test_intersect_ray_with_plane_z_on_plane_returns_none_by_convention(self):
        o = np.array([1.0, 2.0, 0.15], dtype=np.float64)
        d = _normalize(np.array([0.2, -0.1, 0.9], dtype=np.float64))
        p = intersect_ray_with_plane_z(o, d, z_plane=0.15)
        self.assertIsNone(p)

    def test_intersect_ray_with_plane_z_behind_returns_none(self):
        o = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        d = _normalize(np.array([0.0, 0.0, 1.0], dtype=np.float64))
        p = intersect_ray_with_plane_z(o, d, z_plane=0.2)
        self.assertIsNone(p)

    def test_pixel_to_unit_ray_returns_unit_norm_many_pixels(self):
        K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)
        rng = _rng(5)
        for _ in range(6000):
            uv = np.array([float(rng.uniform(0.0, 1279.0)), float(rng.uniform(0.0, 719.0))], dtype=np.float64)
            ray = pixel_to_unit_ray_camera_opencv(calib, uv)
            self.assertAlmostEqual(float(np.linalg.norm(ray)), 1.0, places=12)

    def test_pixel_to_unit_ray_roundtrip_no_distortion_centerish(self):
        K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)
        rng = _rng(6)

        for _ in range(5000):
            x = float(rng.uniform(-0.6, 0.6))
            y = float(rng.uniform(-0.45, 0.45))
            ray_cv = _normalize(np.array([x, y, 1.0], dtype=np.float64))
            depth = float(rng.uniform(2.0, 12.0))
            uv = _project_point_cv(K, dist, ray_cv * depth)
            ray_cv_back = pixel_to_unit_ray_camera_opencv(calib, uv)
            self.assertLess(_angle(ray_cv, ray_cv_back), 5e-8)

    def test_pixel_to_unit_ray_roundtrip_with_distortion_centerish(self):
        K = np.array([[920.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.12, -0.08, 0.001, -0.001, 0.02], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)
        rng = _rng(7)

        for _ in range(9000):
            x = float(rng.uniform(-0.3, 0.3))
            y = float(rng.uniform(-0.25, 0.25))
            ray_cv = _normalize(np.array([x, y, 1.0], dtype=np.float64))
            depth = float(rng.uniform(2.0, 10.0))
            uv = _project_point_cv(K, dist, ray_cv * depth)
            ray_cv_back = pixel_to_unit_ray_camera_opencv(calib, uv)
            self.assertLess(_angle(ray_cv, ray_cv_back), 2e-3)

    def test_estimator_matches_manual_grid_with_frac(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.1, -0.05, 0.0, 0.0, 0.0], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        ball_radius = 0.15

        p_f = np.array([2.0, 1.0, 0.7], dtype=np.float64)
        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0),
            p_f=p_f,
        )

        hits = 0
        for frac_from_top in [0.0, 0.1, 0.2]:
            for u in np.linspace(520.0, 760.0, 11):
                for v in np.linspace(220.0, 520.0, 9):
                    bbox = _bbox_from_uv(np.array([u, v], dtype=np.float64), h=80.0, w=80.0)
                    est = estimate_fuel_center_field(
                        bbox,
                        calib,
                        pose,
                        ball_radius_m=ball_radius,
                        A_cam_from_cv=A_cam_from_cv,
                        frac_from_top=frac_from_top,
                    )
                    _, _, o_f, d_f = _o_d_from_bbox(bbox, calib, pose, A_cam_from_cv, frac_from_top)
                    man = intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius)

                    if man is None:
                        self.assertIsNone(est)
                        continue

                    self.assertIsNotNone(est)
                    assert est is not None
                    hits += 1
                    self.assertAlmostEqual(float(est[2]), float(ball_radius), places=7)
                    self.assertTrue(_colinear(o_f, d_f, est, atol=2e-4))
                    self.assertTrue(np.allclose(est, man, atol=1e-7))
        self.assertGreaterEqual(hits, 10)

    def test_estimator_deterministic_same_inputs(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.05, -0.02, 0.001, 0.0, 0.0], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)

        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=12.0, pitch_deg=25.0, roll_deg=-3.0),
            p_f=np.array([2.0, -1.0, 0.9], dtype=np.float64),
        )

        bbox = YoloBBox(x1=600.0, y1=250.0, x2=680.0, y2=330.0)
        a = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.15)
        b = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.15)
        if a is None or b is None:
            self.assertTrue(a is None and b is None)
        else:
            self.assertTrue(np.allclose(a, b, atol=0.0))

    def test_estimator_invariance_to_bbox_width_same_center(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=10.0, pitch_deg=25.0, roll_deg=-5.0),
            p_f=np.array([2.0, 1.0, 0.7], dtype=np.float64),
        )

        rng = _rng(8)
        frac_from_top = 0.12
        ball_radius = 0.15

        for _ in range(1200):
            u = float(rng.uniform(520.0, 760.0))
            v = float(rng.uniform(220.0, 520.0))
            h = float(rng.uniform(20.0, 140.0))
            w1 = float(rng.uniform(2.0, 220.0))
            w2 = float(rng.uniform(2.0, 220.0))
            bbox1 = YoloBBox(x1=u - 0.5 * w1, y1=v, x2=u + 0.5 * w1, y2=v + h)
            bbox2 = YoloBBox(x1=u - 0.5 * w2, y1=v, x2=u + 0.5 * w2, y2=v + h)

            est1 = estimate_fuel_center_field(bbox1, calib, pose, ball_radius, A_cam_from_cv=A_cam_from_cv, frac_from_top=frac_from_top)
            est2 = estimate_fuel_center_field(bbox2, calib, pose, ball_radius, A_cam_from_cv=A_cam_from_cv, frac_from_top=frac_from_top)

            if est1 is None or est2 is None:
                self.assertTrue(est1 is None and est2 is None)
            else:
                self.assertTrue(np.allclose(est1, est2, atol=1e-9))

    def test_estimator_changes_with_bbox_height_when_frac_not_zero(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0),
            p_f=np.array([2.0, 1.0, 0.7], dtype=np.float64),
        )

        uv = np.array([650.0, 260.0], dtype=np.float64)
        b1 = _bbox_from_uv(uv, h=40.0, w=80.0)
        b2 = _bbox_from_uv(uv, h=140.0, w=80.0)

        frac = 0.2
        p1 = estimate_fuel_center_field(b1, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=frac)
        p2 = estimate_fuel_center_field(b2, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=frac)

        if p1 is None or p2 is None:
            self.assertTrue(p1 is None and p2 is None)
        else:
            self.assertFalse(np.allclose(p1, p2, atol=1e-12))
            self.assertAlmostEqual(float(p1[2]), 0.15, places=9)
            self.assertAlmostEqual(float(p2[2]), 0.15, places=9)

    def test_estimator_min_abs_dz_gate(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0),
            p_f=np.array([2.0, 1.0, 0.7], dtype=np.float64),
        )

        bbox = _bbox_from_uv(np.array([640.0, 360.0], dtype=np.float64), h=80.0, w=80.0)

        est_ok = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1, min_abs_dz=0.02)
        est_block = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1, min_abs_dz=0.999)

        self.assertTrue(est_ok is None or isinstance(est_ok, np.ndarray))
        self.assertIsNone(est_block)

    def test_estimator_max_range_gate(self):
        K = np.array([[950.0, 0.0, 640.0], [0.0, 950.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((5,), dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        pose = CameraPoseField(
            R_f_c=R_field_from_cam_ypr(yaw_deg=0.0, pitch_deg=30.0, roll_deg=0.0),
            p_f=np.array([2.0, 1.0, 0.7], dtype=np.float64),
        )

        bbox = _bbox_from_uv(np.array([640.0, 250.0], dtype=np.float64), h=80.0, w=80.0)

        est = estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1)
        if est is None:
            self.assertIsNone(estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1, max_range_m=0.5))
        else:
            o_f, _ = ray_camera_to_field(pose, apply_axis_conversion(pixel_to_unit_ray_camera_opencv(calib, bbox_top_center_pixel(bbox, 0.1)), A_cam_from_cv))
            dist_m = float(np.linalg.norm(est - o_f))
            self.assertIsNone(estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1, max_range_m=max(0.01, dist_m - 1e-6)))
            self.assertIsNotNone(estimate_fuel_center_field(bbox, calib, pose, 0.15, A_cam_from_cv=A_cam_from_cv, frac_from_top=0.1, max_range_m=dist_m + 1e-6))

    def test_estimator_matches_manual_many_random_poses_many_pixels(self):
        rng = _rng(9)
        K = np.array([[930.0, 0.0, 640.0], [0.0, 935.0, 360.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([0.08, -0.03, 0.001, -0.0007, 0.004], dtype=np.float64)
        calib = CameraCalibration(K=K, dist=dist)

        A_cam_from_cv = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        ball_radius = 0.15

        for _ in range(120):
            p_f = rng.uniform(-4.0, 4.0, size=(3,)).astype(np.float64)
            p_f[2] = float(rng.uniform(0.4, 1.8))
            pose = CameraPoseField(R_f_c=_rand_rot_ypr(rng), p_f=p_f)

            for _ in range(80):
                u = float(rng.uniform(80.0, 1200.0))
                v = float(rng.uniform(40.0, 700.0))
                frac_from_top = float(rng.uniform(0.0, 0.25))
                bbox = _bbox_from_uv(
                    np.array([u, v], dtype=np.float64),
                    h=float(rng.uniform(20.0, 200.0)),
                    w=float(rng.uniform(10.0, 240.0)),
                )

                est = estimate_fuel_center_field(
                    bbox,
                    calib,
                    pose,
                    ball_radius_m=ball_radius,
                    A_cam_from_cv=A_cam_from_cv,
                    frac_from_top=frac_from_top,
                )
                _, _, o_f, d_f = _o_d_from_bbox(bbox, calib, pose, A_cam_from_cv, frac_from_top)
                man = intersect_ray_with_plane_z(o_f, d_f, z_plane=ball_radius)

                if man is None:
                    self.assertIsNone(est)
                else:
                    self.assertIsNotNone(est)
                    assert est is not None
                    self.assertTrue(np.allclose(est, man, atol=3e-6))
                    self.assertAlmostEqual(float(est[2]), float(ball_radius), places=6)


if __name__ == "__main__":
    unittest.main()
