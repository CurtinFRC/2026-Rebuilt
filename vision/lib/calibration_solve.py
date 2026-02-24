from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _collect_images(images_dir: Path) -> list[Path]:
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    files = [p for p in images_dir.iterdir() if p.is_file() and p.suffix.lower() in exts]
    files.sort()
    return files


def _detect(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
    if found and corners is not None:
        return True, corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found2, corners2 = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found2 or corners2 is None:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    cv2.cornerSubPix(gray, corners2, (11, 11), (-1, -1), criteria)
    return True, corners2


def _dict_by_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is required for charuco")
    code = getattr(cv2.aruco, str(name).strip().upper(), None)
    if code is None:
        raise ValueError(f"unsupported aruco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(int(code))


def _make_charuco_board(cols: int, rows: int, marker_ratio: float, dict_name: str):
    dictionary = _dict_by_name(dict_name)
    sx = int(cols) + 1
    sy = int(rows) + 1
    mr = float(marker_ratio)
    if mr <= 0.0 or mr >= 1.0:
        raise ValueError("marker-ratio must be in (0, 1)")
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard((sx, sy), 1.0, mr, dictionary)
    else:
        board = cv2.aruco.CharucoBoard_create(sx, sy, 1.0, mr, dictionary)
    return board, dictionary


def _mean_reprojection_error(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    K: np.ndarray,
    dist: np.ndarray,
) -> float:
    total = 0.0
    count = 0
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj2 = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
        img2 = np.asarray(img, dtype=np.float64).reshape(-1, 2)
        if proj2.shape != img2.shape:
            continue
        err = np.linalg.norm(proj2 - img2, axis=1)
        total += float(np.sum(err))
        count += int(err.size)
    if count <= 0:
        return 0.0
    return total / float(count)


def _mean_reprojection_error_charuco(
    board,
    corners_list: list[np.ndarray],
    ids_list: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    K: np.ndarray,
    dist: np.ndarray,
) -> float:
    all_pts = np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)
    total = 0.0
    count = 0
    for corners, ids, rvec, tvec in zip(corners_list, ids_list, rvecs, tvecs):
        if corners is None or ids is None:
            continue
        idx = np.asarray(ids, dtype=np.int32).reshape(-1)
        if idx.size <= 0:
            continue
        obj = all_pts[idx]
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj2 = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
        img2 = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        if proj2.shape != img2.shape:
            continue
        err = np.linalg.norm(proj2 - img2, axis=1)
        total += float(np.sum(err))
        count += int(err.size)
    if count <= 0:
        return 0.0
    return total / float(count)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", type=str, default="charuco", choices=["charuco", "checkerboard"])
    parser.add_argument("--images-dir", type=str, default="calib_images")
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--marker-ratio", type=float, default=0.70)
    parser.add_argument("--aruco-dict", type=str, default="DICT_5X5_1000")
    parser.add_argument("--min-charuco-corners", type=int, default=14)
    parser.add_argument("--out-json", type=str, default="calibration.json")
    parser.add_argument("--show-used", action="store_true")
    args = parser.parse_args()

    cols = int(args.cols)
    rows = int(args.rows)
    if cols < 2 or rows < 2:
        raise ValueError("rows and cols must be >= 2")
    square_m = float(args.square_mm) / 1000.0
    if square_m <= 0.0:
        raise ValueError("square-mm must be > 0")

    images_dir = Path(args.images_dir).resolve()
    if not images_dir.exists():
        raise FileNotFoundError(f"images dir not found: {images_dir}")
    files = _collect_images(images_dir)
    if not files:
        raise FileNotFoundError(f"no images found in: {images_dir}")

    pattern = (cols, rows)
    mode = str(args.pattern).strip().lower()
    obj_template = np.zeros((rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    obj_template[:, :2] = grid * np.float32(square_m)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    charuco_corners_list: list[np.ndarray] = []
    charuco_ids_list: list[np.ndarray] = []
    used_files: list[Path] = []
    image_size: tuple[int, int] | None = None
    board = None
    dictionary = None
    if mode == "charuco":
        board, dictionary = _make_charuco_board(
            cols=cols,
            rows=rows,
            marker_ratio=float(args.marker_ratio),
            dict_name=str(args.aruco_dict),
        )

    for p in files:
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None or img.size == 0:
            continue
        h, w = img.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if mode == "charuco":
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if marker_ids is None or len(marker_ids) <= 0:
                continue
            retval, cc, ci = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
            if retval is None or float(retval) < float(max(4, int(args.min_charuco_corners))) or cc is None or ci is None:
                continue
            charuco_corners_list.append(cc.astype(np.float32))
            charuco_ids_list.append(ci.astype(np.int32))
            used_files.append(p)
        else:
            found, corners = _detect(gray, pattern)
            if not found or corners is None:
                continue
            object_points.append(obj_template.copy())
            image_points.append(corners.astype(np.float32))
            used_files.append(p)

    if image_size is None:
        raise RuntimeError("no readable images found")
    if mode == "charuco":
        if len(charuco_corners_list) < 8:
            raise RuntimeError(f"need at least 8 good images, got {len(charuco_corners_list)}")
    else:
        if len(object_points) < 8:
            raise RuntimeError(f"need at least 8 good images, got {len(object_points)}")

    flags = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 120, 1e-9)
    if mode == "charuco":
        rms, K, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charuco_corners_list,
            charuco_ids_list,
            board,
            image_size,
            None,
            None,
            flags=flags,
            criteria=criteria,
        )
        mean_err = _mean_reprojection_error_charuco(board, charuco_corners_list, charuco_ids_list, rvecs, tvecs, K, dist)
        used_count = len(charuco_corners_list)
    else:
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
            flags=flags,
            criteria=criteria,
        )
        mean_err = _mean_reprojection_error(object_points, image_points, rvecs, tvecs, K, dist)
        used_count = len(object_points)

    result = {
        "pattern": mode,
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "images_total": int(len(files)),
        "images_used": int(used_count),
        "rms_reprojection_error": float(rms),
        "mean_reprojection_error_px": float(mean_err),
        "K": np.asarray(K, dtype=np.float64).reshape(3, 3).tolist(),
        "dist": np.asarray(dist, dtype=np.float64).reshape(-1).tolist(),
        "runtime_camera_calibration": {
            "calibration": {
                "K": np.asarray(K, dtype=np.float64).reshape(3, 3).tolist(),
                "dist": np.asarray(dist, dtype=np.float64).reshape(-1).tolist(),
            }
        },
    }
    if bool(args.show_used):
        result["used_files"] = [str(p) for p in used_files]

    out_path = Path(args.out_json).resolve()
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(str(out_path))
    print(json.dumps(result["runtime_camera_calibration"], indent=2))


if __name__ == "__main__":
    main()
