#!/usr/bin/env bash
set -euo pipefail

PATTERN="charuco"
CAMERA=0
WIDTH=1280
HEIGHT=720
FPS=20.0
COLS=9
ROWS=6
SQUARE_MM=25.0
MARKER_RATIO=0.70
ARUCO_DICT="DICT_5X5_1000"
MIN_CHARUCO_CORNERS=14
OUT_DIR="calib_images"
PREFIX="calib"
EXT="png"
INTERVAL_S=1.0
TARGET_COUNT=50
MIN_SHARPNESS=120.0
MIN_COVERAGE=0.05
MIN_DIVERSITY=0.10
MAX_PER_VIEW_ERROR=0.90
MAD_MULT=2.2
MIN_VIEWS_AFTER_PRUNE=16
DIST_MODEL="rational"
ZERO_TANGENT=0
AUTO_CAPTURE=1
MANUAL_IGNORE_QUALITY=0
SHOW_USED=1
OUT_JSON="calibration.json"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

capture_args=(
  "$SCRIPT_DIR/calibration_capture.py"
  --pattern "$PATTERN"
  --camera "$CAMERA"
  --width "$WIDTH"
  --height "$HEIGHT"
  --fps "$FPS"
  --cols "$COLS"
  --rows "$ROWS"
  --marker-ratio "$MARKER_RATIO"
  --aruco-dict "$ARUCO_DICT"
  --min-charuco-corners "$MIN_CHARUCO_CORNERS"
  --min-sharpness "$MIN_SHARPNESS"
  --min-coverage "$MIN_COVERAGE"
  --min-diversity "$MIN_DIVERSITY"
  --out-dir "$OUT_DIR"
  --prefix "$PREFIX"
  --ext "$EXT"
  --interval-s "$INTERVAL_S"
  --target-count "$TARGET_COUNT"
)
if [[ "$AUTO_CAPTURE" -eq 1 ]]; then
  capture_args+=(--auto)
fi
if [[ "$MANUAL_IGNORE_QUALITY" -eq 1 ]]; then
  capture_args+=(--manual-ignore-quality)
fi
python "${capture_args[@]}"

solve_args=(
  "$SCRIPT_DIR/calibration_solve.py"
  --pattern "$PATTERN"
  --images-dir "$OUT_DIR"
  --cols "$COLS"
  --rows "$ROWS"
  --square-mm "$SQUARE_MM"
  --marker-ratio "$MARKER_RATIO"
  --aruco-dict "$ARUCO_DICT"
  --min-charuco-corners "$MIN_CHARUCO_CORNERS"
  --min-sharpness "$MIN_SHARPNESS"
  --min-coverage "$MIN_COVERAGE"
  --max-per-view-error "$MAX_PER_VIEW_ERROR"
  --mad-mult "$MAD_MULT"
  --min-views-after-prune "$MIN_VIEWS_AFTER_PRUNE"
  --dist-model "$DIST_MODEL"
  --out-json "$OUT_JSON"
)
if [[ "$ZERO_TANGENT" -eq 1 ]]; then
  solve_args+=(--zero-tangent)
fi
if [[ "$SHOW_USED" -eq 1 ]]; then
  solve_args+=(--show-used)
fi
python "${solve_args[@]}"
