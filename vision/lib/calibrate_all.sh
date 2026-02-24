#!/usr/bin/env bash
set -euo pipefail

CAMERA=0
WIDTH=1280
HEIGHT=720
FPS=20.0
PATTERN="charuco"
COLS=9
ROWS=6
SQUARE_MM=25.0
MARKER_RATIO=0.70
ARUCO_DICT="DICT_5X5_1000"
MIN_CHARUCO_CORNERS=14
PAGE="A4"
ORIENTATION="auto"
MARGIN_MM=10.0
OUT_DIR="calib_images"
PREFIX="calib"
EXT="png"
INTERVAL_S=1.0
TARGET_COUNT=30
OUT_PDF="calibration_board.pdf"
OUT_PNG="calibration_board.png"
OUT_JSON="calibration.json"
MANUAL_CAPTURE=0
SHOW_USED=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pattern) PATTERN="$2"; shift 2 ;;
    --camera) CAMERA="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --height) HEIGHT="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --cols) COLS="$2"; shift 2 ;;
    --rows) ROWS="$2"; shift 2 ;;
    --square-mm) SQUARE_MM="$2"; shift 2 ;;
    --marker-ratio) MARKER_RATIO="$2"; shift 2 ;;
    --aruco-dict) ARUCO_DICT="$2"; shift 2 ;;
    --min-charuco-corners) MIN_CHARUCO_CORNERS="$2"; shift 2 ;;
    --page) PAGE="$2"; shift 2 ;;
    --orientation) ORIENTATION="$2"; shift 2 ;;
    --margin-mm) MARGIN_MM="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --ext) EXT="$2"; shift 2 ;;
    --interval-s) INTERVAL_S="$2"; shift 2 ;;
    --target-count) TARGET_COUNT="$2"; shift 2 ;;
    --out-pdf) OUT_PDF="$2"; shift 2 ;;
    --out-png) OUT_PNG="$2"; shift 2 ;;
    --out-json) OUT_JSON="$2"; shift 2 ;;
    --manual-capture) MANUAL_CAPTURE=1; shift ;;
    --show-used) SHOW_USED=1; shift ;;
    -h|--help)
      echo "usage: calibrate_all.sh [--pattern charuco|checkerboard] [--camera N] [--width W] [--height H] [--fps V] [--cols N] [--rows N] [--square-mm MM] [--marker-ratio V] [--aruco-dict NAME] [--min-charuco-corners N] [--page A4|Letter] [--orientation auto|portrait|landscape] [--margin-mm MM] [--out-dir DIR] [--prefix P] [--ext EXT] [--interval-s S] [--target-count N] [--out-pdf PATH] [--out-png PATH] [--out-json PATH] [--manual-capture] [--show-used]"
      exit 0
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

board_args=(
  "$SCRIPT_DIR/calibration_board_pdf.py"
  --pattern "$PATTERN"
  --cols "$COLS"
  --rows "$ROWS"
  --square-mm "$SQUARE_MM"
  --marker-ratio "$MARKER_RATIO"
  --aruco-dict "$ARUCO_DICT"
  --page "$PAGE"
  --orientation "$ORIENTATION"
  --margin-mm "$MARGIN_MM"
  --out-pdf "$OUT_PDF"
)
if [[ -n "$OUT_PNG" ]]; then
  board_args+=(--out-png "$OUT_PNG")
fi
python "${board_args[@]}"

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
  --out-dir "$OUT_DIR"
  --prefix "$PREFIX"
  --ext "$EXT"
  --interval-s "$INTERVAL_S"
  --target-count "$TARGET_COUNT"
)
if [[ "$MANUAL_CAPTURE" -eq 0 ]]; then
  capture_args+=(--auto)
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
  --out-json "$OUT_JSON"
)
if [[ "$SHOW_USED" -eq 1 ]]; then
  solve_args+=(--show-used)
fi
python "${solve_args[@]}"
