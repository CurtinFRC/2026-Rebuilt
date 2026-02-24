#!/usr/bin/env bash
set -euo pipefail

PATTERN="charuco"
COLS=9
ROWS=6
SQUARE_MM=25.0
MARKER_RATIO=0.70
ARUCO_DICT="DICT_5X5_1000"
PAGE="A4"
ORIENTATION="auto"
MARGIN_MM=10.0
RENDER_DPI=400
OUT_PDF="calibration_board.pdf"
OUT_PNG="calibration_board.png"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python "$SCRIPT_DIR/calibration_board_pdf.py" \
  --pattern "$PATTERN" \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-mm "$SQUARE_MM" \
  --marker-ratio "$MARKER_RATIO" \
  --aruco-dict "$ARUCO_DICT" \
  --page "$PAGE" \
  --orientation "$ORIENTATION" \
  --margin-mm "$MARGIN_MM" \
  --render-dpi "$RENDER_DPI" \
  --out-pdf "$OUT_PDF" \
  --out-png "$OUT_PNG"
