$ErrorActionPreference = "Stop"

$Pattern = "charuco"
$Cols = 9
$Rows = 6
$SquareMm = 25.0
$MarkerRatio = 0.70
$ArucoDict = "DICT_5X5_1000"
$Page = "A4"
$Orientation = "auto"
$MarginMm = 10.0
$RenderDpi = 400
$OutPdf = "calibration_board.pdf"
$OutPng = "calibration_board.png"

$boardScript = Join-Path $PSScriptRoot "calibration_board_pdf.py"

$boardArgs = @(
  $boardScript,
  "--pattern", "$Pattern",
  "--cols", "$Cols",
  "--rows", "$Rows",
  "--square-mm", "$SquareMm",
  "--marker-ratio", "$MarkerRatio",
  "--aruco-dict", "$ArucoDict",
  "--page", "$Page",
  "--orientation", "$Orientation",
  "--margin-mm", "$MarginMm",
  "--render-dpi", "$RenderDpi",
  "--out-pdf", "$OutPdf",
  "--out-png", "$OutPng"
)

python @boardArgs
exit $LASTEXITCODE
