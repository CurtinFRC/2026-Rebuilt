param(
  [ValidateSet("charuco","checkerboard")]
  [string]$Pattern = "charuco",
  [int]$Camera = 0,
  [int]$Width = 1280,
  [int]$Height = 720,
  [double]$Fps = 20.0,
  [int]$Cols = 9,
  [int]$Rows = 6,
  [double]$SquareMm = 25.0,
  [double]$MarkerRatio = 0.70,
  [string]$ArucoDict = "DICT_5X5_1000",
  [int]$MinCharucoCorners = 14,
  [string]$Page = "A4",
  [string]$Orientation = "auto",
  [double]$MarginMm = 10.0,
  [string]$OutDir = "calib_images",
  [string]$Prefix = "calib",
  [string]$Ext = "png",
  [double]$IntervalS = 1.0,
  [int]$TargetCount = 30,
  [string]$OutPdf = "calibration_board.pdf",
  [string]$OutPng = "calibration_board.png",
  [string]$OutJson = "calibration.json",
  [switch]$ManualCapture,
  [switch]$ShowUsed
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$boardScript = Join-Path $PSScriptRoot "calibration_board_pdf.py"
$captureScript = Join-Path $PSScriptRoot "calibration_capture.py"
$solveScript = Join-Path $PSScriptRoot "calibration_solve.py"

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
  "--out-pdf", "$OutPdf"
)
if ([string]::IsNullOrWhiteSpace($OutPng) -eq $false) {
  $boardArgs += @("--out-png", "$OutPng")
}
python @boardArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$captureArgs = @(
  $captureScript,
  "--pattern", "$Pattern",
  "--camera", "$Camera",
  "--width", "$Width",
  "--height", "$Height",
  "--fps", "$Fps",
  "--cols", "$Cols",
  "--rows", "$Rows",
  "--marker-ratio", "$MarkerRatio",
  "--aruco-dict", "$ArucoDict",
  "--min-charuco-corners", "$MinCharucoCorners",
  "--out-dir", "$OutDir",
  "--prefix", "$Prefix",
  "--ext", "$Ext",
  "--interval-s", "$IntervalS",
  "--target-count", "$TargetCount"
)
if (-not $ManualCapture.IsPresent) {
  $captureArgs += "--auto"
}
python @captureArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$solveArgs = @(
  $solveScript,
  "--pattern", "$Pattern",
  "--images-dir", "$OutDir",
  "--cols", "$Cols",
  "--rows", "$Rows",
  "--square-mm", "$SquareMm",
  "--marker-ratio", "$MarkerRatio",
  "--aruco-dict", "$ArucoDict",
  "--min-charuco-corners", "$MinCharucoCorners",
  "--out-json", "$OutJson"
)
if ($ShowUsed.IsPresent) {
  $solveArgs += "--show-used"
}
python @solveArgs
exit $LASTEXITCODE
