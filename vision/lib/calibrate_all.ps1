$ErrorActionPreference = "Stop"

$Pattern = "charuco"
$Camera = 0
$Width = 1280
$Height = 720
$Fps = 20.0
$Cols = 9
$Rows = 6
$SquareMm = 25.0
$MarkerRatio = 0.70
$ArucoDict = "DICT_5X5_1000"
$MinCharucoCorners = 14
$OutDir = "calib_images"
$Prefix = "calib"
$Ext = "png"
$IntervalS = 1.0
$TargetCount = 50
$MinSharpness = 120.0
$MinCoverage = 0.01
$MinDiversity = 0.10
$MaxPerViewError = 0.90
$MadMult = 2.2
$MinViewsAfterPrune = 16
$DistModel = "rational"
$ZeroTangent = $false
$AutoCapture = $true
$ManualIgnoreQuality = $false
$ShowUsed = $true
$OutJson = "calibration.json"

$captureScript = Join-Path $PSScriptRoot "calibration_capture.py"
$solveScript = Join-Path $PSScriptRoot "calibration_solve.py"

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
  "--min-sharpness", "$MinSharpness",
  "--min-coverage", "$MinCoverage",
  "--min-diversity", "$MinDiversity",
  "--out-dir", "$OutDir",
  "--prefix", "$Prefix",
  "--ext", "$Ext",
  "--interval-s", "$IntervalS",
  "--target-count", "$TargetCount"
)
if ($AutoCapture) {
  $captureArgs += "--auto"
}
if ($ManualIgnoreQuality) {
  $captureArgs += "--manual-ignore-quality"
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
  "--min-sharpness", "$MinSharpness",
  "--min-coverage", "$MinCoverage",
  "--max-per-view-error", "$MaxPerViewError",
  "--mad-mult", "$MadMult",
  "--min-views-after-prune", "$MinViewsAfterPrune",
  "--dist-model", "$DistModel",
  "--out-json", "$OutJson"
)
if ($ZeroTangent) {
  $solveArgs += "--zero-tangent"
}
if ($ShowUsed) {
  $solveArgs += "--show-used"
}
python @solveArgs
exit $LASTEXITCODE
