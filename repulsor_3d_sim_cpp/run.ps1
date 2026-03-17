param(
  [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location $root
try {
  cmake -S . -B build -DREPULSOR_SIM_FETCH_DEPS=ON -DREPULSOR_SIM_USE_NTCORE=ON
  cmake --build build --config $Config

  $exe = Join-Path $root "build/$Config/repulsor_3d_sim_cpp.exe"
  if (-not (Test-Path $exe)) {
    throw "Executable not found: $exe"
  }

  & $exe
} finally {
  Pop-Location
}
