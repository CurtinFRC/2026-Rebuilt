param(
  [ValidateSet("Debug", "Release")]
  [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptRoot
$buildDir = Join-Path $projectRoot "build"

cmake --build $buildDir --config $Config --target repulsor_3d_sim_cpp_render_bench
ctest --test-dir $buildDir -C $Config --output-on-failure -R repulsor_3d_sim_cpp_render_bench

