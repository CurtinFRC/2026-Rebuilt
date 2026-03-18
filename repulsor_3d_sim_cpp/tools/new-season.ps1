param(
  [Parameter(Mandatory = $true)]
  [string]$SeasonName
)

$root = Split-Path -Parent $PSScriptRoot
$includeDir = Join-Path $root "include/repulsor3d/render"
$sourceDir = Join-Path $root "src/render"

$headerPath = Join-Path $includeDir "$SeasonName`ModelBuilder.hpp"
$sourcePath = Join-Path $sourceDir "$SeasonName`ModelBuilder.cpp"

if (Test-Path $headerPath -or Test-Path $sourcePath) {
  Write-Error "Season model builder files already exist for '$SeasonName'"
  exit 1
}

$header = @"
#pragma once

#include "repulsor3d/render/SceneModelBuilder.hpp"

namespace repulsor3d {

class ${SeasonName}ModelBuilder final : public ISceneModelBuilder {
 public:
  explicit ${SeasonName}ModelBuilder(const ViewerConfig& cfg);
  RenderSceneFrame BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) override;

 private:
  ViewerConfig cfg_;
};

}  // namespace repulsor3d
"@

$source = @"
#include "repulsor3d/render/${SeasonName}ModelBuilder.hpp"

namespace repulsor3d {

${SeasonName}ModelBuilder::${SeasonName}ModelBuilder(const ViewerConfig& cfg) : cfg_(cfg) {}

RenderSceneFrame ${SeasonName}ModelBuilder::BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) {
  (void)bundle;
  (void)toggles;
  RenderSceneFrame frame;
  frame.overlayLines.push_back("TODO: implement ${SeasonName} model builder");
  return frame;
}

}  // namespace repulsor3d
"@

New-Item -ItemType Directory -Force -Path $includeDir | Out-Null
New-Item -ItemType Directory -Force -Path $sourceDir | Out-Null

Set-Content -Path $headerPath -Value $header -Encoding UTF8
Set-Content -Path $sourcePath -Value $source -Encoding UTF8

Write-Host "Created:"
Write-Host "  $headerPath"
Write-Host "  $sourcePath"
Write-Host ""
Write-Host "Next:"
Write-Host "  1. Add the new source file to CMakeLists.txt"
Write-Host "  2. Register it in SceneModelBuilderFactory"

