#pragma once

#include <cstddef>

namespace repulsor3d::cad {

enum class ShadowQuality {
  Low,
  Medium,
  High,
  Ultra,
};

struct CadLodPolicy {
  std::size_t maxLodCount = 2;
  float reductionRatio = 0.70F;
  std::size_t minVertexCount = 500000;
  std::size_t lod0VertexCap = 4500000;
  bool keepOriginalLod0 = true;
  std::size_t maxPreferredDrawIndices = 50000000;
  int normalBins = 20;
  bool preserveFlatShading = true;
  float lod0ScreenRadiusPx = 5200.0F;
  float screenRadiusDecay = 0.70F;
};

struct CadVisualPolicy {
  bool enableFrustumCulling = true;
  float frustumCullMarginM = 1.00F;
  bool enableBroadphase = true;
  float broadphaseCellSizeM = 3.0F;
  bool enableClusterCulling = true;
  bool fieldDisableClusterCulling = false;
  bool fieldForceLod0 = true;
  bool optimizeVertexCache = true;
  int clusterTargetIndices = 32768;
  int clusterMinIndices = 8192;
  int clusterMergeGapIndices = 512;
  int fieldClusterMaxVisibleIndices = 0;
  bool fieldClusterAdaptiveBudget = false;
  float fieldClusterAdaptiveMinScale = 0.55F;
  float minClusterScreenRadiusPx = 0.0F;
  bool enableDepthPrepass = true;
  bool enableOcclusionCulling = false;
  int hzbTilesX = 64;
  int hzbTilesY = 36;
  float hzbDepthMargin = 0.02F;
  float hzbMaxCullRadiusPx = 96.0F;
  float hzbMinCoverage = 0.92F;
  float hzbMinOccluderRadiusPx = 16.0F;
  int maxIndicesPerDrawCall = 0;
  float keyLightIntensity = 1.10F;
  float fillLightIntensity = 0.24F;
  float ambientStrength = 0.08F;
  float depthCueStrength = 0.08F;
  float specularStrength = 0.50F;
  float rimStrength = 0.03F;
  float roughness = 0.68F;
  float metallic = 0.00F;
  float exposure = 0.72F;
  float fogDensity = 0.0018F;
  float saturation = 1.14F;
  float gamma = 2.2F;
  float weatheringStrength = 0.28F;
  float weatheringScale = 0.42F;
  float detailRoughnessStrength = 0.18F;
  float clearcoatStrength = 0.22F;
  float shadowTintStrength = 0.35F;
  float themePrimaryR = 0.08F;
  float themePrimaryG = 0.66F;
  float themePrimaryB = 1.00F;
  float themeSecondaryR = 1.00F;
  float themeSecondaryG = 0.34F;
  float themeSecondaryB = 0.20F;
  float themeMix = 0.52F;
  float themeEmissiveStrength = 1.00F;
  bool nightMode = false;
  float nightIntensity = 0.68F;
  float postFxStrength = 1.00F;
  float filmGrainStrength = 0.030F;
  float vignetteStrength = 0.28F;
  float chromaticStrength = 0.025F;
  // 0 = realistic CAD, 1 = arcade, 2 = fast flat shading.
  int shadingMode = 1;
  float arcadeToonBandCount = 3.0F;
  float arcadeRimBoost = 1.95F;
  float arcadeOutlineStrength = 0.62F;
  float arcadeThemeTintStrength = 0.88F;
  float arcadeEmissiveBoost = 1.75F;
  float arcadeShadowLift = 0.48F;
  float arcadeFogScale = 0.45F;
  float arcadeBloomStrength = 0.45F;
  float arcadeStatePulseRate = 1.0F;
  float arcadeAlert = 0.0F;
  float arcadeSelected = 0.0F;
  float arcadeCharged = 0.0F;
  float arcadeHighlightBoost = 1.95F;
  float arcadeMotionBoost = 0.45F;
  bool arcadeEnableOutlinePass = false;
  float arcadeOutlineWidthM = 0.012F;
  float arcadeOutlineOpacity = 0.78F;
  bool arcadeDisableWeathering = true;
  bool arcadeHardSpecular = true;
  bool enableInstanceUploadDedup = true;
  bool enableInstanceBufferOrphaning = false;
};

struct CadShadowPolicy {
  bool enabled = true;
  int mapSize = 2048;
  float strength = 0.62F;
  ShadowQuality quality = ShadowQuality::High;
  int pcfRadius = 2;
  int cascadeCount = 2;
  float cascadeSplitDistanceM = 11.0F;
  float cascadeBlendRangeM = 2.5F;
};

int GetEnvInt(const char* name, int fallback);
float GetEnvFloat(const char* name, float fallback);
bool GetEnvBool(const char* name, bool fallback);

const CadLodPolicy& LoadCadLodPolicy();
const CadVisualPolicy& LoadCadVisualPolicy();
const CadShadowPolicy& LoadCadShadowPolicy();

}  // namespace repulsor3d::cad
