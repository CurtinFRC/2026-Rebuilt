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
  float frustumCullMarginM = 0.30F;
  bool enableBroadphase = true;
  float broadphaseCellSizeM = 3.0F;
  bool enableClusterCulling = true;
  bool fieldDisableClusterCulling = true;
  bool fieldForceLod0 = true;
  bool optimizeVertexCache = true;
  int clusterTargetIndices = 32768;
  int clusterMinIndices = 8192;
  int clusterMergeGapIndices = 192;
  int fieldClusterMaxVisibleIndices = 0;
  bool fieldClusterAdaptiveBudget = true;
  float fieldClusterAdaptiveMinScale = 0.55F;
  float minClusterScreenRadiusPx = 0.0F;
  bool enableDepthPrepass = false;
  bool enableOcclusionCulling = false;
  int hzbTilesX = 64;
  int hzbTilesY = 36;
  float hzbDepthMargin = 0.01F;
  float hzbMaxCullRadiusPx = 120.0F;
  int maxIndicesPerDrawCall = 500000;
  float keyLightIntensity = 1.35F;
  float fillLightIntensity = 0.32F;
  float ambientStrength = 0.10F;
  float depthCueStrength = 0.08F;
  float specularStrength = 0.50F;
  float rimStrength = 0.03F;
  float roughness = 0.68F;
  float metallic = 0.00F;
  float exposure = 0.78F;
  float fogDensity = 0.0018F;
  float saturation = 1.08F;
  float gamma = 2.2F;
  float weatheringStrength = 0.28F;
  float weatheringScale = 0.42F;
  float detailRoughnessStrength = 0.18F;
  float clearcoatStrength = 0.22F;
  float shadowTintStrength = 0.35F;
  bool enableInstanceUploadDedup = true;
  bool enableInstanceBufferOrphaning = true;
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
