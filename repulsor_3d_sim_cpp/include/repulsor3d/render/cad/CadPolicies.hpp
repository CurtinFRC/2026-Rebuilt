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
  float reductionRatio = 0.62F;
  std::size_t minVertexCount = 700000;
  std::size_t lod0VertexCap = 4500000;
  bool keepOriginalLod0 = true;
  std::size_t maxPreferredDrawIndices = 8500000;
  int normalBins = 20;
  bool preserveFlatShading = true;
  float lod0ScreenRadiusPx = 1200.0F;
  float screenRadiusDecay = 0.5F;
};

struct CadVisualPolicy {
  bool enableFrustumCulling = true;
  bool enableBroadphase = true;
  float broadphaseCellSizeM = 3.0F;
  float keyLightIntensity = 1.35F;
  float fillLightIntensity = 0.20F;
  float ambientStrength = 0.12F;
  float depthCueStrength = 0.08F;
  float specularStrength = 0.42F;
  float rimStrength = 0.03F;
  float roughness = 0.72F;
  float metallic = 0.00F;
  float exposure = 0.86F;
  float fogDensity = 0.003F;
  float saturation = 1.00F;
  float gamma = 2.2F;
};

struct CadShadowPolicy {
  bool enabled = true;
  int mapSize = 2048;
  float strength = 0.55F;
  ShadowQuality quality = ShadowQuality::High;
  int pcfRadius = 2;
  int cascadeCount = 1;
  float cascadeSplitDistanceM = 12.0F;
};

int GetEnvInt(const char* name, int fallback);
float GetEnvFloat(const char* name, float fallback);
bool GetEnvBool(const char* name, bool fallback);

const CadLodPolicy& LoadCadLodPolicy();
const CadVisualPolicy& LoadCadVisualPolicy();
const CadShadowPolicy& LoadCadShadowPolicy();

}  // namespace repulsor3d::cad

