#include "repulsor3d/render/cad/CadPolicies.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace repulsor3d::cad {

int GetEnvInt(const char* name, const int fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

float GetEnvFloat(const char* name, const float fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stof(value);
  } catch (...) {
    return fallback;
  }
}

bool GetEnvBool(const char* name, const bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (text == "1" || text == "true" || text == "yes" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "no" || text == "off") {
    return false;
  }
  return fallback;
}

const CadLodPolicy& LoadCadLodPolicy() {
  static const CadLodPolicy policy = []() {
    CadLodPolicy out;
    out.maxLodCount = static_cast<std::size_t>(std::max(1, GetEnvInt("CAD_LOD_COUNT", static_cast<int>(out.maxLodCount))));
    out.reductionRatio = std::clamp(GetEnvFloat("CAD_LOD_RATIO", out.reductionRatio), 0.15F, 0.95F);
    out.minVertexCount = static_cast<std::size_t>(std::max(300, GetEnvInt("CAD_LOD_MIN_VERTICES", static_cast<int>(out.minVertexCount))));
    out.lod0VertexCap = static_cast<std::size_t>(std::max(1000, GetEnvInt("CAD_LOD0_MAX_VERTICES", static_cast<int>(out.lod0VertexCap))));
    out.keepOriginalLod0 = GetEnvBool("CAD_LOD_KEEP_FULL", out.keepOriginalLod0);
    out.maxPreferredDrawIndices = static_cast<std::size_t>(
        std::max(0, GetEnvInt("CAD_LOD_MAX_DRAW_INDICES", static_cast<int>(out.maxPreferredDrawIndices))));
    out.normalBins = std::clamp(GetEnvInt("CAD_LOD_NORMAL_BINS", out.normalBins), 4, 28);
    out.preserveFlatShading = GetEnvBool("CAD_LOD_PRESERVE_FLAT", out.preserveFlatShading);
    out.lod0ScreenRadiusPx = std::max(32.0F, GetEnvFloat("CAD_LOD0_SCREEN_RADIUS_PX", out.lod0ScreenRadiusPx));
    out.screenRadiusDecay = std::clamp(GetEnvFloat("CAD_LOD_SCREEN_RADIUS_DECAY", out.screenRadiusDecay), 0.2F, 0.95F);
    return out;
  }();
  return policy;
}

const CadVisualPolicy& LoadCadVisualPolicy() {
  static const CadVisualPolicy policy = []() {
    CadVisualPolicy out;
    out.enableFrustumCulling = GetEnvBool("CAD_FRUSTUM_CULLING", out.enableFrustumCulling);
    out.enableBroadphase = GetEnvBool("CAD_BROADPHASE_ENABLED", out.enableBroadphase);
    out.broadphaseCellSizeM = std::clamp(GetEnvFloat("CAD_BROADPHASE_CELL_SIZE_M", out.broadphaseCellSizeM), 0.5F, 20.0F);
    out.enableClusterCulling = GetEnvBool("CAD_CLUSTER_CULLING", out.enableClusterCulling);
    out.optimizeVertexCache = GetEnvBool("CAD_OPTIMIZE_VERTEX_CACHE", out.optimizeVertexCache);
    out.clusterTargetIndices = std::clamp(GetEnvInt("CAD_CLUSTER_TARGET_INDICES", out.clusterTargetIndices), 3 * 64, 3 * 1024 * 1024);
    out.clusterMinIndices = std::clamp(GetEnvInt("CAD_CLUSTER_MIN_INDICES", out.clusterMinIndices), 3 * 16, out.clusterTargetIndices);
    out.clusterMergeGapIndices = std::clamp(GetEnvInt("CAD_CLUSTER_MERGE_GAP_INDICES", out.clusterMergeGapIndices), 0, 3 * 4096);
    out.fieldClusterMaxVisibleIndices = std::clamp(
        GetEnvInt("CAD_FIELD_CLUSTER_MAX_VISIBLE_INDICES", out.fieldClusterMaxVisibleIndices),
        0,
        50000000);
    out.minClusterScreenRadiusPx = std::clamp(
        GetEnvFloat("CAD_CLUSTER_MIN_SCREEN_RADIUS_PX", out.minClusterScreenRadiusPx),
        0.0F,
        100.0F);
    out.enableDepthPrepass = GetEnvBool("CAD_DEPTH_PREPASS", out.enableDepthPrepass);
    out.enableOcclusionCulling = GetEnvBool("CAD_HIZ_OCCLUSION", out.enableOcclusionCulling);
    out.hzbTilesX = std::clamp(GetEnvInt("CAD_HIZ_TILES_X", out.hzbTilesX), 8, 256);
    out.hzbTilesY = std::clamp(GetEnvInt("CAD_HIZ_TILES_Y", out.hzbTilesY), 8, 256);
    out.hzbDepthMargin = std::clamp(GetEnvFloat("CAD_HIZ_DEPTH_MARGIN", out.hzbDepthMargin), 0.0F, 0.2F);
    out.hzbMaxCullRadiusPx = std::clamp(GetEnvFloat("CAD_HIZ_MAX_CULL_RADIUS_PX", out.hzbMaxCullRadiusPx), 8.0F, 500.0F);
    out.keyLightIntensity = std::clamp(GetEnvFloat("CAD_KEY_LIGHT_INTENSITY", out.keyLightIntensity), 0.0F, 8.0F);
    out.fillLightIntensity = std::clamp(GetEnvFloat("CAD_FILL_LIGHT_INTENSITY", out.fillLightIntensity), 0.0F, 6.0F);
    out.ambientStrength = std::clamp(GetEnvFloat("CAD_AMBIENT_STRENGTH", out.ambientStrength), 0.0F, 2.0F);
    out.depthCueStrength = std::clamp(GetEnvFloat("CAD_DEPTH_CUE_STRENGTH", out.depthCueStrength), 0.0F, 1.5F);
    out.specularStrength = std::clamp(GetEnvFloat("CAD_SPECULAR_STRENGTH", out.specularStrength), 0.0F, 2.0F);
    out.rimStrength = std::clamp(GetEnvFloat("CAD_RIM_STRENGTH", out.rimStrength), 0.0F, 2.0F);
    out.roughness = std::clamp(GetEnvFloat("CAD_ROUGHNESS", out.roughness), 0.03F, 1.0F);
    out.metallic = std::clamp(GetEnvFloat("CAD_METALLIC", out.metallic), 0.0F, 1.0F);
    out.exposure = std::clamp(GetEnvFloat("CAD_EXPOSURE", out.exposure), 0.1F, 4.0F);
    out.fogDensity = std::clamp(GetEnvFloat("CAD_FOG_DENSITY", out.fogDensity), 0.0F, 0.25F);
    out.saturation = std::clamp(GetEnvFloat("CAD_SATURATION", out.saturation), 0.0F, 2.0F);
    out.gamma = std::clamp(GetEnvFloat("CAD_GAMMA", out.gamma), 1.0F, 3.2F);
    return out;
  }();
  return policy;
}

const CadShadowPolicy& LoadCadShadowPolicy() {
  static const CadShadowPolicy policy = []() {
    CadShadowPolicy out;
    out.enabled = GetEnvBool("CAD_SHADOWS_ENABLED", out.enabled);
    out.mapSize = std::clamp(GetEnvInt("CAD_SHADOW_MAP_SIZE", out.mapSize), 512, 8192);
    out.strength = std::clamp(GetEnvFloat("CAD_SHADOW_STRENGTH", out.strength), 0.0F, 1.0F);
    out.cascadeCount = std::clamp(GetEnvInt("CAD_SHADOW_CASCADES", out.cascadeCount), 1, 2);
    out.cascadeSplitDistanceM = std::clamp(GetEnvFloat("CAD_SHADOW_CASCADE_SPLIT_M", out.cascadeSplitDistanceM), 2.0F, 100.0F);

    std::string quality = "high";
    if (const char* value = std::getenv("CAD_SHADOW_QUALITY"); value != nullptr && *value != '\0') {
      quality = value;
      std::transform(quality.begin(), quality.end(), quality.begin(), [](const unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
    }
    if (quality == "low") {
      out.quality = ShadowQuality::Low;
      out.pcfRadius = 1;
      out.mapSize = std::min(out.mapSize, 1024);
    } else if (quality == "medium") {
      out.quality = ShadowQuality::Medium;
      out.pcfRadius = 2;
      out.mapSize = std::min(out.mapSize, 2048);
    } else if (quality == "ultra") {
      out.quality = ShadowQuality::Ultra;
      out.pcfRadius = 3;
      out.mapSize = std::max(out.mapSize, 3072);
    } else {
      out.quality = ShadowQuality::High;
      out.pcfRadius = 2;
    }
    return out;
  }();
  return policy;
}

}  // namespace repulsor3d::cad
