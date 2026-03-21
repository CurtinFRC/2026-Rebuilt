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

std::string GetEnvStringLower(const char* name, std::string fallback) {
  const char* value = std::getenv(name);
  if (value != nullptr && *value != '\0') {
    fallback = value;
  }
  std::transform(fallback.begin(), fallback.end(), fallback.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
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
    out.frustumCullMarginM = std::clamp(GetEnvFloat("CAD_FRUSTUM_MARGIN_M", out.frustumCullMarginM), 0.0F, 3.0F);
    out.enableBroadphase = GetEnvBool("CAD_BROADPHASE_ENABLED", out.enableBroadphase);
    out.broadphaseCellSizeM = std::clamp(GetEnvFloat("CAD_BROADPHASE_CELL_SIZE_M", out.broadphaseCellSizeM), 0.5F, 20.0F);
    out.enableClusterCulling = GetEnvBool("CAD_CLUSTER_CULLING", out.enableClusterCulling);
    out.fieldDisableClusterCulling =
        GetEnvBool("CAD_FIELD_DISABLE_CLUSTER_CULLING", out.fieldDisableClusterCulling);
    out.fieldForceLod0 = GetEnvBool("CAD_FIELD_FORCE_LOD0", out.fieldForceLod0);
    out.optimizeVertexCache = GetEnvBool("CAD_OPTIMIZE_VERTEX_CACHE", out.optimizeVertexCache);
    out.clusterTargetIndices = std::clamp(GetEnvInt("CAD_CLUSTER_TARGET_INDICES", out.clusterTargetIndices), 3 * 64, 3 * 1024 * 1024);
    out.clusterMinIndices = std::clamp(GetEnvInt("CAD_CLUSTER_MIN_INDICES", out.clusterMinIndices), 3 * 16, out.clusterTargetIndices);
    out.clusterMergeGapIndices = std::clamp(GetEnvInt("CAD_CLUSTER_MERGE_GAP_INDICES", out.clusterMergeGapIndices), 0, 3 * 4096);
    out.fieldClusterMaxVisibleIndices = std::clamp(
        GetEnvInt("CAD_FIELD_CLUSTER_MAX_VISIBLE_INDICES", out.fieldClusterMaxVisibleIndices),
        0,
        50000000);
    out.fieldClusterAdaptiveBudget = GetEnvBool("CAD_FIELD_CLUSTER_ADAPTIVE_BUDGET", out.fieldClusterAdaptiveBudget);
    out.fieldClusterAdaptiveMinScale = std::clamp(
        GetEnvFloat("CAD_FIELD_CLUSTER_ADAPTIVE_MIN_SCALE", out.fieldClusterAdaptiveMinScale),
        0.20F,
        1.00F);
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
    out.hzbMinCoverage = std::clamp(
        GetEnvFloat("CAD_HIZ_MIN_COVERAGE", out.hzbMinCoverage),
        0.0F,
        1.0F);
    out.hzbMinOccluderRadiusPx = std::clamp(
        GetEnvFloat("CAD_HIZ_MIN_OCCLUDER_RADIUS_PX", out.hzbMinOccluderRadiusPx),
        0.0F,
        500.0F);
    out.maxIndicesPerDrawCall = std::clamp(
        GetEnvInt("CAD_MAX_INDICES_PER_DRAW_CALL", out.maxIndicesPerDrawCall),
        0,
        50000000);
    const std::string styleMode = GetEnvStringLower("CAD_STYLE_MODE", "arcade");
    if (styleMode == "realistic" || styleMode == "cad") {
      out.shadingMode = 0;
    } else if (styleMode == "fast" || styleMode == "flat") {
      out.shadingMode = 2;
    } else {
      out.shadingMode = 1;
    }
    out.shadingMode = std::clamp(GetEnvInt("CAD_SHADING_MODE", out.shadingMode), 0, 2);
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
    out.weatheringStrength = std::clamp(
        GetEnvFloat("CAD_WEATHERING_STRENGTH", out.weatheringStrength),
        0.0F,
        1.5F);
    out.weatheringScale = std::clamp(
        GetEnvFloat("CAD_WEATHERING_SCALE", out.weatheringScale),
        0.01F,
        8.0F);
    out.detailRoughnessStrength = std::clamp(
        GetEnvFloat("CAD_DETAIL_ROUGHNESS_STRENGTH", out.detailRoughnessStrength),
        0.0F,
        1.0F);
    out.clearcoatStrength = std::clamp(
        GetEnvFloat("CAD_CLEARCOAT_STRENGTH", out.clearcoatStrength),
        0.0F,
        1.0F);
    out.shadowTintStrength = std::clamp(
        GetEnvFloat("CAD_SHADOW_TINT_STRENGTH", out.shadowTintStrength),
        0.0F,
        1.0F);
    const std::string themePreset = GetEnvStringLower("CAD_THEME_PRESET", "arena_blue");
    if (themePreset == "red" || themePreset == "red_alliance") {
      out.themePrimaryR = 0.98F;
      out.themePrimaryG = 0.16F;
      out.themePrimaryB = 0.14F;
      out.themeSecondaryR = 1.00F;
      out.themeSecondaryG = 0.54F;
      out.themeSecondaryB = 0.16F;
      out.themeMix = 0.60F;
    } else if (themePreset == "emerald" || themePreset == "green") {
      out.themePrimaryR = 0.06F;
      out.themePrimaryG = 0.96F;
      out.themePrimaryB = 0.62F;
      out.themeSecondaryR = 0.22F;
      out.themeSecondaryG = 0.66F;
      out.themeSecondaryB = 1.00F;
      out.themeMix = 0.56F;
    } else if (themePreset == "amber" || themePreset == "gold") {
      out.themePrimaryR = 1.00F;
      out.themePrimaryG = 0.62F;
      out.themePrimaryB = 0.06F;
      out.themeSecondaryR = 1.00F;
      out.themeSecondaryG = 0.28F;
      out.themeSecondaryB = 0.08F;
      out.themeMix = 0.58F;
    } else if (themePreset == "neutral") {
      out.themePrimaryR = 0.75F;
      out.themePrimaryG = 0.82F;
      out.themePrimaryB = 0.92F;
      out.themeSecondaryR = 0.56F;
      out.themeSecondaryG = 0.64F;
      out.themeSecondaryB = 0.78F;
      out.themeMix = 0.40F;
    } else {
      // arena_blue default
      out.themePrimaryR = 0.08F;
      out.themePrimaryG = 0.66F;
      out.themePrimaryB = 1.00F;
      out.themeSecondaryR = 1.00F;
      out.themeSecondaryG = 0.34F;
      out.themeSecondaryB = 0.20F;
      out.themeMix = 0.52F;
    }
    out.themePrimaryR = std::clamp(GetEnvFloat("CAD_THEME_PRIMARY_R", out.themePrimaryR), 0.0F, 2.0F);
    out.themePrimaryG = std::clamp(GetEnvFloat("CAD_THEME_PRIMARY_G", out.themePrimaryG), 0.0F, 2.0F);
    out.themePrimaryB = std::clamp(GetEnvFloat("CAD_THEME_PRIMARY_B", out.themePrimaryB), 0.0F, 2.0F);
    out.themeSecondaryR = std::clamp(GetEnvFloat("CAD_THEME_SECONDARY_R", out.themeSecondaryR), 0.0F, 2.0F);
    out.themeSecondaryG = std::clamp(GetEnvFloat("CAD_THEME_SECONDARY_G", out.themeSecondaryG), 0.0F, 2.0F);
    out.themeSecondaryB = std::clamp(GetEnvFloat("CAD_THEME_SECONDARY_B", out.themeSecondaryB), 0.0F, 2.0F);
    out.themeMix = std::clamp(GetEnvFloat("CAD_THEME_MIX", out.themeMix), 0.0F, 1.0F);
    out.themeEmissiveStrength = std::clamp(
        GetEnvFloat("CAD_THEME_EMISSIVE_STRENGTH", out.themeEmissiveStrength),
        0.0F,
        3.0F);
    out.nightMode = GetEnvBool("CAD_NIGHT_MODE", out.nightMode);
    out.nightIntensity = std::clamp(GetEnvFloat("CAD_NIGHT_INTENSITY", out.nightIntensity), 0.0F, 1.5F);
    out.postFxStrength = std::clamp(GetEnvFloat("CAD_POSTFX_STRENGTH", out.postFxStrength), 0.0F, 2.0F);
    out.filmGrainStrength = std::clamp(
        GetEnvFloat("CAD_FILM_GRAIN_STRENGTH", out.filmGrainStrength),
        0.0F,
        0.25F);
    out.vignetteStrength = std::clamp(
        GetEnvFloat("CAD_VIGNETTE_STRENGTH", out.vignetteStrength),
        0.0F,
        1.5F);
    out.chromaticStrength = std::clamp(
        GetEnvFloat("CAD_CHROMATIC_STRENGTH", out.chromaticStrength),
        0.0F,
        0.25F);
    out.arcadeToonBandCount = std::clamp(
        GetEnvFloat("CAD_ARCADE_TOON_BANDS", out.arcadeToonBandCount),
        2.0F,
        6.0F);
    out.arcadeRimBoost = std::clamp(
        GetEnvFloat("CAD_ARCADE_RIM_BOOST", out.arcadeRimBoost),
        0.0F,
        4.0F);
    out.arcadeOutlineStrength = std::clamp(
        GetEnvFloat("CAD_ARCADE_OUTLINE_STRENGTH", out.arcadeOutlineStrength),
        0.0F,
        1.0F);
    out.arcadeThemeTintStrength = std::clamp(
        GetEnvFloat("CAD_ARCADE_THEME_TINT_STRENGTH", out.arcadeThemeTintStrength),
        0.0F,
        1.0F);
    out.arcadeEmissiveBoost = std::clamp(
        GetEnvFloat("CAD_ARCADE_EMISSIVE_BOOST", out.arcadeEmissiveBoost),
        0.0F,
        6.0F);
    out.arcadeShadowLift = std::clamp(
        GetEnvFloat("CAD_ARCADE_SHADOW_LIFT", out.arcadeShadowLift),
        0.0F,
        1.0F);
    out.arcadeFogScale = std::clamp(
        GetEnvFloat("CAD_ARCADE_FOG_SCALE", out.arcadeFogScale),
        0.0F,
        2.0F);
    out.arcadeBloomStrength = std::clamp(
        GetEnvFloat("CAD_ARCADE_BLOOM_STRENGTH", out.arcadeBloomStrength),
        0.0F,
        2.0F);
    out.arcadeStatePulseRate = std::clamp(
        GetEnvFloat("CAD_ARCADE_STATE_PULSE_RATE", out.arcadeStatePulseRate),
        0.0F,
        4.0F);
    out.arcadeAlert = std::clamp(
        GetEnvFloat("CAD_ARCADE_ALERT", out.arcadeAlert),
        0.0F,
        1.0F);
    out.arcadeSelected = std::clamp(
        GetEnvFloat("CAD_ARCADE_SELECTED", out.arcadeSelected),
        0.0F,
        1.0F);
    out.arcadeCharged = std::clamp(
        GetEnvFloat("CAD_ARCADE_CHARGED", out.arcadeCharged),
        0.0F,
        1.0F);
    out.arcadeDisableWeathering = GetEnvBool("CAD_ARCADE_DISABLE_WEATHERING", out.arcadeDisableWeathering);
    out.arcadeHardSpecular = GetEnvBool("CAD_ARCADE_HARD_SPECULAR", out.arcadeHardSpecular);

    const std::string postPreset = GetEnvStringLower("CAD_POSTFX_PRESET", "arcade");
    if (postPreset == "broadcast") {
      out.postFxStrength = 1.15F;
      out.vignetteStrength = 0.34F;
      out.filmGrainStrength = 0.022F;
      out.chromaticStrength = 0.018F;
      out.saturation = 1.12F;
    } else if (postPreset == "neon_night") {
      out.postFxStrength = 1.35F;
      out.vignetteStrength = 0.42F;
      out.filmGrainStrength = 0.018F;
      out.chromaticStrength = 0.040F;
      out.saturation = 1.18F;
      out.nightMode = true;
      out.nightIntensity = std::max(out.nightIntensity, 0.85F);
    } else if (postPreset == "retro_arena") {
      out.postFxStrength = 1.22F;
      out.vignetteStrength = 0.46F;
      out.filmGrainStrength = 0.040F;
      out.chromaticStrength = 0.030F;
      out.saturation = 1.10F;
    } else if (postPreset == "none" || postPreset == "off") {
      out.postFxStrength = 0.0F;
      out.vignetteStrength = 0.0F;
      out.filmGrainStrength = 0.0F;
      out.chromaticStrength = 0.0F;
    } else {
      // arcade default
      out.postFxStrength = 1.05F;
      out.vignetteStrength = 0.30F;
      out.filmGrainStrength = 0.016F;
      out.chromaticStrength = 0.020F;
      out.saturation = std::max(out.saturation, 1.10F);
    }
    out.enableInstanceUploadDedup = GetEnvBool("CAD_INSTANCE_UPLOAD_DEDUP", out.enableInstanceUploadDedup);
    out.enableInstanceBufferOrphaning = GetEnvBool("CAD_INSTANCE_BUFFER_ORPHANING", out.enableInstanceBufferOrphaning);
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
    out.cascadeBlendRangeM = std::clamp(
        GetEnvFloat("CAD_SHADOW_CASCADE_BLEND_M", out.cascadeBlendRangeM),
        0.0F,
        30.0F);

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
