#include "repulsor3d/render/cad/CadShaderSources.hpp"

namespace repulsor3d::cad {

const char* CadMainVertexShaderSource() {
  return R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;
layout(location = 3) in vec4 iModelRow0;
layout(location = 4) in vec4 iModelRow1;
layout(location = 5) in vec4 iModelRow2;
layout(location = 6) in vec4 iModelRow3;
layout(location = 7) in vec3 iNormalRow0;
layout(location = 8) in vec3 iNormalRow1;
layout(location = 9) in vec3 iNormalRow2;
uniform mat4 uViewProjection;
uniform mat4 uLightViewProjection;
uniform mat4 uLightViewProjectionFar;
out vec4 vColor;
out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec4 vLightClipPos;
out vec4 vLightClipPosFar;
void main() {
  mat4 model = mat4(iModelRow0, iModelRow1, iModelRow2, iModelRow3);
  vec4 worldPos = model * vec4(aPos, 1.0);
  vWorldPos = worldPos.xyz;
  mat3 normalMatrix = mat3(iNormalRow0, iNormalRow1, iNormalRow2);
  vWorldNormal = normalize(normalMatrix * aNormal);
  vColor = aColor;
  vLightClipPos = uLightViewProjection * worldPos;
  vLightClipPosFar = uLightViewProjectionFar * worldPos;
  gl_Position = uViewProjection * worldPos;
}
)";
}

const char* CadMainFragmentShaderSource() {
  return R"(
#version 330 core
const float PI = 3.14159265359;
in vec4 vColor;
in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec4 vLightClipPos;
in vec4 vLightClipPosFar;
uniform vec4 uColor;
uniform vec3 uLightDir;
uniform vec3 uCameraPos;
uniform float uKeyLightIntensity;
uniform float uFillLightIntensity;
uniform float uAmbientStrength;
uniform float uDepthCueStrength;
uniform float uSpecularStrength;
uniform float uRimStrength;
uniform float uRoughness;
uniform float uMetallic;
uniform float uExposure;
uniform float uFogDensity;
uniform float uSaturation;
uniform float uGamma;
uniform float uWeatheringStrength;
uniform float uWeatheringScale;
uniform float uDetailRoughnessStrength;
uniform float uClearcoatStrength;
uniform float uShadowTintStrength;
uniform vec3 uThemePrimary;
uniform vec3 uThemeSecondary;
uniform float uThemeMix;
uniform float uThemeEmissiveStrength;
uniform float uNightMode;
uniform float uNightIntensity;
uniform float uPostFxStrength;
uniform float uFilmGrainStrength;
uniform float uVignetteStrength;
uniform float uChromaticStrength;
uniform vec3 uViewportInfo;
uniform float uUseAssetColor;
uniform sampler2D uShadowMap;
uniform sampler2D uShadowMapFar;
uniform sampler2D uAlbedoMap;
uniform sampler2D uNormalMap;
uniform int uShadowEnabled;
uniform float uShadowStrength;
uniform int uShadowPcfRadius;
uniform int uShadowCascadeCount;
uniform float uShadowCascadeSplitM;
uniform float uShadowCascadeBlendRangeM;
uniform int uHasAlbedoMap;
uniform int uHasNormalMap;
uniform int uShadingMode;
uniform float uNormalStrength;
uniform float uTriplanarScale;
uniform float uTimeS;
uniform vec4 uArcadeStyle0;
uniform vec4 uArcadeStyle1;
uniform vec4 uArcadeState;
uniform float uArcadeMotion;
out vec4 FragColor;

float Hash13(vec3 p) {
  p = fract(p * 0.1031);
  p += dot(p, p.yzx + 33.33);
  return fract((p.x + p.y) * p.z);
}

mat2 Rotation2(float a) {
  float s = sin(a);
  float c = cos(a);
  return mat2(c, -s, s, c);
}

float NoiseHash31(vec3 p) {
  p = fract(p * 0.1031);
  p += dot(p, p.yzx + 33.33);
  return fract((p.x + p.y) * p.z);
}

float ValueNoise3(vec3 p) {
  vec3 i = floor(p);
  vec3 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float n000 = NoiseHash31(i + vec3(0.0, 0.0, 0.0));
  float n100 = NoiseHash31(i + vec3(1.0, 0.0, 0.0));
  float n010 = NoiseHash31(i + vec3(0.0, 1.0, 0.0));
  float n110 = NoiseHash31(i + vec3(1.0, 1.0, 0.0));
  float n001 = NoiseHash31(i + vec3(0.0, 0.0, 1.0));
  float n101 = NoiseHash31(i + vec3(1.0, 0.0, 1.0));
  float n011 = NoiseHash31(i + vec3(0.0, 1.0, 1.0));
  float n111 = NoiseHash31(i + vec3(1.0, 1.0, 1.0));
  float nx00 = mix(n000, n100, f.x);
  float nx10 = mix(n010, n110, f.x);
  float nx01 = mix(n001, n101, f.x);
  float nx11 = mix(n011, n111, f.x);
  float nxy0 = mix(nx00, nx10, f.y);
  float nxy1 = mix(nx01, nx11, f.y);
  return mix(nxy0, nxy1, f.z);
}

const vec2 kPoisson16[16] = vec2[](
    vec2(-0.94201624, -0.39906216),
    vec2(0.94558609, -0.76890725),
    vec2(-0.09418410, -0.92938870),
    vec2(0.34495938, 0.29387760),
    vec2(-0.91588581, 0.45771432),
    vec2(-0.81544232, -0.87912464),
    vec2(-0.38277543, 0.27676845),
    vec2(0.97484398, 0.75648379),
    vec2(0.44323325, -0.97511554),
    vec2(0.53742981, -0.47373420),
    vec2(-0.26496911, -0.41893023),
    vec2(0.79197514, 0.19090188),
    vec2(-0.24188840, 0.99706507),
    vec2(-0.81409955, 0.91437590),
    vec2(0.19984126, 0.78641367),
    vec2(0.14383161, -0.14100790));

vec3 FresnelSchlick(float cosTheta, vec3 F0) {
  return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float DistributionGGX(vec3 N, vec3 H, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float NdotH = max(dot(N, H), 0.0);
  float NdotH2 = NdotH * NdotH;
  float denom = (NdotH2 * (a2 - 1.0) + 1.0);
  return a2 / max(PI * denom * denom, 1e-6);
}

float GeometrySchlickGGX(float NdotV, float roughness) {
  float r = roughness + 1.0;
  float k = (r * r) / 8.0;
  float denom = NdotV * (1.0 - k) + k;
  return NdotV / max(denom, 1e-6);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
  float NdotV = max(dot(N, V), 0.0);
  float NdotL = max(dot(N, L), 0.0);
  float ggx2 = GeometrySchlickGGX(NdotV, roughness);
  float ggx1 = GeometrySchlickGGX(NdotL, roughness);
  return ggx1 * ggx2;
}

vec3 TonemapAces(vec3 x) {
  const float a = 2.51;
  const float b = 0.03;
  const float c = 2.43;
  const float d = 0.59;
  const float e = 0.14;
  return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

float ToonRamp3(float n) {
  if (n < 0.22) {
    return 0.12;
  }
  if (n < 0.62) {
    return 0.45;
  }
  return 1.0;
}

float ToonRampN(float n, float bands) {
  float b = max(bands, 2.0);
  float q = floor(clamp(n, 0.0, 0.9999) * b);
  return q / max(b - 1.0, 1.0);
}

float ToonRampSmooth(float n, float bands, float softness) {
  float b = max(bands, 2.0);
  float scaled = clamp(n, 0.0, 0.9999) * b;
  float band = floor(scaled);
  float fracPart = fract(scaled);
  float maxBand = max(b - 1.0, 1.0);
  float a = band / maxBand;
  float nextBand = min(band + 1.0, b - 1.0);
  float c = nextBand / maxBand;
  float soft = clamp(softness, 0.0, 0.95);
  float t0 = 0.5 - soft * 0.5;
  float t1 = 0.5 + soft * 0.5;
  float t = smoothstep(t0, t1, fracPart);
  return mix(a, c, t);
}

vec3 TriplanarWeights(vec3 n) {
  vec3 an = abs(normalize(n));
  an = pow(an, vec3(4.0));
  float sum = max(an.x + an.y + an.z, 1e-6);
  return an / sum;
}

vec4 SampleTriplanarColor(sampler2D tex, vec3 worldPos, vec3 worldNormal, float scale) {
  vec3 weights = TriplanarWeights(worldNormal);
  vec2 uvX = worldPos.yz * scale;
  vec2 uvY = worldPos.xz * scale;
  vec2 uvZ = worldPos.xy * scale;
  vec4 cx = texture(tex, uvX);
  vec4 cy = texture(tex, uvY);
  vec4 cz = texture(tex, uvZ);
  return cx * weights.x + cy * weights.y + cz * weights.z;
}

vec4 SampleDominantProjectionColor(sampler2D tex, vec3 worldPos, vec3 worldNormal, float scale) {
  vec3 an = abs(normalize(worldNormal));
  if (an.x >= an.y && an.x >= an.z) {
    return texture(tex, worldPos.yz * scale);
  }
  if (an.y >= an.z) {
    return texture(tex, worldPos.xz * scale);
  }
  return texture(tex, worldPos.xy * scale);
}

vec3 NormalFromProjectionX(vec3 nTex, vec3 worldNormal) {
  float signX = worldNormal.x >= 0.0 ? 1.0 : -1.0;
  vec3 t = vec3(0.0, 1.0, 0.0);
  vec3 b = vec3(0.0, 0.0, signX);
  vec3 n = vec3(signX, 0.0, 0.0);
  return normalize(t * nTex.x + b * nTex.y + n * nTex.z);
}

vec3 NormalFromProjectionY(vec3 nTex, vec3 worldNormal) {
  float signY = worldNormal.y >= 0.0 ? 1.0 : -1.0;
  vec3 t = vec3(1.0, 0.0, 0.0);
  vec3 b = vec3(0.0, 0.0, signY);
  vec3 n = vec3(0.0, signY, 0.0);
  return normalize(t * nTex.x + b * nTex.y + n * nTex.z);
}

vec3 NormalFromProjectionZ(vec3 nTex, vec3 worldNormal) {
  float signZ = worldNormal.z >= 0.0 ? 1.0 : -1.0;
  vec3 t = vec3(1.0, 0.0, 0.0);
  vec3 b = vec3(0.0, signZ, 0.0);
  vec3 n = vec3(0.0, 0.0, signZ);
  return normalize(t * nTex.x + b * nTex.y + n * nTex.z);
}

vec3 SampleTriplanarNormal(sampler2D tex, vec3 worldPos, vec3 worldNormal, float scale) {
  vec3 weights = TriplanarWeights(worldNormal);
  vec2 uvX = worldPos.yz * scale;
  vec2 uvY = worldPos.xz * scale;
  vec2 uvZ = worldPos.xy * scale;

  vec3 sx = texture(tex, uvX).xyz * 2.0 - 1.0;
  vec3 sy = texture(tex, uvY).xyz * 2.0 - 1.0;
  vec3 sz = texture(tex, uvZ).xyz * 2.0 - 1.0;

  vec3 nx = NormalFromProjectionX(sx, worldNormal);
  vec3 ny = NormalFromProjectionY(sy, worldNormal);
  vec3 nz = NormalFromProjectionZ(sz, worldNormal);
  return normalize(nx * weights.x + ny * weights.y + nz * weights.z);
}

float SampleShadowMap(bool useFarCascade, vec2 uv) {
  return useFarCascade ? texture(uShadowMapFar, uv).r : texture(uShadowMap, uv).r;
}

float ComputeShadowFactor(
    vec4 lightClipPos,
    vec3 normal,
    vec3 lightDir,
    int radius,
    bool useFarCascade,
    vec3 worldPos,
    bool fastMode) {
  if (uShadowEnabled == 0) {
    return 0.0;
  }

  vec3 proj = lightClipPos.xyz / max(lightClipPos.w, 1e-6);
  proj = proj * 0.5 + 0.5;
  if (proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0 || proj.z < 0.0 || proj.z > 1.0) {
    return 0.0;
  }

  vec3 N = normalize(normal);
  vec3 L = normalize(lightDir);
  float ndotl = max(dot(N, L), 0.0);
  float slope = sqrt(max(1.0 - ndotl * ndotl, 0.0));
  float distanceBias = proj.z * (useFarCascade ? 0.00075 : 0.00045);
  float bias = mix(0.0022, 0.00045, ndotl) + slope * 0.0012 + distanceBias;
  vec2 texelSize = useFarCascade
                       ? (1.0 / vec2(textureSize(uShadowMapFar, 0)))
                       : (1.0 / vec2(textureSize(uShadowMap, 0)));

  int pcfRadius = clamp(radius, 0, 4);
  if (fastMode || pcfRadius == 0) {
    float closest = SampleShadowMap(useFarCascade, proj.xy);
    return (proj.z - bias > closest) ? 1.0 : 0.0;
  }

  float shadowHits = 0.0;
  float taps = 0.0;
  float kernelScale = float(max(1, pcfRadius));
  float jitter = Hash13(worldPos * 0.19 + vec3(proj.xy * 17.0, proj.z * 13.0));
  mat2 rot = Rotation2(jitter * 6.28318530718);
  int sampleCount = clamp(8 + pcfRadius * 4, 4, 16);
  for (int i = 0; i < 16; ++i) {
    if (i >= sampleCount) {
      break;
    }
    vec2 offset = rot * kPoisson16[i] * texelSize * kernelScale * 1.7;
    float closest = SampleShadowMap(useFarCascade, proj.xy + offset);
    shadowHits += (proj.z - bias > closest) ? 1.0 : 0.0;
    taps += 1.0;
  }
  return shadowHits / max(taps, 1.0);
}
)"
R"(
vec3 ShadePbrLight(
    vec3 N,
    vec3 V,
    vec3 L,
    vec3 lightColor,
    float lightIntensity,
    vec3 albedo,
    float roughness,
    float metallic,
    vec3 F0) {
  float NdotL = max(dot(N, L), 0.0);
  if (NdotL <= 1e-5) {
    return vec3(0.0);
  }

  vec3 H = normalize(V + L);
  float NDF = DistributionGGX(N, H, roughness);
  float G = GeometrySmith(N, V, L, roughness);
  vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);

  vec3 numerator = NDF * G * F;
  float denominator = max(4.0 * max(dot(N, V), 0.0) * NdotL, 1e-5);
  vec3 specular = numerator / denominator;

  vec3 kS = F;
  vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);
  vec3 radiance = lightColor * lightIntensity;
  return (kD * albedo / PI + specular) * radiance * NdotL;
}

vec3 ShadeClearcoatLight(
    vec3 N,
    vec3 V,
    vec3 L,
    vec3 lightColor,
    float lightIntensity,
    float clearcoatStrength) {
  float NdotL = max(dot(N, L), 0.0);
  float NdotV = max(dot(N, V), 0.0);
  if (NdotL <= 1e-5 || NdotV <= 1e-5 || clearcoatStrength <= 1e-5) {
    return vec3(0.0);
  }
  vec3 H = normalize(V + L);
  float coatRoughness = 0.08;
  float NDF = DistributionGGX(N, H, coatRoughness);
  float G = GeometrySmith(N, V, L, coatRoughness);
  float F = 0.04 + (1.0 - 0.04) * pow(clamp(1.0 - max(dot(H, V), 0.0), 0.0, 1.0), 5.0);
  float denom = max(4.0 * NdotV * NdotL, 1e-5);
  float coatSpec = (NDF * G * F) / denom;
  return lightColor * lightIntensity * coatSpec * NdotL * clearcoatStrength;
}

)"
R"(
void main() {
  float useAssetColor = clamp(uUseAssetColor, 0.0, 1.0);
  vec3 baseColorSrgb = mix(uColor.rgb, vColor.rgb * uColor.rgb, useAssetColor);
  float alpha = uColor.a * mix(1.0, vColor.a, useAssetColor);

  vec3 geomN = normalize(vWorldNormal);
  vec3 N = geomN;
  vec3 V = normalize(uCameraPos - vWorldPos);
  if (uHasNormalMap == 1 && uShadingMode == 0) {
    vec3 mappedNormal = SampleTriplanarNormal(
        uNormalMap,
        vWorldPos,
        geomN,
        max(uTriplanarScale, 1e-4));
    N = normalize(mix(geomN, mappedNormal, clamp(uNormalStrength, 0.0, 2.0)));
  }

  if (uHasAlbedoMap == 1) {
    vec4 texColor = (uShadingMode == 0)
                        ? SampleTriplanarColor(
                              uAlbedoMap,
                              vWorldPos,
                              geomN,
                              max(uTriplanarScale, 1e-4))
                        : SampleDominantProjectionColor(
                              uAlbedoMap,
                              vWorldPos,
                              geomN,
                              max(uTriplanarScale, 1e-4));
    baseColorSrgb *= texColor.rgb;
    alpha *= texColor.a;
  }

  vec3 themePrimary = max(uThemePrimary, vec3(0.0));
  vec3 themeSecondary = max(uThemeSecondary, vec3(0.0));
  float themeMix = clamp(uThemeMix, 0.0, 1.0);
  float themeEmissiveStrength = max(uThemeEmissiveStrength, 0.0);
  float arcadeBands = clamp(uArcadeStyle0.x, 2.0, 6.0);
  float arcadeRimBoost = max(uArcadeStyle0.y, 0.0);
  float arcadeOutlineStrength = clamp(uArcadeStyle0.z, 0.0, 1.0);
  float arcadeThemeTintStrength = clamp(uArcadeStyle0.w, 0.0, 1.0);
  float arcadeEmissiveBoost = max(uArcadeStyle1.x, 0.0);
  float arcadeShadowLift = clamp(uArcadeStyle1.y, 0.0, 1.0);
  float arcadeFogScale = clamp(uArcadeStyle1.z, 0.0, 2.0);
  float arcadeBloomStrength = clamp(uArcadeStyle1.w, 0.0, 2.0);
  float arcadeHighlightBoost = max(uArcadeState.x, 0.5);
  float arcadeAlert = clamp(uArcadeState.y, 0.0, 1.0);
  float arcadeSelected = clamp(uArcadeState.z, 0.0, 1.0);
  float arcadeCharged = clamp(uArcadeState.w, 0.0, 1.0);
  bool arcadeMode = (uShadingMode == 1);
  float arcadeModeF = arcadeMode ? 1.0 : 0.0;

  float weatheringStrength = clamp(uWeatheringStrength, 0.0, 1.5);
  if (arcadeMode) {
    weatheringStrength = 0.0;
  }
  float weatherScale = max(uWeatheringScale, 0.01);
  float macroNoise = ValueNoise3(vWorldPos * weatherScale * 0.35 + vec3(13.7, 7.1, 2.5));
  float microNoise = ValueNoise3(vWorldPos * weatherScale * 2.7 + vec3(1.9, 17.3, 9.7));
  float wearMask = clamp((macroNoise * 0.70 + microNoise * 0.30 - 0.43) * (0.6 + weatheringStrength * 1.15), 0.0, 1.0);
  float colorVariation = (microNoise - 0.5) * 0.12 * weatheringStrength;
  vec3 weatheredSrgb = baseColorSrgb * (1.0 + colorVariation);
  weatheredSrgb = mix(weatheredSrgb, weatheredSrgb * vec3(0.86, 0.84, 0.82), wearMask * 0.35);
  vec3 albedo = pow(clamp(weatheredSrgb, 0.0, 1.0), vec3(2.2));

  float roughness = clamp(uRoughness, 0.03, 1.0);
  float detailRoughnessStrength = clamp(uDetailRoughnessStrength, 0.0, 1.0);
  if (arcadeMode) {
    detailRoughnessStrength = 0.0;
  }
  roughness = clamp(
      roughness +
          (wearMask * 0.22 + (microNoise - 0.5) * 0.16) * detailRoughnessStrength,
      0.03,
      1.0);
  float metallic = clamp(uMetallic * (1.0 - wearMask * 0.30 * weatheringStrength), 0.0, 1.0);
  vec3 F0 = mix(vec3(0.04), albedo, metallic) * max(uSpecularStrength, 0.0);

  vec3 keyDir = normalize(-uLightDir);
  vec3 fillDir = keyDir;
  float viewDistance = length(uCameraPos - vWorldPos);
  float splitDistance = max(uShadowCascadeSplitM, 1.0);

  float up = clamp(N.z * 0.5 + 0.5, 0.0, 1.0);
  vec3 skyAmbient = vec3(0.44, 0.52, 0.64);
  vec3 groundAmbient = vec3(0.19, 0.18, 0.17);
  vec3 ambient = mix(groundAmbient, skyAmbient, up) * albedo * max(uAmbientStrength, 0.0);
  if (arcadeMode) {
    vec3 coolTop = mix(vec3(0.08, 0.14, 0.28), themePrimary * 0.60 + vec3(0.04, 0.08, 0.13), 0.72);
    vec3 coolBottom = mix(vec3(0.04, 0.06, 0.10), themePrimary * 0.28 + vec3(0.02, 0.03, 0.06), 0.55);
    ambient = mix(coolBottom, coolTop, up) * albedo * (0.30 + 0.42 * max(uAmbientStrength, 0.0));
  }
  float normalAgreement = clamp(dot(geomN, N), 0.2, 1.0);
  ambient *= mix(0.78, 1.0, normalAgreement);
  if (!arcadeMode) {
    float curvatureAo = clamp(1.0 - length(fwidth(N)) * 0.75, 0.55, 1.0);
    ambient *= curvatureAo;
  }
  float bounceAmount = (1.0 - up) * (0.35 + 0.65 * max(dot(-keyDir, vec3(0.0, 0.0, 1.0)), 0.0));
  vec3 bounceLight = vec3(0.23, 0.19, 0.14) * albedo * bounceAmount * 0.14;
  if (arcadeMode) {
    bounceLight = vec3(0.0);
  }
  float keyShadow = max(dot(N, keyDir), 0.0);
  float shadowContrast = mix(0.70, 1.0, keyShadow);
  bool fastShadow = (uShadingMode != 0);
  vec3 shadowTint = mix(vec3(1.0), vec3(0.76, 0.82, 0.92), clamp(uShadowTintStrength, 0.0, 1.0));
  if (arcadeMode) {
    vec3 coolShadow = mix(vec3(0.34, 0.46, 0.72), themePrimary * 0.70 + vec3(0.14, 0.20, 0.30), 0.58);
    shadowTint = mix(vec3(1.0), coolShadow, clamp(uShadowTintStrength, 0.0, 1.0));
  }

  float shadow = 0.0;
  if (uShadowEnabled != 0 && uShadowStrength > 1e-4 && keyShadow > 1e-4) {
    if (uShadowCascadeCount > 1) {
      float blendRange = max(uShadowCascadeBlendRangeM, 0.0);
      float blend = (blendRange > 0.0)
                        ? smoothstep(splitDistance - blendRange, splitDistance + blendRange, viewDistance)
                        : (viewDistance > splitDistance ? 1.0 : 0.0);
      float nearShadow = ComputeShadowFactor(
          vLightClipPos,
          N,
          keyDir,
          uShadowPcfRadius,
          false,
          vWorldPos,
          fastShadow);
      float farShadow = ComputeShadowFactor(
          vLightClipPosFar,
          N,
          keyDir,
          uShadowPcfRadius,
          true,
          vWorldPos + vec3(11.7, 23.1, 5.3),
          fastShadow);
      shadow = mix(nearShadow, farShadow, blend);
    } else {
      shadow = ComputeShadowFactor(
          vLightClipPos,
          N,
          keyDir,
          uShadowPcfRadius,
          false,
          vWorldPos,
          fastShadow);
    }
  }

  vec3 linearColor = vec3(0.0);
  vec3 directContribution = vec3(0.0);
  if (uShadingMode == 0) {
    vec3 direct = vec3(0.0);
    direct += ShadePbrLight(
        N, V, keyDir, vec3(1.00, 0.98, 0.94), max(uKeyLightIntensity, 0.0), albedo, roughness, metallic, F0);
    direct += ShadePbrLight(
        N, V, fillDir, vec3(0.60, 0.70, 0.85), max(uFillLightIntensity, 0.0), albedo, roughness, metallic, F0);
    float clearcoat = clamp(uClearcoatStrength * (1.0 - 0.45 * roughness), 0.0, 1.0);
    vec3 clearcoatDirect = vec3(0.0);
    clearcoatDirect += ShadeClearcoatLight(
        N, V, keyDir, vec3(1.00, 0.98, 0.94), max(uKeyLightIntensity, 0.0), clearcoat);
    clearcoatDirect += ShadeClearcoatLight(
        N, V, fillDir, vec3(0.60, 0.70, 0.85), max(uFillLightIntensity, 0.0), clearcoat * 0.8);

    float rim = pow(1.0 - max(dot(N, V), 0.0), 2.6) * max(uRimStrength, 0.0);
    vec3 rimColor = vec3(0.85, 0.92, 1.0) * rim;

    vec3 R = reflect(-V, N);
    float envMix = clamp(R.z * 0.5 + 0.5, 0.0, 1.0);
    vec3 envColor = mix(vec3(0.10, 0.11, 0.12), vec3(0.30, 0.36, 0.46), envMix);
    vec3 envFresnel = FresnelSchlick(max(dot(N, V), 0.0), F0);
    vec3 envSpec = envColor * envFresnel * (1.0 - roughness * 0.65) * 0.32;
    vec3 clearcoatEnv = envColor * FresnelSchlick(max(dot(N, V), 0.0), vec3(0.04)) * clearcoat * 0.22;

    float depthCue = 1.0 - clamp(uDepthCueStrength, 0.0, 2.0) * clamp(viewDistance / 120.0, 0.0, 1.0);
    vec3 directLit = (direct + clearcoatDirect + rimColor) * max(depthCue, 0.45) * shadowContrast;
    vec3 indirectLit = (ambient + envSpec + clearcoatEnv + bounceLight) * max(depthCue, 0.45);
    vec3 shadowFade = mix(vec3(1.0), shadowTint, shadow * clamp(uShadowStrength, 0.0, 1.0));
    directLit *= shadowFade;
    directContribution = directLit;
    linearColor = indirectLit + directLit;
  } else if (uShadingMode == 1) {
    float nearView = 1.0 - smoothstep(4.0, 14.0, viewDistance);
    float farView = 1.0 - nearView;
    float ndotl = max(dot(N, keyDir), 0.0);
    float ndotf = max(dot(N, fillDir), 0.0);
    float toonBandCount = mix(clamp(arcadeBands + 1.0, 2.0, 5.0), clamp(arcadeBands, 2.0, 4.0), farView);
    float toonSoftness = mix(0.68, 0.42, farView);
    float toonKey = ToonRampSmooth(ndotl, toonBandCount, toonSoftness);
    float toonFill = ToonRampSmooth(ndotf, 2.0, mix(0.72, 0.50, farView));
    float motionBoost = 1.0 + uArcadeMotion * mix(0.35, 0.75, farView);
    vec3 themeLit = mix(themeSecondary, themePrimary, toonKey);
    vec3 arcadeTint = (0.26 + 0.56 * themeLit);
    vec3 arcadeAlbedo = mix(albedo, albedo * arcadeTint, clamp(arcadeThemeTintStrength * 1.25, 0.0, 1.0));
    vec3 warmHighlight = mix(vec3(0.90, 0.50, 0.24), themeSecondary * 0.95 + vec3(0.16, 0.07, 0.03), 0.60);
    vec3 direct = arcadeAlbedo * warmHighlight *
                  (0.15 + toonKey * max(uKeyLightIntensity, 0.0) * 0.82 +
                   toonFill * max(uFillLightIntensity, 0.0) * 0.22);
    vec3 H = normalize(V + keyDir);
    float specRaw = max(dot(N, H), 0.0);
    float hardSpec = step(0.925, specRaw) * 0.55 + step(0.975, specRaw) * 0.45;
    float specDistanceScale = mix(0.45, 1.00, farView);
    hardSpec *= max(uSpecularStrength, 0.0) * (0.35 + 0.95 * toonKey) * arcadeHighlightBoost * motionBoost * specDistanceScale;
    vec3 specColor = mix(themePrimary * 0.85 + vec3(0.04, 0.14, 0.24), vec3(0.82, 0.95, 1.00), 0.35);
    vec3 specularArc = specColor * hardSpec;

    float rimWide = pow(1.0 - max(dot(N, V), 0.0), 0.95);
    vec3 rimColor = mix(themePrimary * 1.15 + vec3(0.04, 0.06, 0.10), vec3(0.44, 0.82, 1.0), 0.45);
    float rimDistanceScale = mix(0.55, 1.00, farView);
    vec3 rimLight = rimColor * rimWide * max(uRimStrength, 0.0) * (1.85 + arcadeRimBoost * 2.6) * motionBoost * rimDistanceScale;

    float depthCue = 1.0 - clamp(uDepthCueStrength, 0.0, 2.0) * clamp(viewDistance / 170.0, 0.0, 1.0);
    float shadowAmount = shadow * clamp(uShadowStrength, 0.0, 1.0);
    float shadowBand = ToonRampSmooth(shadowAmount, 3.0, mix(0.74, 0.55, farView));
    vec3 shadowGrade = mix(vec3(1.0), shadowTint * 0.70 + vec3(0.03, 0.05, 0.10), shadowBand * (0.88 - 0.52 * arcadeShadowLift));
    vec3 directLit = (direct + specularArc + rimLight) * max(depthCue, 0.74) * shadowGrade;
    vec3 indirectLit = ambient * max(depthCue, 0.78) * 0.50;
    directContribution = directLit;
    linearColor = indirectLit + directLit;

    float floorMask = smoothstep(0.86, 0.98, abs(geomN.z));
    vec3 floorTint = mix(themeSecondary, themePrimary, 0.45) * vec3(0.10, 0.14, 0.20);
    linearColor += floorMask * floorTint * 0.010;

    float outlineBand = smoothstep(0.24, 0.62, 1.0 - max(dot(N, V), 0.0));
    vec3 outlineColor = mix(vec3(0.02, 0.03, 0.05), themePrimary * 0.42 + vec3(0.03, 0.04, 0.06), 0.65);
    linearColor = mix(linearColor, outlineColor, outlineBand * arcadeOutlineStrength * 0.12);
  } else {
    float ndotl = max(dot(N, keyDir), 0.0);
    float ndotf = max(dot(N, fillDir), 0.0);
    vec3 direct = albedo * (ndotl * max(uKeyLightIntensity, 0.0) + ndotf * max(uFillLightIntensity, 0.0) * 0.65);
    vec3 H = normalize(V + keyDir);
    float spec = pow(max(dot(N, H), 0.0), mix(24.0, 6.0, roughness)) * max(uSpecularStrength, 0.0) * 0.18;
    float depthCue = 1.0 - clamp(uDepthCueStrength, 0.0, 2.0) * clamp(viewDistance / 140.0, 0.0, 1.0);
    vec3 directLit = (direct + vec3(spec)) * max(depthCue, 0.55) * shadowContrast;
    vec3 indirectLit = (ambient + bounceLight) * max(depthCue, 0.55);
    vec3 shadowFade = mix(vec3(1.0), shadowTint, shadow * clamp(uShadowStrength, 0.0, 1.0));
    directLit *= shadowFade;
    directContribution = directLit;
    linearColor = indirectLit + directLit;
  }

  // Stylized state-driven emissive accents (kept stable by default, stronger under gameplay state).
  float alertStrobe = arcadeAlert * (0.70 + 0.30 * step(0.60, fract(uTimeS * 1.8)));
  float selectedEdge = arcadeSelected * smoothstep(0.28, 0.74, 1.0 - max(dot(N, V), 0.0));
  float chargedWave = arcadeCharged * (0.78 + 0.22 * sin(uTimeS * 3.2));
  float motionEdge = uArcadeMotion * smoothstep(0.36, 0.84, 1.0 - max(dot(N, V), 0.0));
  vec3 neonBlue = mix(vec3(0.08, 0.66, 1.00), themePrimary, themeMix);
  vec3 neonWarm = mix(vec3(1.00, 0.34, 0.20), themeSecondary, themeMix);
  float motionBoost = 1.0 + uArcadeMotion * 0.45;
  vec3 emissive =
      neonBlue * (0.100 * selectedEdge + 0.075 * chargedWave + 0.040 * motionEdge) +
      neonWarm * (0.150 * alertStrobe + 0.050 * chargedWave) +
      mix(neonBlue, neonWarm, 0.35) * (0.060 * selectedEdge * arcadeHighlightBoost);
  emissive *= themeEmissiveStrength * mix(1.0, arcadeEmissiveBoost * motionBoost, arcadeModeF);
  if (uShadingMode == 2) {
    emissive *= 0.60;
  }
  linearColor += emissive;

  float nightBlend = clamp(uNightMode * uNightIntensity, 0.0, 1.0);
  if (nightBlend > 1e-4) {
    vec3 nightAmbient = mix(vec3(0.03, 0.05, 0.08), vec3(0.05, 0.07, 0.11), up) * albedo;
    linearColor = mix(linearColor, nightAmbient + emissive * 1.38 + directContribution * 0.72, nightBlend);
  }

  float fogDensity = max(uFogDensity, 0.0) * mix(1.0, arcadeFogScale, arcadeModeF);
  float fog = 1.0 - exp(-fogDensity * viewDistance);
  vec3 fogColor = mix(vec3(0.49, 0.56, 0.68), vec3(0.11, 0.15, 0.22), nightBlend);
  if (arcadeMode) {
    vec3 arcadeFogColor =
        mix(vec3(0.06, 0.10, 0.17), mix(themePrimary, themeSecondary, 0.38) * 0.46 + vec3(0.03, 0.05, 0.08), 0.72);
    fogColor = mix(fogColor, arcadeFogColor, 0.82);
  }
  linearColor = mix(linearColor, fogColor, clamp(fog, 0.0, 1.0));

  float effectiveExposure = max(uExposure, 0.01);
  vec3 mapped = TonemapAces(linearColor * effectiveExposure);
  if (arcadeMode) {
    effectiveExposure *= 0.80;
    vec3 arcadeMapped = (linearColor * effectiveExposure) / (linearColor * effectiveExposure + vec3(0.85));
    mapped = pow(clamp(arcadeMapped, 0.0, 1.0), vec3(0.95));
    mapped = min(mapped, vec3(0.92, 0.94, 0.96));
  }
  if (arcadeMode) {
    float nearTone = 1.0 - smoothstep(4.0, 14.0, viewDistance);
    float peak = max(max(mapped.r, mapped.g), mapped.b);
    float over = max(0.0, peak - 0.58);
    mapped *= (1.0 / (1.0 + over * mix(2.8, 2.2, 1.0 - nearTone)));
  }
  float luma = dot(mapped, vec3(0.2126, 0.7152, 0.0722));
  mapped = mix(vec3(luma), mapped, clamp(uSaturation, 0.0, 2.0));
  if (arcadeMode) {
    mapped = pow(clamp(mapped, 0.0, 1.0), vec3(0.92));
    mapped *= vec3(1.08, 1.07, 1.06);
  }
  if (arcadeMode) {
    float bloomMask = max(max(emissive.r, emissive.g), emissive.b);
    vec3 bloomTint = mix(themePrimary, themeSecondary, 0.40) * 0.18 + vec3(0.02, 0.03, 0.05);
    mapped += bloomTint * bloomMask * arcadeBloomStrength * 0.22;
  }
  float postFx = clamp(uPostFxStrength, 0.0, 2.0);
  if (postFx > 1e-4) {
    vec2 viewport = max(uViewportInfo.xy, vec2(1.0, 1.0));
    vec2 uv = gl_FragCoord.xy / viewport;
    vec2 centered = uv * 2.0 - 1.0;
    float radius = length(centered);
    float vignette = 1.0 - clamp(radius * radius * clamp(uVignetteStrength, 0.0, 2.0), 0.0, 0.92);
    float grain = (Hash13(vec3(gl_FragCoord.xy * 0.0127, uTimeS * 7.31)) - 0.5) *
                  clamp(uFilmGrainStrength, 0.0, 0.5);
    float chromaEdge = clamp(radius * clamp(uChromaticStrength, 0.0, 0.5), 0.0, 0.5);
    vec3 chromaTint = vec3(1.0 + chromaEdge, 1.0, 1.0 - chromaEdge);
    mapped = mapped * mix(vec3(1.0), chromaTint, postFx * 0.7);
    mapped = mapped * mix(1.0, vignette, postFx);
    if (arcadeMode) {
      mapped = mix(mapped, smoothstep(vec3(0.0), vec3(1.0), mapped), postFx * 0.18);
    }
    mapped += vec3(grain);
  }
  mapped = pow(max(mapped, vec3(0.0)), vec3(1.0 / max(uGamma, 1e-3)));
  FragColor = vec4(mapped, alpha);
}
)";
}

}  // namespace repulsor3d::cad
