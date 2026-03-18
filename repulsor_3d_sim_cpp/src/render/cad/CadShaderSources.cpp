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
uniform mat4 uViewProjection;
uniform mat4 uLightViewProjection;
out vec4 vColor;
out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec4 vLightClipPos;
void main() {
  mat4 model = mat4(iModelRow0, iModelRow1, iModelRow2, iModelRow3);
  vec4 worldPos = model * vec4(aPos, 1.0);
  vWorldPos = worldPos.xyz;
  mat3 normalMatrix = transpose(inverse(mat3(model)));
  vWorldNormal = normalize(normalMatrix * aNormal);
  vColor = aColor;
  vLightClipPos = uLightViewProjection * worldPos;
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
uniform float uUseAssetColor;
uniform sampler2D uShadowMap;
uniform sampler2D uShadowMapFar;
uniform int uShadowEnabled;
uniform float uShadowStrength;
uniform int uShadowPcfRadius;
uniform int uShadowCascadeCount;
uniform float uShadowCascadeSplitM;
out vec4 FragColor;

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

float ComputeShadowFactor(vec4 lightClipPos, vec3 normal, vec3 lightDir, int radius, bool useFarCascade) {
  if (uShadowEnabled == 0) {
    return 0.0;
  }

  vec3 proj = lightClipPos.xyz / max(lightClipPos.w, 1e-6);
  proj = proj * 0.5 + 0.5;
  if (proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0 || proj.z < 0.0 || proj.z > 1.0) {
    return 0.0;
  }

  float ndotl = max(dot(normalize(normal), normalize(lightDir)), 0.0);
  float bias = mix(0.0018, 0.0005, ndotl);
  vec2 texelSize = useFarCascade
                       ? (1.0 / vec2(textureSize(uShadowMapFar, 0)))
                       : (1.0 / vec2(textureSize(uShadowMap, 0)));

  float shadowHits = 0.0;
  float taps = 0.0;
  int pcfRadius = clamp(radius, 0, 4);
  for (int x = -pcfRadius; x <= pcfRadius; ++x) {
    for (int y = -pcfRadius; y <= pcfRadius; ++y) {
      vec2 uv = proj.xy + vec2(float(x), float(y)) * texelSize;
      float closest = useFarCascade ? texture(uShadowMapFar, uv).r : texture(uShadowMap, uv).r;
      shadowHits += (proj.z - bias > closest) ? 1.0 : 0.0;
      taps += 1.0;
    }
  }
  return shadowHits / max(taps, 1.0);
}

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

void main() {
  float useAssetColor = clamp(uUseAssetColor, 0.0, 1.0);
  vec3 baseColorSrgb = mix(uColor.rgb, vColor.rgb * uColor.rgb, useAssetColor);
  float alpha = uColor.a * mix(1.0, vColor.a, useAssetColor);

  vec3 N = normalize(vWorldNormal);
  vec3 V = normalize(uCameraPos - vWorldPos);
  vec3 albedo = pow(clamp(baseColorSrgb, 0.0, 1.0), vec3(2.2));

  float roughness = clamp(uRoughness, 0.03, 1.0);
  float metallic = clamp(uMetallic, 0.0, 1.0);
  vec3 F0 = mix(vec3(0.04), albedo, metallic) * max(uSpecularStrength, 0.0);

  vec3 keyDir = normalize(-uLightDir);
  vec3 fillDir = normalize(vec3(0.40, 0.55, 0.72));

  vec3 direct = vec3(0.0);
  direct += ShadePbrLight(
      N, V, keyDir, vec3(1.00, 0.98, 0.94), max(uKeyLightIntensity, 0.0), albedo, roughness, metallic, F0);
  direct += ShadePbrLight(
      N, V, fillDir, vec3(0.60, 0.70, 0.85), max(uFillLightIntensity, 0.0), albedo, roughness, metallic, F0);

  float up = clamp(N.z * 0.5 + 0.5, 0.0, 1.0);
  vec3 skyAmbient = vec3(0.44, 0.52, 0.64);
  vec3 groundAmbient = vec3(0.19, 0.18, 0.17);
  vec3 ambient = mix(groundAmbient, skyAmbient, up) * albedo * max(uAmbientStrength, 0.0);

  float rim = pow(1.0 - max(dot(N, V), 0.0), 2.6) * max(uRimStrength, 0.0);
  vec3 rimColor = vec3(0.85, 0.92, 1.0) * rim;

  vec3 R = reflect(-V, N);
  float envMix = clamp(R.z * 0.5 + 0.5, 0.0, 1.0);
  vec3 envColor = mix(vec3(0.10, 0.11, 0.12), vec3(0.30, 0.36, 0.46), envMix);
  vec3 envFresnel = FresnelSchlick(max(dot(N, V), 0.0), F0);
  vec3 envSpec = envColor * envFresnel * (1.0 - roughness * 0.65) * 0.32;

  float viewDistance = length(uCameraPos - vWorldPos);
  float depthCue = 1.0 - clamp(uDepthCueStrength, 0.0, 2.0) * clamp(viewDistance / 120.0, 0.0, 1.0);
  float keyShadow = max(dot(N, keyDir), 0.0);
  float shadowContrast = mix(0.70, 1.0, keyShadow);
  vec3 linearColor = (ambient + direct + rimColor + envSpec) * max(depthCue, 0.45) * shadowContrast;
  float splitDistance = max(uShadowCascadeSplitM, 1.0);
  bool useFarCascade = (uShadowCascadeCount > 1) && (viewDistance > splitDistance);
  float shadow = useFarCascade
                     ? ComputeShadowFactor(vLightClipPos, N, keyDir, uShadowPcfRadius, true)
                     : ComputeShadowFactor(vLightClipPos, N, keyDir, uShadowPcfRadius, false);
  linearColor *= mix(1.0, 1.0 - clamp(uShadowStrength, 0.0, 1.0), shadow);

  float fog = 1.0 - exp(-max(uFogDensity, 0.0) * viewDistance);
  vec3 fogColor = vec3(0.57, 0.62, 0.70);
  linearColor = mix(linearColor, fogColor, clamp(fog, 0.0, 1.0));

  vec3 mapped = TonemapAces(linearColor * max(uExposure, 0.01));
  float luma = dot(mapped, vec3(0.2126, 0.7152, 0.0722));
  mapped = mix(vec3(luma), mapped, clamp(uSaturation, 0.0, 2.0));
  mapped = pow(max(mapped, vec3(0.0)), vec3(1.0 / max(uGamma, 1e-3)));
  FragColor = vec4(mapped, alpha);
}
)";
}

}  // namespace repulsor3d::cad
