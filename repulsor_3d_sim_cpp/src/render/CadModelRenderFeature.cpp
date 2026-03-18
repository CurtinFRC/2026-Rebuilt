#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/cad/CadFrustum.hpp"
#include "repulsor3d/render/cad/CadShaderSources.hpp"

namespace repulsor3d {
namespace {

std::uint32_t FloatBits(const float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

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

struct CadVisualPolicy {
  bool enableFrustumCulling = true;
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

const CadVisualPolicy& LoadCadVisualPolicy() {
  static const CadVisualPolicy policy = []() {
    CadVisualPolicy out;
    out.enableFrustumCulling = GetEnvBool("CAD_FRUSTUM_CULLING", out.enableFrustumCulling);
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

constexpr std::uint64_t kFnv64OffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnv64Prime = 1099511628211ULL;
constexpr std::uint32_t kPreparedCacheMagic = 0x524C4F44U;  // "RLOD"
constexpr std::uint32_t kPreparedCacheVersion = 1U;

void FnvMixBytes(std::uint64_t& hash, const std::uint8_t* data, const std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    hash ^= static_cast<std::uint64_t>(data[i]);
    hash *= kFnv64Prime;
  }
}

void FnvMixString(std::uint64_t& hash, const std::string& text) {
  FnvMixBytes(hash, reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
}

void FnvMixU64(std::uint64_t& hash, const std::uint64_t value) {
  std::array<std::uint8_t, sizeof(std::uint64_t)> bytes{};
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::uint8_t>((value >> (i * 8U)) & 0xFFU);
  }
  FnvMixBytes(hash, bytes.data(), bytes.size());
}

std::string ToHexString(const std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << value;
  return out.str();
}

struct AssetFileMetadata {
  std::uint64_t fileSize = 0;
  std::uint64_t modifiedTicks = 0;
  bool found = false;
};

AssetFileMetadata ProbeAssetFileMetadata(const std::string& assetPath) {
  AssetFileMetadata meta;
  std::error_code ec;

  const std::filesystem::path cwd = std::filesystem::current_path(ec);
  std::vector<std::filesystem::path> candidates;
  candidates.reserve(5);
  candidates.emplace_back(assetPath);
  if (!ec) {
    candidates.push_back(cwd / assetPath);
    candidates.push_back(cwd / "assets" / assetPath);
    if (cwd.has_parent_path()) {
      candidates.push_back(cwd.parent_path() / "assets" / assetPath);
    }
    if (cwd.has_parent_path() && cwd.parent_path().has_parent_path()) {
      candidates.push_back(cwd.parent_path().parent_path() / "assets" / assetPath);
    }
  }

  for (const auto& candidate : candidates) {
    std::error_code existsEc;
    if (!std::filesystem::exists(candidate, existsEc) || existsEc) {
      continue;
    }
    if (!std::filesystem::is_regular_file(candidate, existsEc) || existsEc) {
      continue;
    }

    const auto fileSize = std::filesystem::file_size(candidate, ec);
    if (ec) {
      continue;
    }
    const auto writeTime = std::filesystem::last_write_time(candidate, ec);
    if (ec) {
      continue;
    }

    meta.fileSize = static_cast<std::uint64_t>(fileSize);
    meta.modifiedTicks = static_cast<std::uint64_t>(writeTime.time_since_epoch().count());
    meta.found = true;
    return meta;
  }
  return meta;
}

std::string BuildPreparedCacheKey(const std::string& assetPath, const CadLodPolicy& policy) {
  std::uint64_t hash = kFnv64OffsetBasis;
  FnvMixString(hash, assetPath);

  const AssetFileMetadata meta = ProbeAssetFileMetadata(assetPath);
  FnvMixU64(hash, meta.fileSize);
  FnvMixU64(hash, meta.modifiedTicks);
  FnvMixU64(hash, meta.found ? 1ULL : 0ULL);

  FnvMixU64(hash, static_cast<std::uint64_t>(policy.maxLodCount));
  FnvMixU64(hash, static_cast<std::uint64_t>(policy.minVertexCount));
  FnvMixU64(hash, static_cast<std::uint64_t>(policy.lod0VertexCap));
  FnvMixU64(hash, static_cast<std::uint64_t>(policy.maxPreferredDrawIndices));
  FnvMixU64(hash, static_cast<std::uint64_t>(policy.normalBins));
  FnvMixU64(hash, static_cast<std::uint64_t>(FloatBits(policy.reductionRatio)));
  FnvMixU64(hash, static_cast<std::uint64_t>(FloatBits(policy.lod0ScreenRadiusPx)));
  FnvMixU64(hash, static_cast<std::uint64_t>(FloatBits(policy.screenRadiusDecay)));
  FnvMixU64(hash, policy.keepOriginalLod0 ? 1ULL : 0ULL);
  FnvMixU64(hash, policy.preserveFlatShading ? 1ULL : 0ULL);
  FnvMixU64(hash, static_cast<std::uint64_t>(kPreparedCacheVersion));
  return ToHexString(hash);
}

std::filesystem::path GetPreparedCacheRoot() {
  if (const char* env = std::getenv("CAD_PREPARED_CACHE_DIR"); env != nullptr && *env != '\0') {
    return std::filesystem::path(env);
  }

#ifdef _WIN32
  if (const char* localAppData = std::getenv("LOCALAPPDATA"); localAppData != nullptr && *localAppData != '\0') {
    return std::filesystem::path(localAppData) / "repulsor_3d_sim_cpp" / "cad_prepared_cache";
  }
#endif
  return std::filesystem::path(".repulsor_cache") / "cad_prepared_lod";
}

struct PreparedCpuMeshBlob {
  struct LodBlob {
    std::vector<PositionNormalMesh::Vertex> vertices;
    std::vector<std::uint32_t> indices;
  };
  std::vector<LodBlob> lods;
  glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
  float boundsRadius = 1.0F;
};

template <typename T>
bool WriteScalar(std::ofstream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return static_cast<bool>(out);
}

template <typename T>
bool ReadScalar(std::ifstream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  return static_cast<bool>(in);
}

bool WriteVertex(std::ofstream& out, const PositionNormalMesh::Vertex& vertex) {
  const std::array<float, 10> payload{
      vertex.position.x,
      vertex.position.y,
      vertex.position.z,
      vertex.normal.x,
      vertex.normal.y,
      vertex.normal.z,
      vertex.color.r,
      vertex.color.g,
      vertex.color.b,
      vertex.color.a,
  };
  out.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size() * sizeof(float)));
  return static_cast<bool>(out);
}

bool ReadVertex(std::ifstream& in, PositionNormalMesh::Vertex& vertex) {
  std::array<float, 10> payload{};
  in.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size() * sizeof(float)));
  if (!in) {
    return false;
  }
  vertex.position = glm::vec3{payload[0], payload[1], payload[2]};
  vertex.normal = glm::vec3{payload[3], payload[4], payload[5]};
  vertex.color = glm::vec4{payload[6], payload[7], payload[8], payload[9]};
  return true;
}

bool TryLoadPreparedCpuMeshCache(
    const std::string& assetPath,
    const CadLodPolicy& policy,
    PreparedCpuMeshBlob& outPrepared) {
  const std::filesystem::path cachePath = GetPreparedCacheRoot() / (BuildPreparedCacheKey(assetPath, policy) + ".bin");
  std::ifstream in(cachePath, std::ios::binary);
  if (!in) {
    return false;
  }

  std::uint32_t magic = 0;
  std::uint32_t version = 0;
  std::uint32_t lodCount = 0;
  if (!ReadScalar(in, magic) || !ReadScalar(in, version) || !ReadScalar(in, lodCount)) {
    return false;
  }
  if (magic != kPreparedCacheMagic || version != kPreparedCacheVersion || lodCount == 0 || lodCount > 8) {
    return false;
  }

  std::array<float, 3> center{};
  float radius = 1.0F;
  in.read(reinterpret_cast<char*>(center.data()), static_cast<std::streamsize>(center.size() * sizeof(float)));
  if (!in || !ReadScalar(in, radius)) {
    return false;
  }

  PreparedCpuMeshBlob prepared;
  prepared.lods.reserve(lodCount);
  for (std::uint32_t i = 0; i < lodCount; ++i) {
    std::uint64_t vertexCount = 0;
    std::uint64_t indexCount = 0;
    if (!ReadScalar(in, vertexCount) || !ReadScalar(in, indexCount)) {
      return false;
    }
    if (vertexCount == 0 || vertexCount > 100000000ULL || indexCount > 300000000ULL) {
      return false;
    }

    PreparedCpuMeshBlob::LodBlob lod;
    lod.vertices.resize(static_cast<std::size_t>(vertexCount));
    lod.indices.resize(static_cast<std::size_t>(indexCount));
    for (auto& vertex : lod.vertices) {
      if (!ReadVertex(in, vertex)) {
        return false;
      }
    }
    if (!lod.indices.empty()) {
      in.read(
          reinterpret_cast<char*>(lod.indices.data()),
          static_cast<std::streamsize>(lod.indices.size() * sizeof(std::uint32_t)));
      if (!in) {
        return false;
      }
    }
    prepared.lods.push_back(std::move(lod));
  }

  prepared.boundsCenter = glm::vec3{center[0], center[1], center[2]};
  prepared.boundsRadius = radius;
  outPrepared = std::move(prepared);
  return true;
}

void StorePreparedCpuMeshCache(
    const std::string& assetPath,
    const CadLodPolicy& policy,
    const PreparedCpuMeshBlob& prepared) {
  if (prepared.lods.empty()) {
    return;
  }

  std::error_code ec;
  const std::filesystem::path root = GetPreparedCacheRoot();
  std::filesystem::create_directories(root, ec);
  if (ec) {
    return;
  }

  const std::filesystem::path finalPath = root / (BuildPreparedCacheKey(assetPath, policy) + ".bin");
  const std::filesystem::path tmpPath = finalPath.string() + ".tmp";

  std::ofstream out(tmpPath, std::ios::binary | std::ios::trunc);
  if (!out) {
    return;
  }

  const std::uint32_t lodCount = static_cast<std::uint32_t>(prepared.lods.size());
  const std::array<float, 3> center{
      prepared.boundsCenter.x,
      prepared.boundsCenter.y,
      prepared.boundsCenter.z,
  };
  if (!WriteScalar(out, kPreparedCacheMagic) ||
      !WriteScalar(out, kPreparedCacheVersion) ||
      !WriteScalar(out, lodCount)) {
    return;
  }
  out.write(reinterpret_cast<const char*>(center.data()), static_cast<std::streamsize>(center.size() * sizeof(float)));
  if (!out || !WriteScalar(out, prepared.boundsRadius)) {
    return;
  }

  for (const auto& lod : prepared.lods) {
    const std::uint64_t vertexCount = static_cast<std::uint64_t>(lod.vertices.size());
    const std::uint64_t indexCount = static_cast<std::uint64_t>(lod.indices.size());
    if (!WriteScalar(out, vertexCount) || !WriteScalar(out, indexCount)) {
      return;
    }
    for (const auto& vertex : lod.vertices) {
      if (!WriteVertex(out, vertex)) {
        return;
      }
    }
    if (!lod.indices.empty()) {
      out.write(
          reinterpret_cast<const char*>(lod.indices.data()),
          static_cast<std::streamsize>(lod.indices.size() * sizeof(std::uint32_t)));
      if (!out) {
        return;
      }
    }
  }
  out.flush();
  out.close();

  std::filesystem::remove(finalPath, ec);
  ec.clear();
  std::filesystem::rename(tmpPath, finalPath, ec);
  if (ec) {
    std::filesystem::remove(tmpPath, ec);
  }
}

struct VertexKey {
  std::uint32_t px = 0;
  std::uint32_t py = 0;
  std::uint32_t pz = 0;
  std::uint32_t nx = 0;
  std::uint32_t ny = 0;
  std::uint32_t nz = 0;
  std::uint32_t cr = 0;
  std::uint32_t cg = 0;
  std::uint32_t cb = 0;
  std::uint32_t ca = 0;

  bool operator==(const VertexKey& other) const {
    return px == other.px && py == other.py && pz == other.pz &&
           nx == other.nx && ny == other.ny && nz == other.nz &&
           cr == other.cr && cg == other.cg && cb == other.cb && ca == other.ca;
  }
};

struct VertexKeyHash {
  std::size_t operator()(const VertexKey& key) const {
    std::size_t h = 1469598103934665603ULL;
    const auto mix = [&](const std::uint32_t value) {
      h ^= static_cast<std::size_t>(value);
      h *= 1099511628211ULL;
    };
    mix(key.px);
    mix(key.py);
    mix(key.pz);
    mix(key.nx);
    mix(key.ny);
    mix(key.nz);
    mix(key.cr);
    mix(key.cg);
    mix(key.cb);
    mix(key.ca);
    return h;
  }
};

VertexKey ToVertexKey(const PositionNormalMesh::Vertex& vertex) {
  return VertexKey{
      .px = FloatBits(vertex.position.x),
      .py = FloatBits(vertex.position.y),
      .pz = FloatBits(vertex.position.z),
      .nx = FloatBits(vertex.normal.x),
      .ny = FloatBits(vertex.normal.y),
      .nz = FloatBits(vertex.normal.z),
      .cr = FloatBits(vertex.color.r),
      .cg = FloatBits(vertex.color.g),
      .cb = FloatBits(vertex.color.b),
      .ca = FloatBits(vertex.color.a),
  };
}

struct IndexedMesh {
  std::vector<PositionNormalMesh::Vertex> vertices;
  std::vector<std::uint32_t> indices;
};

glm::vec3 ComputeTrimmedBoundsCenterXY(const std::vector<PositionNormalMesh::Vertex>& vertices) {
  if (vertices.empty()) {
    return glm::vec3{0.0F, 0.0F, 0.0F};
  }

  std::size_t sampleCount = vertices.size();
  std::size_t stride = 1;
  constexpr std::size_t kMaxSamples = 500000;
  if (sampleCount > kMaxSamples) {
    stride = (sampleCount + kMaxSamples - 1) / kMaxSamples;
    sampleCount = (vertices.size() + stride - 1) / stride;
  }

  std::vector<float> xs;
  std::vector<float> ys;
  xs.reserve(sampleCount);
  ys.reserve(sampleCount);
  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::lowest();
  for (std::size_t i = 0; i < vertices.size(); i += stride) {
    const glm::vec3& p = vertices[i].position;
    xs.push_back(p.x);
    ys.push_back(p.y);
    minZ = std::min(minZ, p.z);
    maxZ = std::max(maxZ, p.z);
  }
  if (xs.empty() || ys.empty()) {
    return glm::vec3{0.0F, 0.0F, 0.0F};
  }

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  const auto quantile = [](const std::vector<float>& sorted, const float q) -> float {
    const std::size_t idx = static_cast<std::size_t>(q * static_cast<float>(sorted.size() - 1));
    return sorted[idx];
  };

  const float xLo = quantile(xs, 0.01F);
  const float xHi = quantile(xs, 0.99F);
  const float yLo = quantile(ys, 0.01F);
  const float yHi = quantile(ys, 0.99F);
  return glm::vec3{(xLo + xHi) * 0.5F, (yLo + yHi) * 0.5F, (minZ + maxZ) * 0.5F};
}

float ComputeRadiusFromCenter(
    const std::vector<PositionNormalMesh::Vertex>& vertices,
    const glm::vec3& center) {
  if (vertices.empty()) {
    return 1.0F;
  }
  float radiusSq = 0.0F;
  for (const auto& vertex : vertices) {
    const glm::vec3 delta = vertex.position - center;
    radiusSq = std::max(radiusSq, glm::dot(delta, delta));
  }
  return std::max(std::sqrt(radiusSq), 1e-3F);
}

IndexedMesh BuildIndexedMesh(const PositionNormalMesh& mesh) {
  IndexedMesh indexed;
  if (mesh.vertices.empty()) {
    return indexed;
  }

  indexed.vertices.reserve(mesh.vertices.size());
  indexed.indices.reserve(mesh.vertices.size());
  std::unordered_map<VertexKey, std::uint32_t, VertexKeyHash> dedup;
  dedup.reserve(mesh.vertices.size());

  for (const auto& vertex : mesh.vertices) {
    const VertexKey key = ToVertexKey(vertex);
    const auto found = dedup.find(key);
    if (found != dedup.end()) {
      indexed.indices.push_back(found->second);
      continue;
    }
    const auto next = static_cast<std::uint32_t>(indexed.vertices.size());
    indexed.vertices.push_back(vertex);
    dedup.emplace(key, next);
    indexed.indices.push_back(next);
  }
  return indexed;
}

void CompactUsedVertices(IndexedMesh& mesh) {
  if (mesh.indices.empty() || mesh.vertices.empty()) {
    mesh.vertices.clear();
    mesh.indices.clear();
    return;
  }

  std::vector<std::uint32_t> remap(mesh.vertices.size(), std::numeric_limits<std::uint32_t>::max());
  std::vector<PositionNormalMesh::Vertex> compact;
  compact.reserve(mesh.vertices.size());

  for (auto& index : mesh.indices) {
    if (index >= remap.size()) {
      continue;
    }
    std::uint32_t mapped = remap[index];
    if (mapped == std::numeric_limits<std::uint32_t>::max()) {
      mapped = static_cast<std::uint32_t>(compact.size());
      remap[index] = mapped;
      compact.push_back(mesh.vertices[index]);
    }
    index = mapped;
  }
  mesh.vertices.swap(compact);
}

void RemoveDegenerateTriangles(IndexedMesh& mesh) {
  if (mesh.indices.size() < 3 || mesh.vertices.empty()) {
    mesh.indices.clear();
    mesh.vertices.clear();
    return;
  }

  std::vector<std::uint32_t> filtered;
  filtered.reserve(mesh.indices.size());
  constexpr float kAreaEpsilonSq = 1e-16F;

  for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    const std::uint32_t ia = mesh.indices[i + 0];
    const std::uint32_t ib = mesh.indices[i + 1];
    const std::uint32_t ic = mesh.indices[i + 2];
    if (ia == ib || ib == ic || ia == ic) {
      continue;
    }
    if (ia >= mesh.vertices.size() || ib >= mesh.vertices.size() || ic >= mesh.vertices.size()) {
      continue;
    }

    const glm::vec3& a = mesh.vertices[ia].position;
    const glm::vec3& b = mesh.vertices[ib].position;
    const glm::vec3& c = mesh.vertices[ic].position;
    const glm::vec3 cross = glm::cross(b - a, c - a);
    if (glm::dot(cross, cross) <= kAreaEpsilonSq) {
      continue;
    }
    filtered.push_back(ia);
    filtered.push_back(ib);
    filtered.push_back(ic);
  }

  mesh.indices.swap(filtered);
  CompactUsedVertices(mesh);
}

struct ClusterKey {
  int x = 0;
  int y = 0;
  int z = 0;
  int nx = 0;
  int ny = 0;
  int nz = 0;

  bool operator==(const ClusterKey& other) const {
    return x == other.x && y == other.y && z == other.z &&
           nx == other.nx && ny == other.ny && nz == other.nz;
  }
};

struct ClusterKeyHash {
  std::size_t operator()(const ClusterKey& key) const {
    std::size_t h = 1469598103934665603ULL;
    const auto mix = [&](const std::uint32_t value) {
      h ^= static_cast<std::size_t>(value);
      h *= 1099511628211ULL;
    };
    mix(static_cast<std::uint32_t>(key.x));
    mix(static_cast<std::uint32_t>(key.y));
    mix(static_cast<std::uint32_t>(key.z));
    mix(static_cast<std::uint32_t>(key.nx));
    mix(static_cast<std::uint32_t>(key.ny));
    mix(static_cast<std::uint32_t>(key.nz));
    return h;
  }
};

struct ClusterAccum {
  glm::vec3 positionSum{0.0F, 0.0F, 0.0F};
  glm::vec3 normalSum{0.0F, 0.0F, 0.0F};
  glm::vec4 colorSum{0.0F, 0.0F, 0.0F, 0.0F};
  glm::vec3 representativePosition{0.0F, 0.0F, 0.0F};
  glm::vec3 representativeNormal{0.0F, 0.0F, 1.0F};
  glm::vec4 representativeColor{1.0F, 1.0F, 1.0F, 1.0F};
  bool hasRepresentative = false;
  std::uint32_t count = 0;
};

IndexedMesh BuildClusteredMesh(
    const IndexedMesh& source,
    const int cellResolution,
    const int normalBins,
    const bool keepRepresentativePosition,
    const bool preserveFlatShading) {
  IndexedMesh out;
  if (source.vertices.empty() || source.indices.empty() || cellResolution < 2 || normalBins < 2) {
    return out;
  }

  glm::vec3 minV = source.vertices.front().position;
  glm::vec3 maxV = source.vertices.front().position;
  for (const auto& vertex : source.vertices) {
    minV = glm::min(minV, vertex.position);
    maxV = glm::max(maxV, vertex.position);
  }

  glm::vec3 extent = maxV - minV;
  extent.x = std::max(extent.x, 1e-6F);
  extent.y = std::max(extent.y, 1e-6F);
  extent.z = std::max(extent.z, 1e-6F);

  std::unordered_map<ClusterKey, std::uint32_t, ClusterKeyHash> clusterIndex;
  clusterIndex.reserve(source.vertices.size() / 3);

  std::vector<ClusterAccum> accumulators;
  accumulators.reserve(source.vertices.size() / 3);
  std::vector<std::uint32_t> oldToCluster(source.vertices.size(), std::numeric_limits<std::uint32_t>::max());

  for (std::size_t i = 0; i < source.vertices.size(); ++i) {
    const auto& vertex = source.vertices[i];
    glm::vec3 n = vertex.normal;
    if (glm::dot(n, n) < 1e-10F) {
      n = glm::vec3{0.0F, 0.0F, 1.0F};
    } else {
      n = glm::normalize(n);
    }

    const glm::vec3 normalized = (vertex.position - minV) / extent;
    const auto q = [&](const float value) -> int {
      const float clamped = std::clamp(value, 0.0F, 1.0F);
      return std::clamp(
          static_cast<int>(std::lround(clamped * static_cast<float>(cellResolution - 1))),
          0,
          cellResolution - 1);
    };
    const auto qn = [&](const float value) -> int {
      const float clamped = std::clamp(value * 0.5F + 0.5F, 0.0F, 1.0F);
      return std::clamp(
          static_cast<int>(std::lround(clamped * static_cast<float>(normalBins - 1))),
          0,
          normalBins - 1);
    };

    const ClusterKey key{
        .x = q(normalized.x),
        .y = q(normalized.y),
        .z = q(normalized.z),
        .nx = qn(n.x),
        .ny = qn(n.y),
        .nz = qn(n.z),
    };

    auto found = clusterIndex.find(key);
    std::uint32_t clusterId = 0;
    if (found == clusterIndex.end()) {
      clusterId = static_cast<std::uint32_t>(accumulators.size());
      clusterIndex.emplace(key, clusterId);
      accumulators.push_back({});
    } else {
      clusterId = found->second;
    }

    auto& accum = accumulators[clusterId];
    accum.positionSum += vertex.position;
    accum.normalSum += n;
    accum.colorSum += vertex.color;
    if (!accum.hasRepresentative) {
      accum.representativePosition = vertex.position;
      accum.representativeNormal = n;
      accum.representativeColor = vertex.color;
      accum.hasRepresentative = true;
    }
    accum.count += 1;
    oldToCluster[i] = clusterId;
  }

  out.vertices.reserve(accumulators.size());
  out.indices.reserve(source.indices.size());

  for (const auto& accum : accumulators) {
    const float inv = accum.count > 0 ? 1.0F / static_cast<float>(accum.count) : 1.0F;
    PositionNormalMesh::Vertex vertex;
    vertex.position = keepRepresentativePosition ? accum.representativePosition : (accum.positionSum * inv);
    if (preserveFlatShading) {
      vertex.normal = glm::normalize(accum.representativeNormal);
      vertex.color = accum.representativeColor;
    } else {
      const glm::vec3 averagedNormal = accum.normalSum * inv;
      if (glm::dot(averagedNormal, averagedNormal) > 1e-10F) {
        vertex.normal = glm::normalize(averagedNormal);
      } else {
        vertex.normal = glm::vec3{0.0F, 0.0F, 1.0F};
      }
      vertex.color = accum.colorSum * inv;
    }
    out.vertices.push_back(vertex);
  }

  for (const std::uint32_t index : source.indices) {
    if (index >= oldToCluster.size()) {
      continue;
    }
    out.indices.push_back(oldToCluster[index]);
  }

  RemoveDegenerateTriangles(out);
  return out;
}

IndexedMesh SimplifyByVertexClustering(
    const IndexedMesh& source,
    const std::size_t targetVertices,
    const int normalBins,
    const bool preserveFlatShading) {
  if (source.vertices.empty() || source.indices.empty() || targetVertices < 3 ||
      source.vertices.size() <= targetVertices) {
    return source;
  }
  const float targetCube = std::cbrt(static_cast<float>(targetVertices));
  int cellResolution = std::clamp(static_cast<int>(std::ceil(targetCube * 1.95F)), 12, 1536);
  IndexedMesh result = BuildClusteredMesh(source, cellResolution, normalBins, true, preserveFlatShading);
  if (result.vertices.empty() || result.indices.empty()) {
    return source;
  }

  const float tooSmallThreshold = 0.80F;
  if (result.vertices.size() < static_cast<std::size_t>(static_cast<float>(targetVertices) * tooSmallThreshold) &&
      result.vertices.size() > 0) {
    const float scale = std::cbrt(static_cast<float>(targetVertices) / static_cast<float>(result.vertices.size()));
    const int refinedResolution = std::clamp(
        static_cast<int>(std::ceil(static_cast<float>(cellResolution) * scale * 1.10F)),
        cellResolution + 1,
        2048);
    IndexedMesh refined = BuildClusteredMesh(source, refinedResolution, normalBins, true, preserveFlatShading);
    if (!refined.vertices.empty() && !refined.indices.empty() &&
        refined.vertices.size() > result.vertices.size()) {
      result = std::move(refined);
    }
  }
  return result;
}

std::vector<IndexedMesh> BuildLodChain(const IndexedMesh& baseMesh, const CadLodPolicy& policy) {
  std::vector<IndexedMesh> lods;
  if (baseMesh.vertices.empty() || baseMesh.indices.empty()) {
    return lods;
  }

  lods.reserve(policy.maxLodCount);

  IndexedMesh lod0 = baseMesh;
  if (!policy.keepOriginalLod0 && lod0.vertices.size() > policy.lod0VertexCap) {
    lod0 = SimplifyByVertexClustering(baseMesh, policy.lod0VertexCap, policy.normalBins, policy.preserveFlatShading);
  }
  if (lod0.vertices.empty() || lod0.indices.empty()) {
    lod0 = baseMesh;
  }

  lods.push_back(std::move(lod0));
  while (lods.size() < policy.maxLodCount) {
    const IndexedMesh& previous = lods.back();
    if (previous.vertices.size() <= policy.minVertexCount) {
      break;
    }

    const std::size_t requested = static_cast<std::size_t>(std::floor(static_cast<float>(previous.vertices.size()) * policy.reductionRatio));
    const std::size_t targetVertices = std::max(policy.minVertexCount, requested);
    if (targetVertices + 256 >= previous.vertices.size()) {
      break;
    }

    IndexedMesh next = SimplifyByVertexClustering(previous, targetVertices, policy.normalBins, policy.preserveFlatShading);
    if (next.vertices.empty() || next.indices.empty()) {
      break;
    }
    if (next.vertices.size() + 256 >= previous.vertices.size()) {
      break;
    }
    lods.push_back(std::move(next));
  }

  return lods;
}

float ComputeScreenSpaceRadiusPixels(
    const glm::vec3& center,
    const float radius,
    const glm::mat4& mvp,
    const int viewportWidth,
    const int viewportHeight) {
  if (radius <= 1e-6F || viewportWidth <= 0 || viewportHeight <= 0) {
    return std::numeric_limits<float>::infinity();
  }

  const glm::vec4 clipCenter = mvp * glm::vec4(center, 1.0F);
  const glm::vec4 clipOffset = mvp * glm::vec4(center + glm::vec3{radius, 0.0F, 0.0F}, 1.0F);
  if (std::abs(clipCenter.w) < 1e-6F || std::abs(clipOffset.w) < 1e-6F) {
    return std::numeric_limits<float>::infinity();
  }

  const glm::vec2 ndcCenter{clipCenter.x / clipCenter.w, clipCenter.y / clipCenter.w};
  const glm::vec2 ndcOffset{clipOffset.x / clipOffset.w, clipOffset.y / clipOffset.w};
  const float ndcRadius = glm::length(ndcOffset - ndcCenter);
  return ndcRadius * 0.5F * static_cast<float>(std::min(viewportWidth, viewportHeight));
}

struct PackedVertex {
  glm::vec3 position{0.0F, 0.0F, 0.0F};
  glm::vec3 normal{0.0F, 0.0F, 1.0F};
  std::array<std::uint8_t, 4> color{255, 255, 255, 255};
};

struct InstanceGpuData {
  glm::vec4 modelRow0{1.0F, 0.0F, 0.0F, 0.0F};
  glm::vec4 modelRow1{0.0F, 1.0F, 0.0F, 0.0F};
  glm::vec4 modelRow2{0.0F, 0.0F, 1.0F, 0.0F};
  glm::vec4 modelRow3{0.0F, 0.0F, 0.0F, 1.0F};
};

InstanceGpuData ToInstanceData(const glm::mat4& model) {
  InstanceGpuData data;
  data.modelRow0 = model[0];
  data.modelRow1 = model[1];
  data.modelRow2 = model[2];
  data.modelRow3 = model[3];
  return data;
}

std::vector<PackedVertex> PackVerticesForGpu(const std::vector<PositionNormalMesh::Vertex>& vertices) {
  std::vector<PackedVertex> packed;
  packed.reserve(vertices.size());
  const auto packColor = [](const float channel) -> std::uint8_t {
    const float clamped = std::clamp(channel, 0.0F, 1.0F);
    return static_cast<std::uint8_t>(std::lround(clamped * 255.0F));
  };

  for (const auto& vertex : vertices) {
    PackedVertex out;
    out.position = vertex.position;
    out.normal = vertex.normal;
    out.color = {
        packColor(vertex.color.r),
        packColor(vertex.color.g),
        packColor(vertex.color.b),
        packColor(vertex.color.a),
    };
    packed.push_back(out);
  }
  return packed;
}

}  // namespace

CadModelRenderFeature::CadModelRenderFeature(
    const RenderPass renderPass,
    std::string featureName,
    std::vector<std::string> dependencies)
    : renderPass_(renderPass), featureName_(std::move(featureName)), dependencies_(std::move(dependencies)) {}

void CadModelRenderFeature::FlatLitMaterialPass::Apply(const MaterialInstance& material) {
  if (colorUniformLocation_ >= 0) {
    const auto& color = material.definition.baseColor;
    backend_.SetUniformVec4(colorUniformLocation_, color.r, color.g, color.b, color.a);
  }
}

void CadModelRenderFeature::WireframeMaterialPass::Apply(const MaterialInstance& material) {
  backend_.SetWireframeMode(material.definition.wireframe);
}

CadModelRenderFeature::~CadModelRenderFeature() {
  for (auto& [_, pending] : pendingLoads_) {
    if (pending.future.valid()) {
      try {
        pending.future.wait();
      } catch (...) {
      }
    }
  }
  pendingLoads_.clear();
  for (auto& [_, pendingUpload] : pendingGpuUploads_) {
    DestroyMesh(pendingUpload.gpu);
  }
  pendingGpuUploads_.clear();

  for (auto& [_, mesh] : meshCache_) {
    DestroyMesh(mesh);
  }
  meshCache_.clear();

  shadowDepthTexture_.Reset();
  if (shadowFbo_ != 0) {
    glDeleteFramebuffers(1, &shadowFbo_);
    shadowFbo_ = 0;
  }
  shadowShader_.Reset();
  shader_.Reset();
}

bool CadModelRenderFeature::Initialize(Renderer& renderer) {
  renderer_ = &renderer;
  geometryProvider_ = &renderer.GetGeometryProvider();
  backend_ = &renderer.GetRenderBackend();
  uploadBudgetPerFrame_ = std::max(1, GetEnvInt("CAD_UPLOADS_PER_FRAME", uploadBudgetPerFrame_));
  shadowEnabled_ = GetEnvBool("CAD_SHADOWS_ENABLED", true);
  shadowMapSize_ = std::clamp(GetEnvInt("CAD_SHADOW_MAP_SIZE", shadowMapSize_), 512, 8192);
  shadowStrength_ = std::clamp(GetEnvFloat("CAD_SHADOW_STRENGTH", shadowStrength_), 0.0F, 1.0F);
  if (initialized_) {
    return true;
  }

  initialized_ = CreateShader();
  return initialized_;
}

void CadModelRenderFeature::Render(const RenderFeatureContext& context, const RendererDrawApi& /*drawApi*/) {
  if (!initialized_ || shader_.Get() == 0 || geometryProvider_ == nullptr || backend_ == nullptr) {
    return;
  }
  uploadsThisFrame_ = 0;

  if (renderPass_ == RenderPass::Opaque) {
    backend_->SetBlendEnabled(false);
  } else {
    backend_->SetBlendEnabled(true);
  }

  backend_->UseProgram(shader_.Get());
  const CadVisualPolicy& visualPolicy = LoadCadVisualPolicy();
  const auto frustumPlanes = cad::ExtractFrustumPlanes(context.viewProjection);
  backend_->SetUniformVec3(uLightDirLoc_, -0.3F, -0.5F, -1.0F);
  backend_->SetUniformVec3(
      uCameraPosLoc_,
      context.cameraWorldPosition.x,
      context.cameraWorldPosition.y,
      context.cameraWorldPosition.z);
  backend_->SetUniform1f(uDepthCueStrengthLoc_, visualPolicy.depthCueStrength);
  backend_->SetUniform1f(uSpecularStrengthLoc_, visualPolicy.specularStrength);
  backend_->SetUniform1f(uRimStrengthLoc_, visualPolicy.rimStrength);
  backend_->SetUniform1f(uKeyLightIntensityLoc_, visualPolicy.keyLightIntensity);
  backend_->SetUniform1f(uFillLightIntensityLoc_, visualPolicy.fillLightIntensity);
  backend_->SetUniform1f(uAmbientStrengthLoc_, visualPolicy.ambientStrength);
  backend_->SetUniform1f(uRoughnessLoc_, visualPolicy.roughness);
  backend_->SetUniform1f(uMetallicLoc_, visualPolicy.metallic);
  backend_->SetUniform1f(uExposureLoc_, visualPolicy.exposure);
  backend_->SetUniform1f(uFogDensityLoc_, visualPolicy.fogDensity);
  backend_->SetUniform1f(uSaturationLoc_, visualPolicy.saturation);
  backend_->SetUniform1f(uGammaLoc_, visualPolicy.gamma);

  struct DrawItem {
    const GpuMesh* mesh = nullptr;
    const GpuMesh::LodGpu* lod = nullptr;
    glm::mat4 model{1.0F};
    glm::vec4 color{1.0F};
    bool useAssetColor = false;
    bool wireframe = false;
    bool mirroredWinding = false;
  };
  std::vector<DrawItem> drawItems;
  drawItems.reserve(256);

  for (const auto& command : context.commandBuffer) {
    std::visit(
        [&](const auto& typed) {
          using CommandType = std::decay_t<decltype(typed)>;
          if constexpr (!std::is_same_v<CommandType, DrawMeshInstanceCommand>) {
            return;
          } else {
            if (typed.pass != renderPass_) {
              return;
            }

            const auto& instance = typed.primitive;
            const GpuMesh* mesh = GetOrLoadMesh(instance.assetPath);
            if (mesh == nullptr || mesh->lods.empty()) {
              return;
            }

            glm::mat4 model(1.0F);
            model = glm::translate(model, instance.position);
            model = glm::rotate(model, glm::radians(instance.rotationDeg.x), glm::vec3{1.0F, 0.0F, 0.0F});
            model = glm::rotate(model, glm::radians(instance.rotationDeg.y), glm::vec3{0.0F, 1.0F, 0.0F});
            model = glm::rotate(model, glm::radians(instance.rotationDeg.z), glm::vec3{0.0F, 0.0F, 1.0F});
            model = glm::scale(model, instance.scale);
            if (instance.centerOnMeshBounds) {
              model =
                  model * glm::translate(glm::mat4(1.0F), glm::vec3{-mesh->boundsCenter.x, -mesh->boundsCenter.y, 0.0F});
            }

            if (visualPolicy.enableFrustumCulling) {
              const glm::vec3 worldBoundsCenter = glm::vec3(model * glm::vec4(mesh->boundsCenter, 1.0F));
              const glm::vec3 scaleAxisX{model[0][0], model[0][1], model[0][2]};
              const glm::vec3 scaleAxisY{model[1][0], model[1][1], model[1][2]};
              const glm::vec3 scaleAxisZ{model[2][0], model[2][1], model[2][2]};
              const float maxScale =
                  std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
              const float worldBoundsRadius = std::max(mesh->boundsRadius * std::max(maxScale, 1e-4F), 1e-4F);
              if (!cad::IsSphereVisible(frustumPlanes, worldBoundsCenter, worldBoundsRadius)) {
                return;
              }
            }

            const glm::mat4 mvp = context.viewProjection * model;
            const std::size_t lodIndex = SelectLodLevel(*mesh, mvp, context.viewportWidth, context.viewportHeight);
            const auto& lod = mesh->lods[std::min(lodIndex, mesh->lods.size() - 1)];

            drawItems.push_back(
                {.mesh = mesh,
                 .lod = &lod,
                 .model = model,
                 .color = instance.color,
                 .useAssetColor = instance.useAssetColor,
                 .wireframe = instance.wireframe,
                 .mirroredWinding = glm::determinant(glm::mat3(model)) < 0.0F});
          }
        },
        command);
  }

  if (drawItems.empty()) {
    backend_->SetBlendEnabled(true);
    backend_->SetWireframeMode(false);
    return;
  }

  glm::vec3 sceneCenter{0.0F, 0.0F, 0.0F};
  float sceneRadius = 1.0F;
  {
    glm::vec3 minP{std::numeric_limits<float>::max()};
    glm::vec3 maxP{std::numeric_limits<float>::lowest()};
    for (const auto& item : drawItems) {
      const glm::vec3 c = glm::vec3(item.model * glm::vec4(item.mesh->boundsCenter, 1.0F));
      const glm::vec3 scaleAxisX{item.model[0][0], item.model[0][1], item.model[0][2]};
      const glm::vec3 scaleAxisY{item.model[1][0], item.model[1][1], item.model[1][2]};
      const glm::vec3 scaleAxisZ{item.model[2][0], item.model[2][1], item.model[2][2]};
      const float maxScale =
          std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
      const float r = std::max(item.mesh->boundsRadius * std::max(maxScale, 1e-4F), 0.1F);
      minP = glm::min(minP, c - glm::vec3(r, r, r));
      maxP = glm::max(maxP, c + glm::vec3(r, r, r));
    }
    sceneCenter = 0.5F * (minP + maxP);
    sceneRadius = std::max(glm::length(maxP - minP) * 0.6F, 1.0F);
  }

  const glm::vec3 lightDir = glm::normalize(glm::vec3{-0.3F, -0.5F, -1.0F});
  const glm::vec3 lightEye = sceneCenter - lightDir * (sceneRadius * 2.3F);
  const glm::mat4 lightView = glm::lookAt(lightEye, sceneCenter, glm::vec3{0.0F, 0.0F, 1.0F});
  const glm::mat4 lightProj =
      glm::ortho(-sceneRadius, sceneRadius, -sceneRadius, sceneRadius, 0.1F, sceneRadius * 5.0F + 0.1F);
  const glm::mat4 lightViewProjection = lightProj * lightView;

  if (shadowEnabled_ && shadowFbo_ != 0 && shadowShader_.Get() != 0) {
    GLint oldViewport[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_VIEWPORT, oldViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glViewport(0, 0, shadowMapSize_, shadowMapSize_);
    glClear(GL_DEPTH_BUFFER_BIT);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glCullFace(GL_FRONT);
    backend_->UseProgram(shadowShader_.Get());
    backend_->SetUniformMat4(uShadowLightMvpLoc_, &lightViewProjection[0][0]);

    struct ShadowBatch {
      const GpuMesh::LodGpu* lod = nullptr;
      bool mirroredWinding = false;
      std::vector<InstanceGpuData> instances;
    };
    std::vector<ShadowBatch> shadowBatches;
    shadowBatches.reserve(drawItems.size());
    for (const auto& item : drawItems) {
      auto it = std::find_if(shadowBatches.begin(), shadowBatches.end(), [&](const ShadowBatch& batch) {
        return batch.lod == item.lod && batch.mirroredWinding == item.mirroredWinding;
      });
      if (it == shadowBatches.end()) {
        ShadowBatch batch;
        batch.lod = item.lod;
        batch.mirroredWinding = item.mirroredWinding;
        batch.instances.push_back(ToInstanceData(item.model));
        shadowBatches.push_back(std::move(batch));
      } else {
        it->instances.push_back(ToInstanceData(item.model));
      }
    }

    for (const auto& batch : shadowBatches) {
      if (batch.lod == nullptr || batch.instances.empty()) {
        continue;
      }
      if (batch.mirroredWinding) {
        glFrontFace(GL_CW);
      }
      backend_->BindVertexArray(batch.lod->vao.Get());
      backend_->BindArrayBuffer(batch.lod->instanceVbo.Get());
      backend_->UploadArrayBufferData(batch.instances.size() * sizeof(InstanceGpuData), batch.instances.data(), true);
      if (batch.lod->indexCount > 0) {
        backend_->DrawIndexedTrianglesInstanced(batch.lod->indexCount, static_cast<int>(batch.instances.size()));
      } else {
        backend_->DrawTrianglesInstanced(batch.lod->vertexCount, static_cast<int>(batch.instances.size()));
      }
      backend_->BindVertexArray(0);
      if (batch.mirroredWinding) {
        glFrontFace(GL_CCW);
      }
    }

    glCullFace(GL_BACK);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
  }

  backend_->UseProgram(shader_.Get());
  backend_->SetUniformMat4(uMvpLoc_, &context.viewProjection[0][0]);
  backend_->SetUniformMat4(uLightViewProjectionLoc_, &lightViewProjection[0][0]);
  backend_->SetUniform1i(uShadowEnabledLoc_, (shadowEnabled_ && shadowFbo_ != 0) ? 1 : 0);
  backend_->SetUniform1f(uShadowStrengthLoc_, shadowStrength_);
  backend_->SetActiveTextureUnit(3);
  backend_->BindTexture2D(shadowDepthTexture_.Get());
  backend_->SetUniform1i(uShadowMapLoc_, 3);

  struct BatchKey {
    const GpuMesh::LodGpu* lod = nullptr;
    std::uint32_t colorR = 0;
    std::uint32_t colorG = 0;
    std::uint32_t colorB = 0;
    std::uint32_t colorA = 0;
    bool useAssetColor = false;
    bool wireframe = false;
    bool mirroredWinding = false;
    bool operator==(const BatchKey& other) const {
      return lod == other.lod &&
             colorR == other.colorR && colorG == other.colorG && colorB == other.colorB && colorA == other.colorA &&
             useAssetColor == other.useAssetColor && wireframe == other.wireframe &&
             mirroredWinding == other.mirroredWinding;
    }
  };
  struct BatchKeyHash {
    std::size_t operator()(const BatchKey& key) const {
      std::size_t h = reinterpret_cast<std::size_t>(key.lod);
      h ^= static_cast<std::size_t>(key.colorR) * 16777619U;
      h ^= static_cast<std::size_t>(key.colorG) * 2166136261U;
      h ^= static_cast<std::size_t>(key.colorB) * 709607U;
      h ^= static_cast<std::size_t>(key.colorA) * 1469598103U;
      h ^= key.useAssetColor ? 0x1U : 0x0U;
      h ^= key.wireframe ? 0x2U : 0x0U;
      h ^= key.mirroredWinding ? 0x4U : 0x0U;
      return h;
    }
  };

  struct DrawBatch {
    glm::vec4 color{1.0F};
    bool useAssetColor = false;
    bool wireframe = false;
    bool mirroredWinding = false;
    const GpuMesh::LodGpu* lod = nullptr;
    std::vector<InstanceGpuData> instances;
  };

  std::unordered_map<BatchKey, DrawBatch, BatchKeyHash> batches;
  batches.reserve(drawItems.size());
  for (const auto& item : drawItems) {
    BatchKey key{
        .lod = item.lod,
        .colorR = FloatBits(item.color.r),
        .colorG = FloatBits(item.color.g),
        .colorB = FloatBits(item.color.b),
        .colorA = FloatBits(item.color.a),
        .useAssetColor = item.useAssetColor,
        .wireframe = item.wireframe,
        .mirroredWinding = item.mirroredWinding,
    };
    auto& batch = batches[key];
    batch.color = item.color;
    batch.useAssetColor = item.useAssetColor;
    batch.wireframe = item.wireframe;
    batch.mirroredWinding = item.mirroredWinding;
    batch.lod = item.lod;
    batch.instances.push_back(ToInstanceData(item.model));
  }

  for (auto& [_, batch] : batches) {
    if (batch.lod == nullptr || batch.instances.empty()) {
      continue;
    }
    MaterialInstance material;
    material.definition.baseColor = batch.color;
    material.definition.wireframe = batch.wireframe;
    materialPipeline_.Apply(material);
    backend_->SetUniform1f(uUseAssetColorLoc_, batch.useAssetColor ? 1.0F : 0.0F);

    if (batch.mirroredWinding) {
      glFrontFace(GL_CW);
    }
    backend_->BindVertexArray(batch.lod->vao.Get());
    backend_->BindArrayBuffer(batch.lod->instanceVbo.Get());
    backend_->UploadArrayBufferData(batch.instances.size() * sizeof(InstanceGpuData), batch.instances.data(), true);
    if (batch.lod->indexCount > 0) {
      backend_->DrawIndexedTrianglesInstanced(batch.lod->indexCount, static_cast<int>(batch.instances.size()));
    } else {
      backend_->DrawTrianglesInstanced(batch.lod->vertexCount, static_cast<int>(batch.instances.size()));
    }
    backend_->BindVertexArray(0);
    if (batch.mirroredWinding) {
      glFrontFace(GL_CCW);
    }
  }

  backend_->SetBlendEnabled(true);
  backend_->SetWireframeMode(false);
  backend_->BindTexture2D(0);
}

bool CadModelRenderFeature::CreateShader() {
  const char* vsSrc = cad::CadMainVertexShaderSource();
  const char* fsSrc = cad::CadMainFragmentShaderSource();

  const unsigned int vs = CompileShader(GL_VERTEX_SHADER, vsSrc);
  const unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
  if (vs == 0 || fs == 0) {
    if (vs != 0) {
      glDeleteShader(vs);
    }
    if (fs != 0) {
      glDeleteShader(fs);
    }
    return false;
  }

  if (!LinkShader(shader_, vs, fs)) {
    glDeleteShader(vs);
    glDeleteShader(fs);
    return false;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);

  uMvpLoc_ = backend_->GetUniformLocation(shader_.Get(), "uViewProjection");
  uModelLoc_ = -1;
  uNormalMatrixLoc_ = -1;
  uColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uColor");
  uLightDirLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightDir");
  uCameraPosLoc_ = backend_->GetUniformLocation(shader_.Get(), "uCameraPos");
  uKeyLightIntensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uKeyLightIntensity");
  uFillLightIntensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uFillLightIntensity");
  uAmbientStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uAmbientStrength");
  uDepthCueStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uDepthCueStrength");
  uSpecularStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uSpecularStrength");
  uRimStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uRimStrength");
  uRoughnessLoc_ = backend_->GetUniformLocation(shader_.Get(), "uRoughness");
  uMetallicLoc_ = backend_->GetUniformLocation(shader_.Get(), "uMetallic");
  uExposureLoc_ = backend_->GetUniformLocation(shader_.Get(), "uExposure");
  uFogDensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uFogDensity");
  uSaturationLoc_ = backend_->GetUniformLocation(shader_.Get(), "uSaturation");
  uGammaLoc_ = backend_->GetUniformLocation(shader_.Get(), "uGamma");
  uUseAssetColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uUseAssetColor");
  uLightViewProjectionLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightViewProjection");
  uShadowMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowMap");
  uShadowEnabledLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowEnabled");
  uShadowStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowStrength");

  constexpr const char* shadowVsSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 3) in vec4 iModelRow0;
layout(location = 4) in vec4 iModelRow1;
layout(location = 5) in vec4 iModelRow2;
layout(location = 6) in vec4 iModelRow3;
uniform mat4 uLightViewProjection;
void main() {
  mat4 model = mat4(iModelRow0, iModelRow1, iModelRow2, iModelRow3);
  gl_Position = uLightViewProjection * model * vec4(aPos, 1.0);
}
)";
  constexpr const char* shadowFsSrc = R"(
#version 330 core
void main() {}
)";
  const unsigned int shadowVs = CompileShader(GL_VERTEX_SHADER, shadowVsSrc);
  const unsigned int shadowFs = CompileShader(GL_FRAGMENT_SHADER, shadowFsSrc);
  if (shadowVs == 0 || shadowFs == 0) {
    if (shadowVs != 0) {
      glDeleteShader(shadowVs);
    }
    if (shadowFs != 0) {
      glDeleteShader(shadowFs);
    }
    return false;
  }
  if (!LinkShader(shadowShader_, shadowVs, shadowFs)) {
    glDeleteShader(shadowVs);
    glDeleteShader(shadowFs);
    return false;
  }
  glDeleteShader(shadowVs);
  glDeleteShader(shadowFs);
  uShadowLightMvpLoc_ = backend_->GetUniformLocation(shadowShader_.Get(), "uLightViewProjection");

  if (shadowEnabled_) {
    const unsigned int tex = backend_->CreateTexture2D();
    shadowDepthTexture_.Set(tex);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture_.Get());
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_DEPTH_COMPONENT24,
        shadowMapSize_,
        shadowMapSize_,
        0,
        GL_DEPTH_COMPONENT,
        GL_FLOAT,
        nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float border[] = {1.0F, 1.0F, 1.0F, 1.0F};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

    glGenFramebuffers(1, &shadowFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowDepthTexture_.Get(), 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  materialPipeline_.AddPass(std::make_unique<FlatLitMaterialPass>(*backend_, uColorLoc_));
  materialPipeline_.AddPass(std::make_unique<WireframeMaterialPass>(*backend_));
  return uMvpLoc_ >= 0 &&
         uColorLoc_ >= 0 && uLightDirLoc_ >= 0 && uCameraPosLoc_ >= 0 &&
         uKeyLightIntensityLoc_ >= 0 && uFillLightIntensityLoc_ >= 0 &&
         uAmbientStrengthLoc_ >= 0 &&
         uDepthCueStrengthLoc_ >= 0 && uSpecularStrengthLoc_ >= 0 &&
         uRimStrengthLoc_ >= 0 && uRoughnessLoc_ >= 0 && uMetallicLoc_ >= 0 &&
         uExposureLoc_ >= 0 && uFogDensityLoc_ >= 0 && uSaturationLoc_ >= 0 &&
         uGammaLoc_ >= 0 && uUseAssetColorLoc_ >= 0 &&
         uLightViewProjectionLoc_ >= 0 && uShadowMapLoc_ >= 0 &&
         uShadowEnabledLoc_ >= 0 && uShadowStrengthLoc_ >= 0 && uShadowLightMvpLoc_ >= 0;
}

unsigned int CadModelRenderFeature::CompileShader(const unsigned int type, const char* source) {
  const unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);

  int ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_TRUE) {
    return shader;
  }

  int length = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(std::max(0, length)), '\0');
  if (length > 0) {
    glGetShaderInfoLog(shader, length, nullptr, log.data());
  }
  std::cerr << "CAD shader compile error: " << log << "\n";
  glDeleteShader(shader);
  return 0;
}

bool CadModelRenderFeature::LinkShader(GlProgramHandle& program, const unsigned int vs, const unsigned int fs) {
  const unsigned int p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);

  int ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) {
    program.Set(p);
    return true;
  }

  int length = 0;
  glGetProgramiv(p, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(std::max(0, length)), '\0');
  if (length > 0) {
    glGetProgramInfoLog(p, length, nullptr, log.data());
  }
  std::cerr << "CAD shader link error: " << log << "\n";
  glDeleteProgram(p);
  return false;
}

CadModelRenderFeature::PreparedCpuMesh CadModelRenderFeature::PrepareCpuMesh(const PositionNormalMesh& cpu) {
  PreparedCpuMesh prepared;
  if (cpu.vertices.empty()) {
    return prepared;
  }

  const CadLodPolicy& lodPolicy = LoadCadLodPolicy();
  IndexedMesh indexed = BuildIndexedMesh(cpu);
  RemoveDegenerateTriangles(indexed);
  if (indexed.vertices.empty() || indexed.indices.empty()) {
    return prepared;
  }

  const glm::vec3 center = ComputeTrimmedBoundsCenterXY(indexed.vertices);
  const float radius = ComputeRadiusFromCenter(indexed.vertices, center);
  std::vector<IndexedMesh> lodMeshes = BuildLodChain(indexed, lodPolicy);
  prepared.lods.reserve(lodMeshes.size());
  for (auto& lod : lodMeshes) {
    if (lod.vertices.empty() || lod.indices.empty()) {
      continue;
    }
    PreparedCpuMesh::LodCpu outLod;
    outLod.vertices = std::move(lod.vertices);
    outLod.indices = std::move(lod.indices);
    prepared.lods.push_back(std::move(outLod));
  }

  prepared.boundsCenter = center;
  prepared.boundsRadius = radius;
  return prepared;
}

bool CadModelRenderFeature::UploadMesh(const PreparedCpuMesh& cpu, GpuMesh& gpu, IRenderBackend& backend) {
  DestroyMesh(gpu);
  if (cpu.lods.empty()) {
    return false;
  }

  gpu.lods.reserve(cpu.lods.size());
  for (const auto& cpuLod : cpu.lods) {
    if (cpuLod.vertices.empty()) {
      continue;
    }

    GpuMesh::LodGpu gpuLod;
    if (!UploadSingleLod(cpuLod, gpuLod, backend)) {
      DestroyMesh(gpu);
      return false;
    }
    gpu.lods.push_back(std::move(gpuLod));
  }

  gpu.boundsCenter = cpu.boundsCenter;
  gpu.boundsRadius = cpu.boundsRadius;
  return !gpu.lods.empty();
}

bool CadModelRenderFeature::UploadSingleLod(
    const PreparedCpuMesh::LodCpu& cpuLod,
    GpuMesh::LodGpu& gpuLod,
    IRenderBackend& backend) {
  const unsigned int vao = backend.CreateVertexArray();
  const unsigned int vbo = backend.CreateBuffer();
  const unsigned int ebo = backend.CreateBuffer();
  const unsigned int instanceVbo = backend.CreateBuffer();
  if (vao == 0 || vbo == 0 || ebo == 0 || instanceVbo == 0) {
    return false;
  }

  gpuLod.vao.Set(vao);
  gpuLod.vbo.Set(vbo);
  gpuLod.ebo.Set(ebo);
  gpuLod.instanceVbo.Set(instanceVbo);

  const std::vector<PackedVertex> packed = PackVerticesForGpu(cpuLod.vertices);
  backend.BindVertexArray(gpuLod.vao.Get());
  backend.BindArrayBuffer(gpuLod.vbo.Get());
  backend.UploadArrayBufferData(packed.size() * sizeof(PackedVertex), packed.data(), false);
  backend.BindElementArrayBuffer(gpuLod.ebo.Get());
  backend.UploadElementArrayBufferData(cpuLod.indices.size() * sizeof(std::uint32_t), cpuLod.indices.data(), false);

  backend.EnableVertexAttrib(0);
  backend.DefineVertexAttribFloat(0, 3, sizeof(PackedVertex), offsetof(PackedVertex, position));
  backend.EnableVertexAttrib(1);
  backend.DefineVertexAttribFloat(1, 3, sizeof(PackedVertex), offsetof(PackedVertex, normal));
  backend.EnableVertexAttrib(2);
  backend.DefineVertexAttribNormalizedU8(2, 4, sizeof(PackedVertex), offsetof(PackedVertex, color));

  backend.BindArrayBuffer(gpuLod.instanceVbo.Get());
  backend.UploadArrayBufferData(sizeof(InstanceGpuData), nullptr, true);
  backend.EnableVertexAttrib(3);
  backend.DefineVertexAttribFloat(3, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow0));
  backend.EnableVertexAttrib(4);
  backend.DefineVertexAttribFloat(4, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow1));
  backend.EnableVertexAttrib(5);
  backend.DefineVertexAttribFloat(5, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow2));
  backend.EnableVertexAttrib(6);
  backend.DefineVertexAttribFloat(6, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow3));
  backend.SetVertexAttribDivisor(3, 1);
  backend.SetVertexAttribDivisor(4, 1);
  backend.SetVertexAttribDivisor(5, 1);
  backend.SetVertexAttribDivisor(6, 1);

  backend.BindVertexArray(0);

  gpuLod.vertexCount = static_cast<int>(cpuLod.vertices.size());
  gpuLod.indexCount = static_cast<int>(cpuLod.indices.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  for (auto& lod : gpu.lods) {
    lod.instanceVbo.Reset();
    lod.ebo.Reset();
    lod.vbo.Reset();
    lod.vao.Reset();
    lod.vertexCount = 0;
    lod.indexCount = 0;
  }
  gpu.lods.clear();
  gpu.boundsCenter = glm::vec3{0.0F, 0.0F, 0.0F};
  gpu.boundsRadius = 1.0F;
}

std::size_t CadModelRenderFeature::SelectLodLevel(
    const GpuMesh& mesh,
    const glm::mat4& mvp,
    const int viewportWidth,
    const int viewportHeight) {
  if (mesh.lods.size() <= 1) {
    return 0;
  }

  const CadLodPolicy& policy = LoadCadLodPolicy();
  const float radiusPx = ComputeScreenSpaceRadiusPixels(
      mesh.boundsCenter,
      mesh.boundsRadius,
      mvp,
      viewportWidth,
      viewportHeight);

  std::size_t selected = mesh.lods.size() - 1;
  float threshold = policy.lod0ScreenRadiusPx;
  for (std::size_t i = 0; i < mesh.lods.size(); ++i) {
    if (i == mesh.lods.size() - 1 || radiusPx >= threshold) {
      selected = i;
      break;
    }
    threshold *= policy.screenRadiusDecay;
  }

  if (policy.maxPreferredDrawIndices > 0) {
    while (selected + 1 < mesh.lods.size() &&
           static_cast<std::size_t>(std::max(mesh.lods[selected].indexCount, 0)) > policy.maxPreferredDrawIndices) {
      ++selected;
    }
  }
  return selected;
}

const CadModelRenderFeature::GpuMesh* CadModelRenderFeature::GetOrLoadMesh(const std::string& assetPath) {
  const auto cached = meshCache_.find(assetPath);
  if (cached != meshCache_.end()) {
    return &cached->second;
  }
  if (failedLoads_.find(assetPath) != failedLoads_.end()) {
    return nullptr;
  }
  if (geometryProvider_ == nullptr) {
    return nullptr;
  }

  const auto pending = pendingLoads_.find(assetPath);
  if (pending != pendingLoads_.end() &&
      pending->second.future.valid() &&
      pending->second.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    PreparedCpuMesh prepared = pending->second.future.get();
    pendingLoads_.erase(pending);
    if (prepared.lods.empty()) {
      failedLoads_.insert(assetPath);
      std::cerr << "CAD load failed: " << assetPath << "\n";
      return nullptr;
    }

    PendingGpuUpload upload;
    upload.prepared = std::move(prepared);
    upload.gpu.boundsCenter = upload.prepared.boundsCenter;
    upload.gpu.boundsRadius = upload.prepared.boundsRadius;
    pendingGpuUploads_[assetPath] = std::move(upload);
  }

  const auto uploadIt = pendingGpuUploads_.find(assetPath);
  if (uploadIt != pendingGpuUploads_.end()) {
    if (backend_ == nullptr) {
      return nullptr;
    }
    auto& upload = uploadIt->second;
    while (uploadsThisFrame_ < uploadBudgetPerFrame_ && upload.nextLod < upload.prepared.lods.size()) {
      const auto& cpuLod = upload.prepared.lods[upload.nextLod];
      if (cpuLod.vertices.empty()) {
        ++upload.nextLod;
        continue;
      }

      GpuMesh::LodGpu gpuLod;
      if (!UploadSingleLod(cpuLod, gpuLod, *backend_)) {
        failedLoads_.insert(assetPath);
        DestroyMesh(upload.gpu);
        pendingGpuUploads_.erase(assetPath);
        std::cerr << "CAD upload failed: " << assetPath << "\n";
        return nullptr;
      }
      upload.gpu.lods.push_back(std::move(gpuLod));
      ++upload.nextLod;
      ++uploadsThisFrame_;
      break;  // spread heavy uploads across frames
    }

    if (upload.nextLod < upload.prepared.lods.size()) {
      return nullptr;
    }

    auto [it, inserted] = meshCache_.emplace(assetPath, std::move(upload.gpu));
    pendingGpuUploads_.erase(assetPath);
    if (!inserted) {
      return &it->second;
    }

    std::ostringstream lodSummary;
    for (std::size_t i = 0; i < it->second.lods.size(); ++i) {
      if (i > 0) {
        lodSummary << ", ";
      }
      lodSummary << "L" << i << "(v=" << it->second.lods[i].vertexCount
                 << ",i=" << it->second.lods[i].indexCount << ")";
    }
    std::cerr << "CAD loaded: " << assetPath
              << " (lods=" << it->second.lods.size()
              << ", " << lodSummary.str()
              << ", centerXY=(" << it->second.boundsCenter.x << ", " << it->second.boundsCenter.y << ")"
              << ", radius=" << it->second.boundsRadius << ")\n";
    return &it->second;
  }

  if (pendingLoads_.find(assetPath) != pendingLoads_.end()) {
    return nullptr;
  }

  IGeometryProvider* provider = geometryProvider_;
  pendingLoads_.emplace(
      assetPath,
      PendingLoad{
          std::async(std::launch::async, [provider, assetPath]() {
            const CadLodPolicy& policy = LoadCadLodPolicy();
            PreparedCpuMeshBlob cachedBlob;
            if (TryLoadPreparedCpuMeshCache(assetPath, policy, cachedBlob)) {
              PreparedCpuMesh prepared;
              prepared.boundsCenter = cachedBlob.boundsCenter;
              prepared.boundsRadius = cachedBlob.boundsRadius;
              prepared.lods.reserve(cachedBlob.lods.size());
              for (auto& cachedLod : cachedBlob.lods) {
                PreparedCpuMesh::LodCpu lod;
                lod.vertices = std::move(cachedLod.vertices);
                lod.indices = std::move(cachedLod.indices);
                prepared.lods.push_back(std::move(lod));
              }
              std::cerr << "CAD prepared cache hit: " << assetPath << "\n";
              return prepared;
            }
            std::cerr << "CAD prepared cache miss: " << assetPath << "\n";

            PositionNormalMesh cpu;
            if (provider == nullptr || !provider->GetCadMesh(assetPath, cpu)) {
              return PreparedCpuMesh{};
            }
            PreparedCpuMesh prepared = PrepareCpuMesh(cpu);
            if (!prepared.lods.empty()) {
              PreparedCpuMeshBlob blob;
              blob.boundsCenter = prepared.boundsCenter;
              blob.boundsRadius = prepared.boundsRadius;
              blob.lods.reserve(prepared.lods.size());
              for (const auto& lod : prepared.lods) {
                PreparedCpuMeshBlob::LodBlob cachedLod;
                cachedLod.vertices = lod.vertices;
                cachedLod.indices = lod.indices;
                blob.lods.push_back(std::move(cachedLod));
              }
              StorePreparedCpuMeshCache(assetPath, policy, blob);
            }
            return prepared;
          })});
  std::cerr << "CAD loading started: " << assetPath << "\n";
  return nullptr;
}

}  // namespace repulsor3d
