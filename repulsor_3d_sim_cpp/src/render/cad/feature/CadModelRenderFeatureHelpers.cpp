#include "CadModelRenderFeatureInternals.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>

namespace repulsor3d::cadfeature {
using cad::CadLodPolicy;
using cad::CadShadowPolicy;
using cad::CadVisualPolicy;
using cad::GetEnvBool;
using cad::GetEnvFloat;
using cad::GetEnvInt;
using cad::LoadCadLodPolicy;
using cad::LoadCadShadowPolicy;
using cad::LoadCadVisualPolicy;

std::uint32_t FloatBits(const float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

constexpr std::uint64_t kFnv64OffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnv64Prime = 1099511628211ULL;
constexpr std::uint32_t kPreparedCacheMagic = 0x524C4F44U;  // "RLOD"
constexpr std::uint32_t kPreparedCacheVersion = 2U;

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

void FnvMixFloat(std::uint64_t& hash, const float value) {
  FnvMixU64(hash, static_cast<std::uint64_t>(FloatBits(value)));
}

std::string ToHexString(const std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << value;
  return out.str();
}

std::uint64_t BuildBroadphaseFingerprint(
    const std::vector<glm::vec3>& centers,
    const std::vector<float>& radii,
    const float cellSizeMeters) {
  std::uint64_t hash = kFnv64OffsetBasis;
  FnvMixU64(hash, static_cast<std::uint64_t>(centers.size()));
  FnvMixFloat(hash, cellSizeMeters);
  const std::size_t count = std::min(centers.size(), radii.size());
  for (std::size_t i = 0; i < count; ++i) {
    FnvMixFloat(hash, centers[i].x);
    FnvMixFloat(hash, centers[i].y);
    FnvMixFloat(hash, centers[i].z);
    FnvMixFloat(hash, radii[i]);
  }
  return hash;
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
  float roughnessHint = -1.0F;
  float metallicHint = -1.0F;
  if (!ReadScalar(in, roughnessHint) || !ReadScalar(in, metallicHint)) {
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
  prepared.roughnessHint = roughnessHint;
  prepared.metallicHint = metallicHint;
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
  if (!out || !WriteScalar(out, prepared.boundsRadius) ||
      !WriteScalar(out, prepared.roughnessHint) ||
      !WriteScalar(out, prepared.metallicHint)) {
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

  const int targetLastLodIndicesRaw = GetEnvInt("CAD_TARGET_MAX_LAST_LOD_INDICES", 1800000);
  const std::size_t targetLastLodIndices = static_cast<std::size_t>(std::max(0, targetLastLodIndicesRaw));
  const std::size_t hardMaxLodCount = std::max<std::size_t>(policy.maxLodCount, 6);
  while (!lods.empty() &&
         targetLastLodIndices > 0 &&
         lods.back().indices.size() > targetLastLodIndices &&
         lods.size() < hardMaxLodCount) {
    const IndexedMesh& previous = lods.back();
    if (previous.vertices.size() <= 50000) {
      break;
    }
    const std::size_t requested =
        static_cast<std::size_t>(std::floor(static_cast<float>(previous.vertices.size()) * 0.70F));
    const std::size_t targetVertices = std::max<std::size_t>(50000, requested);
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

IndexedMesh BuildShadowProxyMesh(const std::vector<IndexedMesh>& lods, const CadLodPolicy& policy) {
  IndexedMesh out;
  if (lods.empty()) {
    return out;
  }
  const IndexedMesh& source = lods.back();
  if (source.vertices.empty() || source.indices.empty()) {
    return out;
  }

  const int targetVerticesRaw = GetEnvInt("CAD_SHADOW_PROXY_MAX_VERTICES", 180000);
  const std::size_t targetVertices = static_cast<std::size_t>(std::clamp(targetVerticesRaw, 5000, 2000000));
  if (source.vertices.size() <= targetVertices) {
    return source;
  }
  const int proxyNormalBins = std::clamp(policy.normalBins / 2, 8, 20);
  return SimplifyByVertexClustering(source, targetVertices, proxyNormalBins, policy.preserveFlatShading);
}

namespace {

float VertexCacheScore(const int cachePosition, const int activeTriangleCount, const std::size_t cacheSize) {
  if (activeTriangleCount <= 0) {
    return -1.0F;
  }

  constexpr float kCacheDecayPower = 1.5F;
  constexpr float kLastTriScore = 0.75F;
  constexpr float kValenceBoostScale = 2.0F;
  constexpr float kValenceBoostPower = 0.5F;

  float score = 0.0F;
  if (cachePosition >= 0) {
    if (cachePosition < 3) {
      score = kLastTriScore;
    } else if (cacheSize > 3) {
      const float scaler = 1.0F / static_cast<float>(cacheSize - 3);
      const float normalized = 1.0F - (static_cast<float>(cachePosition - 3) * scaler);
      score = std::pow(std::clamp(normalized, 0.0F, 1.0F), kCacheDecayPower);
    }
  }
  score += kValenceBoostScale * std::pow(static_cast<float>(activeTriangleCount), -kValenceBoostPower);
  return score;
}

void SpatiallyBucketTriangles(
    std::vector<std::uint32_t>& triangleIndices,
    const std::vector<PositionNormalMesh::Vertex>& vertices,
    const std::size_t targetClusterIndices) {
  if (triangleIndices.size() < 3 || vertices.empty()) {
    return;
  }
  const std::size_t triCount = triangleIndices.size() / 3;
  if (triCount < 512) {
    return;
  }

  const std::size_t targetClusterTriCount = std::max<std::size_t>(128, (targetClusterIndices / 3));
  const std::size_t desiredClusters = std::max<std::size_t>(1, triCount / targetClusterTriCount);
  int binsX = std::clamp(static_cast<int>(std::ceil(std::sqrt(static_cast<float>(desiredClusters)))), 4, 96);
  int binsY = binsX;
  int binsZ = std::clamp(static_cast<int>(std::ceil(static_cast<float>(desiredClusters) / static_cast<float>(binsX * binsY))), 1, 8);
  if (binsX * binsY * binsZ > 16384) {
    binsZ = std::max(1, 16384 / std::max(1, binsX * binsY));
  }
  const int totalBins = std::max(1, binsX * binsY * binsZ);

  glm::vec3 minC{std::numeric_limits<float>::max()};
  glm::vec3 maxC{std::numeric_limits<float>::lowest()};
  for (std::size_t t = 0; t < triCount; ++t) {
    const std::uint32_t ia = triangleIndices[t * 3 + 0];
    const std::uint32_t ib = triangleIndices[t * 3 + 1];
    const std::uint32_t ic = triangleIndices[t * 3 + 2];
    if (ia >= vertices.size() || ib >= vertices.size() || ic >= vertices.size()) {
      continue;
    }
    const glm::vec3 c = (vertices[ia].position + vertices[ib].position + vertices[ic].position) * (1.0F / 3.0F);
    minC = glm::min(minC, c);
    maxC = glm::max(maxC, c);
  }
  glm::vec3 extent = maxC - minC;
  extent.x = std::max(extent.x, 1e-5F);
  extent.y = std::max(extent.y, 1e-5F);
  extent.z = std::max(extent.z, 1e-5F);

  std::vector<std::vector<std::uint32_t>> bins(static_cast<std::size_t>(totalBins));
  for (auto& bucket : bins) {
    bucket.reserve(std::max<std::size_t>(4, triCount / static_cast<std::size_t>(totalBins)));
  }

  const auto quantize = [](const float value, const int binsCount) -> int {
    const float clamped = std::clamp(value, 0.0F, 1.0F);
    return std::clamp(
        static_cast<int>(std::floor(clamped * static_cast<float>(binsCount))),
        0,
        std::max(0, binsCount - 1));
  };

  for (std::uint32_t t = 0; t < static_cast<std::uint32_t>(triCount); ++t) {
    const std::uint32_t ia = triangleIndices[static_cast<std::size_t>(t) * 3 + 0];
    const std::uint32_t ib = triangleIndices[static_cast<std::size_t>(t) * 3 + 1];
    const std::uint32_t ic = triangleIndices[static_cast<std::size_t>(t) * 3 + 2];
    if (ia >= vertices.size() || ib >= vertices.size() || ic >= vertices.size()) {
      continue;
    }
    const glm::vec3 c = (vertices[ia].position + vertices[ib].position + vertices[ic].position) * (1.0F / 3.0F);
    const glm::vec3 n = (c - minC) / extent;
    const int bx = quantize(n.x, binsX);
    const int by = quantize(n.y, binsY);
    const int bz = quantize(n.z, binsZ);
    const int bin = bx + binsX * (by + binsY * bz);
    bins[static_cast<std::size_t>(std::clamp(bin, 0, totalBins - 1))].push_back(t);
  }

  std::vector<std::uint32_t> reordered;
  reordered.reserve(triangleIndices.size());
  for (const auto& bucket : bins) {
    for (const std::uint32_t tri : bucket) {
      const std::size_t base = static_cast<std::size_t>(tri) * 3;
      if (base + 2 >= triangleIndices.size()) {
        continue;
      }
      reordered.push_back(triangleIndices[base + 0]);
      reordered.push_back(triangleIndices[base + 1]);
      reordered.push_back(triangleIndices[base + 2]);
    }
  }
  if (reordered.size() == triangleIndices.size()) {
    triangleIndices.swap(reordered);
  }
}

}  // namespace

void OptimizeIndexOrderForVertexCache(std::vector<std::uint32_t>& triangleIndices, const std::size_t cacheSize) {
  if (triangleIndices.size() < 6 || cacheSize < 3) {
    return;
  }
  const std::size_t triCount = triangleIndices.size() / 3;
  if (triCount == 0) {
    triangleIndices.clear();
    return;
  }
  triangleIndices.resize(triCount * 3);

  std::unordered_map<std::uint32_t, std::uint32_t> globalToLocal;
  globalToLocal.reserve(triangleIndices.size() / 2);
  std::vector<std::uint32_t> localToGlobal;
  localToGlobal.reserve(triangleIndices.size() / 2);
  std::vector<std::uint32_t> localIndices;
  localIndices.reserve(triangleIndices.size());

  for (const std::uint32_t globalIndex : triangleIndices) {
    auto it = globalToLocal.find(globalIndex);
    std::uint32_t localIndex = 0;
    if (it == globalToLocal.end()) {
      localIndex = static_cast<std::uint32_t>(localToGlobal.size());
      globalToLocal.emplace(globalIndex, localIndex);
      localToGlobal.push_back(globalIndex);
    } else {
      localIndex = it->second;
    }
    localIndices.push_back(localIndex);
  }

  struct VertexState {
    int activeTriangleCount = 0;
    int cachePosition = -1;
    float score = 0.0F;
    std::vector<std::uint32_t> adjacentTriangles;
  };
  struct TriangleState {
    std::array<std::uint32_t, 3> indices{};
    float score = 0.0F;
    bool emitted = false;
  };

  std::vector<VertexState> vertices(localToGlobal.size());
  std::vector<TriangleState> triangles(triCount);
  for (std::size_t t = 0; t < triCount; ++t) {
    for (std::size_t k = 0; k < 3; ++k) {
      const std::uint32_t local = localIndices[t * 3 + k];
      triangles[t].indices[k] = local;
      vertices[local].activeTriangleCount += 1;
      vertices[local].adjacentTriangles.push_back(static_cast<std::uint32_t>(t));
    }
  }
  for (auto& vertex : vertices) {
    vertex.score = VertexCacheScore(vertex.cachePosition, vertex.activeTriangleCount, cacheSize);
  }
  for (auto& triangle : triangles) {
    triangle.score =
        vertices[triangle.indices[0]].score +
        vertices[triangle.indices[1]].score +
        vertices[triangle.indices[2]].score;
  }

  std::vector<std::uint32_t> cache;
  cache.reserve(cacheSize + 3);
  std::vector<std::uint32_t> outputLocal;
  outputLocal.reserve(localIndices.size());
  std::vector<std::uint32_t> triStamps(triCount, 0U);
  std::vector<std::uint32_t> vertexStamps(vertices.size(), 0U);
  std::vector<std::uint32_t> candidates;
  std::vector<std::uint32_t> affectedVertices;
  std::uint32_t stampCounter = 1U;
  std::size_t fallbackTriangle = 0;

  auto nextStamp = [&]() -> std::uint32_t {
    ++stampCounter;
    if (stampCounter == 0U) {
      std::fill(triStamps.begin(), triStamps.end(), 0U);
      std::fill(vertexStamps.begin(), vertexStamps.end(), 0U);
      stampCounter = 1U;
    }
    return stampCounter;
  };

  std::size_t remainingTriangles = triCount;
  while (remainingTriangles > 0) {
    const std::uint32_t triStamp = nextStamp();
    candidates.clear();
    for (const std::uint32_t vertexId : cache) {
      for (const std::uint32_t triId : vertices[vertexId].adjacentTriangles) {
        if (triangles[triId].emitted || triStamps[triId] == triStamp) {
          continue;
        }
        triStamps[triId] = triStamp;
        candidates.push_back(triId);
      }
    }

    std::uint32_t bestTriangle = std::numeric_limits<std::uint32_t>::max();
    float bestScore = -std::numeric_limits<float>::infinity();
    for (const std::uint32_t triId : candidates) {
      if (triangles[triId].score > bestScore) {
        bestScore = triangles[triId].score;
        bestTriangle = triId;
      }
    }
    if (bestTriangle == std::numeric_limits<std::uint32_t>::max()) {
      while (fallbackTriangle < triCount && triangles[fallbackTriangle].emitted) {
        ++fallbackTriangle;
      }
      if (fallbackTriangle >= triCount) {
        break;
      }
      bestTriangle = static_cast<std::uint32_t>(fallbackTriangle);
    }

    auto& triangle = triangles[bestTriangle];
    triangle.emitted = true;
    --remainingTriangles;
    for (const std::uint32_t local : triangle.indices) {
      outputLocal.push_back(local);
    }

    const std::vector<std::uint32_t> oldCache = cache;
    for (const std::uint32_t local : triangle.indices) {
      const auto existing = std::find(cache.begin(), cache.end(), local);
      if (existing != cache.end()) {
        cache.erase(existing);
      }
      cache.insert(cache.begin(), local);
    }
    if (cache.size() > cacheSize) {
      cache.resize(cacheSize);
    }

    for (const std::uint32_t vertexId : oldCache) {
      vertices[vertexId].cachePosition = -1;
    }
    for (std::size_t i = 0; i < cache.size(); ++i) {
      vertices[cache[i]].cachePosition = static_cast<int>(i);
    }
    for (const std::uint32_t local : triangle.indices) {
      if (vertices[local].activeTriangleCount > 0) {
        vertices[local].activeTriangleCount -= 1;
      }
    }

    const std::uint32_t vertexStamp = nextStamp();
    affectedVertices.clear();
    auto addAffected = [&](const std::uint32_t vertexId) {
      if (vertexId >= vertexStamps.size() || vertexStamps[vertexId] == vertexStamp) {
        return;
      }
      vertexStamps[vertexId] = vertexStamp;
      affectedVertices.push_back(vertexId);
    };
    for (const std::uint32_t vertexId : oldCache) {
      addAffected(vertexId);
    }
    for (const std::uint32_t vertexId : cache) {
      addAffected(vertexId);
    }
    for (const std::uint32_t vertexId : triangle.indices) {
      addAffected(vertexId);
    }

    for (const std::uint32_t vertexId : affectedVertices) {
      auto& vertex = vertices[vertexId];
      vertex.score = VertexCacheScore(vertex.cachePosition, vertex.activeTriangleCount, cacheSize);
      for (const std::uint32_t triId : vertex.adjacentTriangles) {
        if (triangles[triId].emitted) {
          continue;
        }
        const auto& tri = triangles[triId];
        triangles[triId].score =
            vertices[tri.indices[0]].score +
            vertices[tri.indices[1]].score +
            vertices[tri.indices[2]].score;
      }
    }
  }

  if (outputLocal.size() != triangleIndices.size()) {
    return;
  }
  for (std::size_t i = 0; i < outputLocal.size(); ++i) {
    const std::uint32_t local = outputLocal[i];
    if (local >= localToGlobal.size()) {
      return;
    }
    triangleIndices[i] = localToGlobal[local];
  }
}

std::vector<MeshCluster> BuildMeshClusters(
    std::vector<std::uint32_t>& triangleIndices,
    const std::vector<PositionNormalMesh::Vertex>& vertices,
    const std::size_t targetClusterIndices,
    const std::size_t minClusterIndices,
    const bool optimizeVertexCache) {
  std::vector<MeshCluster> out;
  if (triangleIndices.size() < 3 || vertices.empty()) {
    return out;
  }

  const std::size_t triCount = triangleIndices.size() / 3;
  if (triCount == 0) {
    triangleIndices.clear();
    return out;
  }
  triangleIndices.resize(triCount * 3);

  const std::size_t targetIndices = std::max<std::size_t>(3 * 128, (targetClusterIndices / 3) * 3);
  const std::size_t minIndices = std::clamp<std::size_t>(((minClusterIndices / 3) * 3), 3 * 16, targetIndices);

  SpatiallyBucketTriangles(triangleIndices, vertices, targetIndices);

  std::size_t start = 0;
  while (start < triangleIndices.size()) {
    std::size_t count = std::min(targetIndices, triangleIndices.size() - start);
    count = (count / 3) * 3;
    if (count < 3) {
      break;
    }
    const std::size_t remaining = triangleIndices.size() - (start + count);
    if (remaining > 0 && remaining < minIndices) {
      count += remaining;
      count = (count / 3) * 3;
    }
    if (count < 3) {
      break;
    }

    if (optimizeVertexCache && count >= 3 * 32) {
      std::vector<std::uint32_t> clusterSlice(
          triangleIndices.begin() + static_cast<std::ptrdiff_t>(start),
          triangleIndices.begin() + static_cast<std::ptrdiff_t>(start + count));
      OptimizeIndexOrderForVertexCache(clusterSlice, 32);
      std::copy(
          clusterSlice.begin(),
          clusterSlice.end(),
          triangleIndices.begin() + static_cast<std::ptrdiff_t>(start));
    }

    glm::vec3 minP{std::numeric_limits<float>::max()};
    glm::vec3 maxP{std::numeric_limits<float>::lowest()};
    bool hasVertex = false;
    for (std::size_t i = start; i < start + count; ++i) {
      const std::uint32_t index = triangleIndices[i];
      if (index >= vertices.size()) {
        continue;
      }
      const glm::vec3 p = vertices[index].position;
      minP = glm::min(minP, p);
      maxP = glm::max(maxP, p);
      hasVertex = true;
    }
    if (!hasVertex) {
      start += count;
      continue;
    }

    const glm::vec3 center = 0.5F * (minP + maxP);
    float radiusSq = 0.0F;
    for (std::size_t i = start; i < start + count; ++i) {
      const std::uint32_t index = triangleIndices[i];
      if (index >= vertices.size()) {
        continue;
      }
      const glm::vec3 delta = vertices[index].position - center;
      radiusSq = std::max(radiusSq, glm::dot(delta, delta));
    }

    out.push_back(MeshCluster{
        .firstIndex = static_cast<std::uint32_t>(start),
        .indexCount = static_cast<std::uint32_t>(count),
        .boundsCenter = center,
        .boundsRadius = std::max(std::sqrt(radiusSq), 1e-3F),
    });
    start += count;
  }

  if (out.empty()) {
    out.push_back(MeshCluster{
        .firstIndex = 0U,
        .indexCount = static_cast<std::uint32_t>(triangleIndices.size()),
        .boundsCenter = glm::vec3{0.0F, 0.0F, 0.0F},
        .boundsRadius = 1.0F,
    });
  }
  return out;
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

}  // namespace repulsor3d::cadfeature
