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

struct PreparedCpuMeshBlob {
  struct LodBlob {
    std::vector<PositionNormalMesh::Vertex> vertices;
    std::vector<std::uint32_t> indices;
  };
  std::vector<LodBlob> lods;
  glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
  float boundsRadius = 1.0F;
  float roughnessHint = -1.0F;
  float metallicHint = -1.0F;
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
