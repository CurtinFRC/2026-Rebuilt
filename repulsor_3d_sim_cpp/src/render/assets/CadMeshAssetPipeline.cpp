#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {
namespace {

constexpr float kEpsilon = 1e-6F;
constexpr char kMeshCacheMagic[] = "R3DMESH1";

glm::vec3 NormalizeSafe(const glm::vec3& v) {
  const float len = glm::length(v);
  if (len <= kEpsilon) {
    return {0.0F, 0.0F, 1.0F};
  }
  return v / len;
}

glm::vec3 ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  return NormalizeSafe(glm::cross(b - a, c - a));
}

int ResolveObjIndex(const int indexValue, const int size) {
  if (indexValue > 0) {
    return indexValue - 1;
  }
  if (indexValue < 0) {
    return size + indexValue;
  }
  return -1;
}

bool ParseObjFaceVertex(const std::string& token, int& outPos, int& outNorm) {
  outPos = -1;
  outNorm = -1;

  const size_t firstSlash = token.find('/');
  if (firstSlash == std::string::npos) {
    try {
      outPos = std::stoi(token);
      return true;
    } catch (...) {
      return false;
    }
  }

  const size_t secondSlash = token.find('/', firstSlash + 1);
  try {
    outPos = std::stoi(token.substr(0, firstSlash));
  } catch (...) {
    return false;
  }

  if (secondSlash != std::string::npos && secondSlash + 1 < token.size()) {
    try {
      outNorm = std::stoi(token.substr(secondSlash + 1));
    } catch (...) {
      outNorm = -1;
    }
  }

  return true;
}

bool DecodeBase64(const std::string& encoded, std::vector<std::uint8_t>& out) {
  static const std::string kAlphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::array<int, 256> lookup{};
  lookup.fill(-1);
  for (int i = 0; i < static_cast<int>(kAlphabet.size()); ++i) {
    lookup[static_cast<unsigned char>(kAlphabet[static_cast<size_t>(i)])] = i;
  }

  out.clear();
  int value = 0;
  int bits = -8;
  for (const char c : encoded) {
    if (c == '=') {
      break;
    }
    const int decoded = lookup[static_cast<unsigned char>(c)];
    if (decoded < 0) {
      continue;
    }
    value = (value << 6) + decoded;
    bits += 6;
    if (bits >= 0) {
      out.push_back(static_cast<std::uint8_t>((value >> bits) & 0xFF));
      bits -= 8;
    }
  }

  return !out.empty();
}

bool LoadBufferUri(const std::filesystem::path& modelPath, const std::string& uri, std::vector<std::uint8_t>& out) {
  out.clear();

  if (uri.rfind("data:", 0) == 0) {
    const size_t comma = uri.find(',');
    if (comma == std::string::npos) {
      return false;
    }
    const std::string meta = uri.substr(0, comma);
    const std::string payload = uri.substr(comma + 1);
    if (meta.find(";base64") == std::string::npos) {
      return false;
    }
    return DecodeBase64(payload, out);
  }

  const std::filesystem::path filePath = modelPath.parent_path() / std::filesystem::path(uri);
  std::ifstream in(filePath, std::ios::binary);
  if (!in) {
    return false;
  }
  in.seekg(0, std::ios::end);
  const std::streamoff size = in.tellg();
  if (size <= 0) {
    return false;
  }
  in.seekg(0, std::ios::beg);
  out.resize(static_cast<size_t>(size));
  in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return in.good();
}

}  // namespace

bool StlCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  if (ParseBinaryStl(resolvedAssetPath, outMesh)) {
    return true;
  }
  return ParseAsciiStl(resolvedAssetPath, outMesh);
}

bool StlCadMeshImporter::ParseBinaryStl(const std::string& filePath, PositionNormalMesh& outMesh) {
  std::ifstream in(filePath, std::ios::binary);
  if (!in) {
    return false;
  }

  in.seekg(0, std::ios::end);
  const std::streamoff fileSize = in.tellg();
  if (fileSize < 84) {
    return false;
  }
  in.seekg(0, std::ios::beg);

  std::array<char, 80> header{};
  in.read(header.data(), static_cast<std::streamsize>(header.size()));

  std::uint32_t triCount = 0;
  in.read(reinterpret_cast<char*>(&triCount), sizeof(std::uint32_t));
  if (!in) {
    return false;
  }

  const std::uint64_t expected = 84ULL + static_cast<std::uint64_t>(triCount) * 50ULL;
  if (expected != static_cast<std::uint64_t>(fileSize)) {
    return false;
  }

  outMesh.vertices.clear();
  outMesh.vertices.reserve(static_cast<size_t>(triCount) * 3);

  for (std::uint32_t i = 0; i < triCount; ++i) {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    glm::vec3 p[3];

    in.read(reinterpret_cast<char*>(&nx), sizeof(float));
    in.read(reinterpret_cast<char*>(&ny), sizeof(float));
    in.read(reinterpret_cast<char*>(&nz), sizeof(float));

    for (auto& vertex : p) {
      in.read(reinterpret_cast<char*>(&vertex.x), sizeof(float));
      in.read(reinterpret_cast<char*>(&vertex.y), sizeof(float));
      in.read(reinterpret_cast<char*>(&vertex.z), sizeof(float));
    }

    std::uint16_t attr = 0;
    in.read(reinterpret_cast<char*>(&attr), sizeof(std::uint16_t));
    (void)attr;
    if (!in) {
      return false;
    }

    const glm::vec3 rawNormal{nx, ny, nz};
    glm::vec3 normal = NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= kEpsilon) {
      normal = ComputeFallbackNormal(p[0], p[1], p[2]);
    }

    outMesh.vertices.push_back({p[0], normal});
    outMesh.vertices.push_back({p[1], normal});
    outMesh.vertices.push_back({p[2], normal});
  }

  return !outMesh.vertices.empty();
}

bool StlCadMeshImporter::ParseAsciiStl(const std::string& filePath, PositionNormalMesh& outMesh) {
  std::ifstream in(filePath);
  if (!in) {
    return false;
  }

  outMesh.vertices.clear();
  std::string token;
  while (in >> token) {
    if (token != "facet") {
      continue;
    }
    if (!(in >> token) || token != "normal") {
      return false;
    }

    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    if (!(in >> nx >> ny >> nz)) {
      return false;
    }

    if (!(in >> token) || token != "outer") {
      return false;
    }
    if (!(in >> token) || token != "loop") {
      return false;
    }

    glm::vec3 p[3];
    for (auto& vertex : p) {
      if (!(in >> token) || token != "vertex") {
        return false;
      }
      if (!(in >> vertex.x >> vertex.y >> vertex.z)) {
        return false;
      }
    }

    if (!(in >> token) || token != "endloop") {
      return false;
    }
    if (!(in >> token) || token != "endfacet") {
      return false;
    }

    const glm::vec3 rawNormal{nx, ny, nz};
    glm::vec3 normal = NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= kEpsilon) {
      normal = ComputeFallbackNormal(p[0], p[1], p[2]);
    }

    outMesh.vertices.push_back({p[0], normal});
    outMesh.vertices.push_back({p[1], normal});
    outMesh.vertices.push_back({p[2], normal});
  }

  return !outMesh.vertices.empty();
}

bool ObjCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  std::ifstream in(resolvedAssetPath);
  if (!in) {
    return false;
  }

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  outMesh.vertices.clear();

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream iss(line);
    std::string kind;
    iss >> kind;
    if (kind == "v") {
      glm::vec3 p{};
      if (iss >> p.x >> p.y >> p.z) {
        positions.push_back(p);
      }
    } else if (kind == "vn") {
      glm::vec3 n{};
      if (iss >> n.x >> n.y >> n.z) {
        normals.push_back(NormalizeSafe(n));
      }
    } else if (kind == "f") {
      std::vector<int> posIndices;
      std::vector<int> normIndices;
      std::string token;
      while (iss >> token) {
        int pIdx = -1;
        int nIdx = -1;
        if (!ParseObjFaceVertex(token, pIdx, nIdx)) {
          continue;
        }
        posIndices.push_back(ResolveObjIndex(pIdx, static_cast<int>(positions.size())));
        normIndices.push_back(ResolveObjIndex(nIdx, static_cast<int>(normals.size())));
      }

      if (posIndices.size() < 3) {
        continue;
      }

      for (size_t i = 1; i + 1 < posIndices.size(); ++i) {
        const int i0 = posIndices[0];
        const int i1 = posIndices[i];
        const int i2 = posIndices[i + 1];
        if (i0 < 0 || i1 < 0 || i2 < 0 ||
            i0 >= static_cast<int>(positions.size()) ||
            i1 >= static_cast<int>(positions.size()) ||
            i2 >= static_cast<int>(positions.size())) {
          continue;
        }

        glm::vec3 p0 = positions[static_cast<size_t>(i0)];
        glm::vec3 p1 = positions[static_cast<size_t>(i1)];
        glm::vec3 p2 = positions[static_cast<size_t>(i2)];

        glm::vec3 n0 = ComputeFallbackNormal(p0, p1, p2);
        glm::vec3 n1 = n0;
        glm::vec3 n2 = n0;

        const int nIdx0 = normIndices[0];
        const int nIdx1 = normIndices[i];
        const int nIdx2 = normIndices[i + 1];
        if (nIdx0 >= 0 && nIdx1 >= 0 && nIdx2 >= 0 &&
            nIdx0 < static_cast<int>(normals.size()) &&
            nIdx1 < static_cast<int>(normals.size()) &&
            nIdx2 < static_cast<int>(normals.size())) {
          n0 = normals[static_cast<size_t>(nIdx0)];
          n1 = normals[static_cast<size_t>(nIdx1)];
          n2 = normals[static_cast<size_t>(nIdx2)];
        }

        outMesh.vertices.push_back({p0, n0});
        outMesh.vertices.push_back({p1, n1});
        outMesh.vertices.push_back({p2, n2});
      }
    }
  }

  return !outMesh.vertices.empty();
}

bool GltfCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  std::ifstream in(resolvedAssetPath);
  if (!in) {
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    return false;
  }

  if (!root.contains("buffers") || !root["buffers"].is_array() ||
      !root.contains("bufferViews") || !root["bufferViews"].is_array() ||
      !root.contains("accessors") || !root["accessors"].is_array() ||
      !root.contains("meshes") || !root["meshes"].is_array() ||
      root["meshes"].empty()) {
    return false;
  }

  std::vector<std::vector<std::uint8_t>> buffers;
  buffers.resize(root["buffers"].size());

  const std::filesystem::path modelPath(resolvedAssetPath);
  for (size_t i = 0; i < root["buffers"].size(); ++i) {
    const auto& bufferNode = root["buffers"][i];
    if (!bufferNode.contains("uri") || !bufferNode["uri"].is_string()) {
      return false;
    }
    if (!LoadBufferUri(modelPath, bufferNode["uri"].get<std::string>(), buffers[i])) {
      return false;
    }
  }

  const auto& mesh = root["meshes"][0];
  if (!mesh.contains("primitives") || !mesh["primitives"].is_array() || mesh["primitives"].empty()) {
    return false;
  }
  const auto& primitive = mesh["primitives"][0];
  if (primitive.contains("mode") && primitive["mode"].is_number_integer() && primitive["mode"].get<int>() != 4) {
    return false;
  }
  if (!primitive.contains("attributes") || !primitive["attributes"].is_object()) {
    return false;
  }
  const auto& attrs = primitive["attributes"];
  if (!attrs.contains("POSITION") || !attrs["POSITION"].is_number_integer()) {
    return false;
  }

  const int positionAccessorIndex = attrs["POSITION"].get<int>();
  const int normalAccessorIndex =
      (attrs.contains("NORMAL") && attrs["NORMAL"].is_number_integer()) ? attrs["NORMAL"].get<int>() : -1;
  const int indexAccessorIndex =
      (primitive.contains("indices") && primitive["indices"].is_number_integer()) ? primitive["indices"].get<int>() : -1;

  auto readVec3Accessor = [&](const int accessorIndex, std::vector<glm::vec3>& out) -> bool {
    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(root["accessors"].size())) {
      return false;
    }
    const auto& accessor = root["accessors"][static_cast<size_t>(accessorIndex)];
    if (!accessor.contains("bufferView") || !accessor["bufferView"].is_number_integer()) {
      return false;
    }
    const int bufferViewIndex = accessor["bufferView"].get<int>();
    if (bufferViewIndex < 0 || bufferViewIndex >= static_cast<int>(root["bufferViews"].size())) {
      return false;
    }
    const auto& view = root["bufferViews"][static_cast<size_t>(bufferViewIndex)];
    if (!view.contains("buffer") || !view["buffer"].is_number_integer()) {
      return false;
    }

    const int bufferIndex = view["buffer"].get<int>();
    if (bufferIndex < 0 || bufferIndex >= static_cast<int>(buffers.size())) {
      return false;
    }

    if (!accessor.contains("count") || !accessor["count"].is_number_integer()) {
      return false;
    }
    if (!accessor.contains("componentType") || accessor["componentType"].get<int>() != 5126) {
      return false;
    }
    if (!accessor.contains("type") || accessor["type"].get<std::string>() != "VEC3") {
      return false;
    }

    const size_t count = static_cast<size_t>(accessor["count"].get<int>());
    const size_t viewOffset =
        view.contains("byteOffset") && view["byteOffset"].is_number_integer() ? static_cast<size_t>(view["byteOffset"].get<int>()) : 0U;
    const size_t accessorOffset =
        accessor.contains("byteOffset") && accessor["byteOffset"].is_number_integer()
            ? static_cast<size_t>(accessor["byteOffset"].get<int>())
            : 0U;
    const size_t stride =
        view.contains("byteStride") && view["byteStride"].is_number_integer() ? static_cast<size_t>(view["byteStride"].get<int>()) : 12U;
    const size_t base = viewOffset + accessorOffset;

    const auto& bufferData = buffers[static_cast<size_t>(bufferIndex)];
    if (base + count * stride > bufferData.size()) {
      return false;
    }

    out.resize(count);
    for (size_t i = 0; i < count; ++i) {
      const std::uint8_t* ptr = bufferData.data() + base + i * stride;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, ptr + 0, sizeof(float));
      std::memcpy(&y, ptr + 4, sizeof(float));
      std::memcpy(&z, ptr + 8, sizeof(float));
      out[i] = glm::vec3{x, y, z};
    }
    return true;
  };

  auto readIndexAccessor = [&](const int accessorIndex, std::vector<std::uint32_t>& out) -> bool {
    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(root["accessors"].size())) {
      return false;
    }
    const auto& accessor = root["accessors"][static_cast<size_t>(accessorIndex)];
    if (!accessor.contains("bufferView") || !accessor["bufferView"].is_number_integer()) {
      return false;
    }
    const int bufferViewIndex = accessor["bufferView"].get<int>();
    if (bufferViewIndex < 0 || bufferViewIndex >= static_cast<int>(root["bufferViews"].size())) {
      return false;
    }
    const auto& view = root["bufferViews"][static_cast<size_t>(bufferViewIndex)];
    if (!view.contains("buffer") || !view["buffer"].is_number_integer()) {
      return false;
    }
    const int bufferIndex = view["buffer"].get<int>();
    if (bufferIndex < 0 || bufferIndex >= static_cast<int>(buffers.size())) {
      return false;
    }
    if (!accessor.contains("count") || !accessor["count"].is_number_integer()) {
      return false;
    }
    const size_t count = static_cast<size_t>(accessor["count"].get<int>());

    const int componentType = accessor.contains("componentType") && accessor["componentType"].is_number_integer()
                                  ? accessor["componentType"].get<int>()
                                  : 0;
    size_t componentSize = 0;
    if (componentType == 5121) {
      componentSize = 1;
    } else if (componentType == 5123) {
      componentSize = 2;
    } else if (componentType == 5125) {
      componentSize = 4;
    } else {
      return false;
    }

    const size_t viewOffset =
        view.contains("byteOffset") && view["byteOffset"].is_number_integer() ? static_cast<size_t>(view["byteOffset"].get<int>()) : 0U;
    const size_t accessorOffset =
        accessor.contains("byteOffset") && accessor["byteOffset"].is_number_integer()
            ? static_cast<size_t>(accessor["byteOffset"].get<int>())
            : 0U;
    const size_t stride =
        view.contains("byteStride") && view["byteStride"].is_number_integer()
            ? static_cast<size_t>(view["byteStride"].get<int>())
            : componentSize;
    const size_t base = viewOffset + accessorOffset;

    const auto& bufferData = buffers[static_cast<size_t>(bufferIndex)];
    if (base + count * stride > bufferData.size()) {
      return false;
    }

    out.resize(count);
    for (size_t i = 0; i < count; ++i) {
      const std::uint8_t* ptr = bufferData.data() + base + i * stride;
      std::uint32_t value = 0;
      if (componentType == 5121) {
        value = *ptr;
      } else if (componentType == 5123) {
        std::uint16_t v = 0;
        std::memcpy(&v, ptr, sizeof(std::uint16_t));
        value = v;
      } else if (componentType == 5125) {
        std::memcpy(&value, ptr, sizeof(std::uint32_t));
      }
      out[i] = value;
    }
    return true;
  };

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  if (!readVec3Accessor(positionAccessorIndex, positions)) {
    return false;
  }
  if (normalAccessorIndex >= 0) {
    if (!readVec3Accessor(normalAccessorIndex, normals)) {
      normals.clear();
    }
  }

  std::vector<std::uint32_t> indices;
  if (indexAccessorIndex >= 0) {
    if (!readIndexAccessor(indexAccessorIndex, indices)) {
      return false;
    }
  } else {
    indices.resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
      indices[i] = static_cast<std::uint32_t>(i);
    }
  }

  if (indices.size() < 3) {
    return false;
  }

  outMesh.vertices.clear();
  outMesh.vertices.reserve(indices.size());

  for (size_t i = 0; i + 2 < indices.size(); i += 3) {
    const std::uint32_t ia = indices[i];
    const std::uint32_t ib = indices[i + 1];
    const std::uint32_t ic = indices[i + 2];
    if (ia >= positions.size() || ib >= positions.size() || ic >= positions.size()) {
      continue;
    }

    const glm::vec3 pa = positions[ia];
    const glm::vec3 pb = positions[ib];
    const glm::vec3 pc = positions[ic];

    glm::vec3 na = ComputeFallbackNormal(pa, pb, pc);
    glm::vec3 nb = na;
    glm::vec3 nc = na;
    if (normals.size() == positions.size()) {
      na = NormalizeSafe(normals[ia]);
      nb = NormalizeSafe(normals[ib]);
      nc = NormalizeSafe(normals[ic]);
    }

    outMesh.vertices.push_back({pa, na});
    outMesh.vertices.push_back({pb, nb});
    outMesh.vertices.push_back({pc, nc});
  }

  return !outMesh.vertices.empty();
}

bool MultiFormatCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  const std::filesystem::path path(resolvedAssetPath);
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (ext == ".stl") {
    return stlImporter_.Import(resolvedAssetPath, outMesh);
  }
  if (ext == ".obj") {
    return objImporter_.Import(resolvedAssetPath, outMesh);
  }
  if (ext == ".gltf") {
    return gltfImporter_.Import(resolvedAssetPath, outMesh);
  }

  return stlImporter_.Import(resolvedAssetPath, outMesh) ||
         objImporter_.Import(resolvedAssetPath, outMesh) ||
         gltfImporter_.Import(resolvedAssetPath, outMesh);
}

void PassThroughCadMeshCooker::Cook(PositionNormalMesh& /*mesh*/) {}

bool InMemoryCadMeshCache::TryLoad(const std::string& key, PositionNormalMesh& outMesh) const {
  const auto it = cache_.find(key);
  if (it == cache_.end()) {
    return false;
  }
  outMesh = it->second;
  return true;
}

void InMemoryCadMeshCache::Store(const std::string& key, const PositionNormalMesh& mesh) {
  cache_[key] = mesh;
}

FileCadMeshCache::FileCadMeshCache(std::string rootDirectory) : rootDirectory_(std::move(rootDirectory)) {}

std::string FileCadMeshCache::KeyToPath(const std::string& key) const {
  const std::uint64_t hash = static_cast<std::uint64_t>(std::hash<std::string>{}(key));
  std::ostringstream filename;
  filename << std::hex << hash << ".bin";
  return (std::filesystem::path(rootDirectory_) / filename.str()).string();
}

bool FileCadMeshCache::TryLoad(const std::string& key, PositionNormalMesh& outMesh) const {
  const std::string path = KeyToPath(key);
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }

  char magic[sizeof(kMeshCacheMagic)]{};
  in.read(magic, static_cast<std::streamsize>(sizeof(kMeshCacheMagic) - 1));
  if (!in || std::strncmp(magic, kMeshCacheMagic, sizeof(kMeshCacheMagic) - 1) != 0) {
    return false;
  }

  std::uint32_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(std::uint32_t));
  if (!in) {
    return false;
  }

  outMesh.vertices.clear();
  outMesh.vertices.resize(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    auto& v = outMesh.vertices[i];
    in.read(reinterpret_cast<char*>(&v.position.x), sizeof(float));
    in.read(reinterpret_cast<char*>(&v.position.y), sizeof(float));
    in.read(reinterpret_cast<char*>(&v.position.z), sizeof(float));
    in.read(reinterpret_cast<char*>(&v.normal.x), sizeof(float));
    in.read(reinterpret_cast<char*>(&v.normal.y), sizeof(float));
    in.read(reinterpret_cast<char*>(&v.normal.z), sizeof(float));
    if (!in) {
      outMesh.vertices.clear();
      return false;
    }
  }
  return !outMesh.vertices.empty();
}

void FileCadMeshCache::Store(const std::string& key, const PositionNormalMesh& mesh) {
  std::error_code ec;
  std::filesystem::create_directories(rootDirectory_, ec);
  (void)ec;

  const std::string path = KeyToPath(key);
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return;
  }

  out.write(kMeshCacheMagic, static_cast<std::streamsize>(sizeof(kMeshCacheMagic) - 1));
  const std::uint32_t count = static_cast<std::uint32_t>(mesh.vertices.size());
  out.write(reinterpret_cast<const char*>(&count), sizeof(std::uint32_t));
  for (const auto& v : mesh.vertices) {
    out.write(reinterpret_cast<const char*>(&v.position.x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.position.y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.position.z), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.z), sizeof(float));
  }
}

CompositeCadMeshCache::CompositeCadMeshCache(std::unique_ptr<ICadMeshCache> first, std::unique_ptr<ICadMeshCache> second)
    : first_(std::move(first)), second_(std::move(second)) {}

bool CompositeCadMeshCache::TryLoad(const std::string& key, PositionNormalMesh& outMesh) const {
  if (first_ != nullptr && first_->TryLoad(key, outMesh)) {
    return true;
  }
  if (second_ != nullptr && second_->TryLoad(key, outMesh)) {
    if (first_ != nullptr) {
      first_->Store(key, outMesh);
    }
    return true;
  }
  return false;
}

void CompositeCadMeshCache::Store(const std::string& key, const PositionNormalMesh& mesh) {
  if (first_ != nullptr) {
    first_->Store(key, mesh);
  }
  if (second_ != nullptr) {
    second_->Store(key, mesh);
  }
}

std::unique_ptr<ICadMeshImporter> CreateDefaultCadMeshImporter() {
  return std::make_unique<MultiFormatCadMeshImporter>();
}

std::unique_ptr<ICadMeshCache> CreateDefaultCadMeshCache() {
  return std::make_unique<CompositeCadMeshCache>(
      std::make_unique<InMemoryCadMeshCache>(),
      std::make_unique<FileCadMeshCache>(".repulsor_cache/cad_mesh"));
}

CadMeshAssetPipeline::CadMeshAssetPipeline(
    const ISceneAssetResolver& resolver,
    std::unique_ptr<ICadMeshImporter> importer,
    std::unique_ptr<ICadMeshCooker> cooker,
    std::unique_ptr<ICadMeshCache> cache)
    : resolver_(resolver), importer_(std::move(importer)), cooker_(std::move(cooker)), cache_(std::move(cache)) {}

bool CadMeshAssetPipeline::Load(const std::string& assetPath, PositionNormalMesh& outMesh) {
  if (importer_ == nullptr || cooker_ == nullptr || cache_ == nullptr) {
    return false;
  }

  const std::string resolved = resolver_.ResolveFilePath(assetPath);
  if (resolved.empty()) {
    return false;
  }

  if (cache_->TryLoad(resolved, outMesh)) {
    return true;
  }

  PositionNormalMesh mesh;
  if (!importer_->Import(resolved, mesh)) {
    return false;
  }

  cooker_->Cook(mesh);
  cache_->Store(resolved, mesh);
  outMesh = std::move(mesh);
  return true;
}

}  // namespace repulsor3d

