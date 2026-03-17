#include "CadMeshImportHelpers.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec4.hpp>

namespace repulsor3d::cad_import {
namespace {

constexpr float kEpsilon = 1e-6F;
constexpr std::uint32_t kGlbMagic = 0x46546C67U;
constexpr std::uint32_t kGlbJsonChunkType = 0x4E4F534AU;
constexpr std::uint32_t kGlbBinChunkType = 0x004E4942U;

glm::vec3 ParseJsonVec3(const nlohmann::json& node, const glm::vec3& fallback) {
  if (!node.is_array() || node.size() < 3) {
    return fallback;
  }
  const float x = node[0].is_number() ? node[0].get<float>() : fallback.x;
  const float y = node[1].is_number() ? node[1].get<float>() : fallback.y;
  const float z = node[2].is_number() ? node[2].get<float>() : fallback.z;
  return glm::vec3{x, y, z};
}

glm::vec4 ParseJsonVec4(const nlohmann::json& node, const glm::vec4& fallback) {
  if (!node.is_array() || node.size() < 4) {
    return fallback;
  }
  const float x = node[0].is_number() ? node[0].get<float>() : fallback.x;
  const float y = node[1].is_number() ? node[1].get<float>() : fallback.y;
  const float z = node[2].is_number() ? node[2].get<float>() : fallback.z;
  const float w = node[3].is_number() ? node[3].get<float>() : fallback.w;
  return glm::vec4{x, y, z, w};
}

bool ReadAccessorColorVec4(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const int accessorIndex,
    std::vector<glm::vec4>& out) {
  if (!root.contains("accessors") || !root["accessors"].is_array() ||
      accessorIndex < 0 || accessorIndex >= static_cast<int>(root["accessors"].size())) {
    return false;
  }

  const auto& accessor = root["accessors"][static_cast<size_t>(accessorIndex)];
  if (!accessor.contains("bufferView") || !accessor["bufferView"].is_number_integer()) {
    return false;
  }
  const int bufferViewIndex = accessor["bufferView"].get<int>();
  if (!root.contains("bufferViews") || !root["bufferViews"].is_array() ||
      bufferViewIndex < 0 || bufferViewIndex >= static_cast<int>(root["bufferViews"].size())) {
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
  if (!accessor.contains("type") || !accessor["type"].is_string()) {
    return false;
  }
  const std::string type = accessor["type"].get<std::string>();
  const bool isVec3 = type == "VEC3";
  const bool isVec4 = type == "VEC4";
  if (!isVec3 && !isVec4) {
    return false;
  }

  const size_t count = static_cast<size_t>(accessor["count"].get<int>());
  const size_t viewOffset =
      view.contains("byteOffset") && view["byteOffset"].is_number_integer()
          ? static_cast<size_t>(view["byteOffset"].get<int>())
          : 0U;
  const size_t accessorOffset =
      accessor.contains("byteOffset") && accessor["byteOffset"].is_number_integer()
          ? static_cast<size_t>(accessor["byteOffset"].get<int>())
          : 0U;
  const size_t defaultStride = isVec3 ? 12U : 16U;
  const size_t stride =
      view.contains("byteStride") && view["byteStride"].is_number_integer()
          ? static_cast<size_t>(view["byteStride"].get<int>())
          : defaultStride;
  const size_t base = viewOffset + accessorOffset;
  if (stride < defaultStride) {
    return false;
  }

  const auto& bufferData = buffers[static_cast<size_t>(bufferIndex)];
  if (base + count * stride > bufferData.size()) {
    return false;
  }

  out.resize(count);
  for (size_t i = 0; i < count; ++i) {
    const std::uint8_t* ptr = bufferData.data() + base + i * stride;
    float r = 1.0F;
    float g = 1.0F;
    float b = 1.0F;
    float a = 1.0F;
    std::memcpy(&r, ptr + 0, sizeof(float));
    std::memcpy(&g, ptr + 4, sizeof(float));
    std::memcpy(&b, ptr + 8, sizeof(float));
    if (isVec4) {
      std::memcpy(&a, ptr + 12, sizeof(float));
    }
    out[i] = glm::vec4{
        std::clamp(r, 0.0F, 1.0F),
        std::clamp(g, 0.0F, 1.0F),
        std::clamp(b, 0.0F, 1.0F),
        std::clamp(a, 0.0F, 1.0F)};
  }
  return true;
}

glm::vec4 ResolvePrimitiveBaseColor(const nlohmann::json& root, const nlohmann::json& primitive) {
  if (!primitive.contains("material") || !primitive["material"].is_number_integer()) {
    return glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
  }
  if (!root.contains("materials") || !root["materials"].is_array()) {
    return glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
  }

  const int materialIndex = primitive["material"].get<int>();
  if (materialIndex < 0 || materialIndex >= static_cast<int>(root["materials"].size())) {
    return glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
  }

  const auto& material = root["materials"][static_cast<size_t>(materialIndex)];
  if (!material.is_object() || !material.contains("pbrMetallicRoughness")) {
    return glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
  }
  const auto& pbr = material["pbrMetallicRoughness"];
  if (!pbr.is_object() || !pbr.contains("baseColorFactor")) {
    return glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
  }
  return ParseJsonVec4(pbr["baseColorFactor"], glm::vec4{1.0F, 1.0F, 1.0F, 1.0F});
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

bool LoadBufferUriImpl(const std::filesystem::path& modelPath, const std::string& uri, std::vector<std::uint8_t>& out) {
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

bool ReadEntireFileBinary(const std::string& path, std::vector<std::uint8_t>& out) {
  std::ifstream in(path, std::ios::binary);
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

bool ParseGlb(const std::string& path, nlohmann::json& outRoot, std::vector<std::uint8_t>& outBinChunk) {
  std::vector<std::uint8_t> bytes;
  if (!ReadEntireFileBinary(path, bytes)) {
    return false;
  }
  if (bytes.size() < 12) {
    return false;
  }

  auto readU32 = [&](const size_t offset) -> std::uint32_t {
    std::uint32_t value = 0;
    std::memcpy(&value, bytes.data() + offset, sizeof(std::uint32_t));
    return value;
  };

  if (readU32(0) != kGlbMagic || readU32(4) != 2U) {
    return false;
  }

  const std::uint32_t declaredLength = readU32(8);
  if (declaredLength < 12U || declaredLength > bytes.size()) {
    return false;
  }

  std::vector<std::uint8_t> jsonChunk;
  outBinChunk.clear();

  size_t cursor = 12;
  while (cursor + 8 <= declaredLength) {
    const std::uint32_t chunkLength = readU32(cursor);
    const std::uint32_t chunkType = readU32(cursor + 4);
    cursor += 8;
    if (cursor + chunkLength > declaredLength) {
      return false;
    }

    if (chunkType == kGlbJsonChunkType) {
      jsonChunk.assign(
          bytes.begin() + static_cast<std::ptrdiff_t>(cursor),
          bytes.begin() + static_cast<std::ptrdiff_t>(cursor + chunkLength));
    } else if (chunkType == kGlbBinChunkType) {
      outBinChunk.assign(
          bytes.begin() + static_cast<std::ptrdiff_t>(cursor),
          bytes.begin() + static_cast<std::ptrdiff_t>(cursor + chunkLength));
    }

    cursor += chunkLength;
  }

  if (jsonChunk.empty()) {
    return false;
  }

  std::string jsonText(reinterpret_cast<const char*>(jsonChunk.data()), jsonChunk.size());
  while (!jsonText.empty() && jsonText.back() == '\0') {
    jsonText.pop_back();
  }
  try {
    outRoot = nlohmann::json::parse(jsonText);
  } catch (...) {
    return false;
  }
  return true;
}

}  // namespace

bool LoadBufferUri(const std::filesystem::path& modelPath, const std::string& uri, std::vector<std::uint8_t>& out) {
  return LoadBufferUriImpl(modelPath, uri, out);
}

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

bool LoadGltfRoot(const std::string& resolvedAssetPath, nlohmann::json& outRoot, std::vector<std::uint8_t>& outGlbBinChunk) {
  std::filesystem::path path(resolvedAssetPath);
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  outGlbBinChunk.clear();
  if (ext == ".glb") {
    return ParseGlb(resolvedAssetPath, outRoot, outGlbBinChunk);
  }

  std::ifstream in(resolvedAssetPath);
  if (!in) {
    return false;
  }
  try {
    in >> outRoot;
  } catch (...) {
    return false;
  }
  return true;
}

bool ReadAccessorVec3(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const int accessorIndex,
    std::vector<glm::vec3>& out) {
  if (!root.contains("accessors") || !root["accessors"].is_array() ||
      accessorIndex < 0 || accessorIndex >= static_cast<int>(root["accessors"].size())) {
    return false;
  }

  const auto& accessor = root["accessors"][static_cast<size_t>(accessorIndex)];
  if (!accessor.contains("bufferView") || !accessor["bufferView"].is_number_integer()) {
    return false;
  }
  const int bufferViewIndex = accessor["bufferView"].get<int>();
  if (!root.contains("bufferViews") || !root["bufferViews"].is_array() ||
      bufferViewIndex < 0 || bufferViewIndex >= static_cast<int>(root["bufferViews"].size())) {
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
      view.contains("byteOffset") && view["byteOffset"].is_number_integer()
          ? static_cast<size_t>(view["byteOffset"].get<int>())
          : 0U;
  const size_t accessorOffset =
      accessor.contains("byteOffset") && accessor["byteOffset"].is_number_integer()
          ? static_cast<size_t>(accessor["byteOffset"].get<int>())
          : 0U;
  const size_t stride =
      view.contains("byteStride") && view["byteStride"].is_number_integer()
          ? static_cast<size_t>(view["byteStride"].get<int>())
          : 12U;
  const size_t base = viewOffset + accessorOffset;
  if (stride < 12U) {
    return false;
  }

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
}

bool ReadAccessorIndices(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const int accessorIndex,
    std::vector<std::uint32_t>& out) {
  if (!root.contains("accessors") || !root["accessors"].is_array() ||
      accessorIndex < 0 || accessorIndex >= static_cast<int>(root["accessors"].size())) {
    return false;
  }

  const auto& accessor = root["accessors"][static_cast<size_t>(accessorIndex)];
  if (!accessor.contains("bufferView") || !accessor["bufferView"].is_number_integer()) {
    return false;
  }
  const int bufferViewIndex = accessor["bufferView"].get<int>();
  if (!root.contains("bufferViews") || !root["bufferViews"].is_array() ||
      bufferViewIndex < 0 || bufferViewIndex >= static_cast<int>(root["bufferViews"].size())) {
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
      view.contains("byteOffset") && view["byteOffset"].is_number_integer()
          ? static_cast<size_t>(view["byteOffset"].get<int>())
          : 0U;
  const size_t accessorOffset =
      accessor.contains("byteOffset") && accessor["byteOffset"].is_number_integer()
          ? static_cast<size_t>(accessor["byteOffset"].get<int>())
          : 0U;
  const size_t stride =
      view.contains("byteStride") && view["byteStride"].is_number_integer()
          ? static_cast<size_t>(view["byteStride"].get<int>())
          : componentSize;
  const size_t base = viewOffset + accessorOffset;
  if (stride < componentSize) {
    return false;
  }

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
}

glm::mat4 ParseNodeLocalTransform(const nlohmann::json& node) {
  if (node.contains("matrix") && node["matrix"].is_array() && node["matrix"].size() == 16) {
    glm::mat4 matrix(1.0F);
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        const size_t idx = static_cast<size_t>(col * 4 + row);
        if (node["matrix"][idx].is_number()) {
          matrix[col][row] = node["matrix"][idx].get<float>();
        }
      }
    }
    return matrix;
  }

  glm::vec3 translation{0.0F, 0.0F, 0.0F};
  glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
  glm::vec3 scale{1.0F, 1.0F, 1.0F};

  if (node.contains("translation") && node["translation"].is_array() && node["translation"].size() >= 3) {
    translation = ParseJsonVec3(node["translation"], translation);
  }
  if (node.contains("scale") && node["scale"].is_array() && node["scale"].size() >= 3) {
    scale = ParseJsonVec3(node["scale"], scale);
  }
  if (node.contains("rotation") && node["rotation"].is_array() && node["rotation"].size() >= 4) {
    const auto& r = node["rotation"];
    if (r[0].is_number() && r[1].is_number() && r[2].is_number() && r[3].is_number()) {
      rotation = glm::quat(r[3].get<float>(), r[0].get<float>(), r[1].get<float>(), r[2].get<float>());
    }
  }

  glm::mat4 transform(1.0F);
  transform = glm::translate(transform, translation) * glm::mat4_cast(rotation);
  transform = glm::scale(transform, scale);
  return transform;
}

bool PrimitiveUsesTriangles(const nlohmann::json& primitive) {
  if (!primitive.contains("mode") || !primitive["mode"].is_number_integer()) {
    return true;
  }
  return primitive["mode"].get<int>() == 4;
}

void AppendPrimitiveVertices(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const nlohmann::json& primitive,
    const glm::mat4& nodeTransform,
    PositionNormalMesh& outMesh) {
  if (!PrimitiveUsesTriangles(primitive)) {
    return;
  }
  if (!primitive.contains("attributes") || !primitive["attributes"].is_object()) {
    return;
  }
  const auto& attrs = primitive["attributes"];
  if (!attrs.contains("POSITION") || !attrs["POSITION"].is_number_integer()) {
    return;
  }

  const int positionAccessorIndex = attrs["POSITION"].get<int>();
  const int normalAccessorIndex =
      (attrs.contains("NORMAL") && attrs["NORMAL"].is_number_integer()) ? attrs["NORMAL"].get<int>() : -1;
  const int colorAccessorIndex =
      (attrs.contains("COLOR_0") && attrs["COLOR_0"].is_number_integer()) ? attrs["COLOR_0"].get<int>() : -1;
  const int indexAccessorIndex =
      (primitive.contains("indices") && primitive["indices"].is_number_integer()) ? primitive["indices"].get<int>() : -1;

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec4> colors;
  if (!ReadAccessorVec3(root, buffers, positionAccessorIndex, positions)) {
    return;
  }
  if (normalAccessorIndex >= 0 && !ReadAccessorVec3(root, buffers, normalAccessorIndex, normals)) {
    normals.clear();
  }
  if (colorAccessorIndex >= 0 && !ReadAccessorColorVec4(root, buffers, colorAccessorIndex, colors)) {
    colors.clear();
  }
  const glm::vec4 primitiveColor = ResolvePrimitiveBaseColor(root, primitive);

  std::vector<std::uint32_t> indices;
  if (indexAccessorIndex >= 0) {
    if (!ReadAccessorIndices(root, buffers, indexAccessorIndex, indices)) {
      return;
    }
  } else {
    indices.resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
      indices[i] = static_cast<std::uint32_t>(i);
    }
  }
  if (indices.size() < 3) {
    return;
  }

  const glm::mat3 normalMatrix = glm::mat3(glm::transpose(glm::inverse(nodeTransform)));
  for (size_t i = 0; i + 2 < indices.size(); i += 3) {
    const std::uint32_t ia = indices[i];
    const std::uint32_t ib = indices[i + 1];
    const std::uint32_t ic = indices[i + 2];
    if (ia >= positions.size() || ib >= positions.size() || ic >= positions.size()) {
      continue;
    }

    const glm::vec3 pa = glm::vec3(nodeTransform * glm::vec4{positions[ia], 1.0F});
    const glm::vec3 pb = glm::vec3(nodeTransform * glm::vec4{positions[ib], 1.0F});
    const glm::vec3 pc = glm::vec3(nodeTransform * glm::vec4{positions[ic], 1.0F});

    glm::vec3 na = ComputeFallbackNormal(pa, pb, pc);
    glm::vec3 nb = na;
    glm::vec3 nc = na;
    if (normals.size() == positions.size()) {
      na = NormalizeSafe(normalMatrix * normals[ia]);
      nb = NormalizeSafe(normalMatrix * normals[ib]);
      nc = NormalizeSafe(normalMatrix * normals[ic]);
    }

    glm::vec4 ca = primitiveColor;
    glm::vec4 cb = primitiveColor;
    glm::vec4 cc = primitiveColor;
    if (colors.size() == positions.size()) {
      ca *= colors[ia];
      cb *= colors[ib];
      cc *= colors[ic];
    }

    outMesh.vertices.push_back({pa, na, ca});
    outMesh.vertices.push_back({pb, nb, cb});
    outMesh.vertices.push_back({pc, nc, cc});
  }
}

void TraverseGltfNode(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const int nodeIndex,
    const glm::mat4& parentTransform,
    std::vector<int>& visitState,
    PositionNormalMesh& outMesh) {
  if (!root.contains("nodes") || !root["nodes"].is_array() ||
      nodeIndex < 0 || nodeIndex >= static_cast<int>(root["nodes"].size())) {
    return;
  }
  if (visitState[static_cast<size_t>(nodeIndex)] == 1) {
    return;
  }

  visitState[static_cast<size_t>(nodeIndex)] = 1;
  const auto& node = root["nodes"][static_cast<size_t>(nodeIndex)];
  const glm::mat4 world = parentTransform * ParseNodeLocalTransform(node);

  if (node.contains("mesh") && node["mesh"].is_number_integer() &&
      root.contains("meshes") && root["meshes"].is_array()) {
    const int meshIndex = node["mesh"].get<int>();
    if (meshIndex >= 0 && meshIndex < static_cast<int>(root["meshes"].size())) {
      const auto& mesh = root["meshes"][static_cast<size_t>(meshIndex)];
      if (mesh.contains("primitives") && mesh["primitives"].is_array()) {
        for (const auto& primitive : mesh["primitives"]) {
          if (primitive.is_object()) {
            AppendPrimitiveVertices(root, buffers, primitive, world, outMesh);
          }
        }
      }
    }
  }

  if (node.contains("children") && node["children"].is_array()) {
    for (const auto& child : node["children"]) {
      if (child.is_number_integer()) {
        TraverseGltfNode(root, buffers, child.get<int>(), world, visitState, outMesh);
      }
    }
  }

  visitState[static_cast<size_t>(nodeIndex)] = 2;
}

}  // namespace repulsor3d::cad_import
