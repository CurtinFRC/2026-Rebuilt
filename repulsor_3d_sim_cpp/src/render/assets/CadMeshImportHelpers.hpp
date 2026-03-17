#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <nlohmann/json.hpp>

#include "repulsor3d/render/geometry/GeometryProvider.hpp"

namespace repulsor3d::cad_import {

glm::vec3 NormalizeSafe(const glm::vec3& v);
glm::vec3 ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

int ResolveObjIndex(int indexValue, int size);
bool ParseObjFaceVertex(const std::string& token, int& outPos, int& outNorm);

bool LoadBufferUri(const std::filesystem::path& modelPath, const std::string& uri, std::vector<std::uint8_t>& out);
bool LoadGltfRoot(const std::string& resolvedAssetPath, nlohmann::json& outRoot, std::vector<std::uint8_t>& outGlbBinChunk);
bool ReadAccessorVec3(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    int accessorIndex,
    std::vector<glm::vec3>& out);
bool ReadAccessorIndices(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    int accessorIndex,
    std::vector<std::uint32_t>& out);

glm::mat4 ParseNodeLocalTransform(const nlohmann::json& node);
bool PrimitiveUsesTriangles(const nlohmann::json& primitive);
void AppendPrimitiveVertices(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const nlohmann::json& primitive,
    const glm::mat4& nodeTransform,
    PositionNormalMesh& outMesh);
void TraverseGltfNode(
    const nlohmann::json& root,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    int nodeIndex,
    const glm::mat4& parentTransform,
    std::vector<int>& visitState,
    PositionNormalMesh& outMesh);

}  // namespace repulsor3d::cad_import
