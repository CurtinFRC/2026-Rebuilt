#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

#include <glm/mat4x4.hpp>
#include <nlohmann/json.hpp>

#include "CadMeshImportHelpers.hpp"

namespace repulsor3d {
namespace {

class GltfSceneAssembler {
 public:
  GltfSceneAssembler(nlohmann::json root, std::vector<std::uint8_t> embeddedGlbBinChunk, std::filesystem::path modelPath)
      : root_(std::move(root)),
        embeddedGlbBinChunk_(std::move(embeddedGlbBinChunk)),
        modelPath_(std::move(modelPath)) {}

  bool Build(PositionNormalMesh& outMesh) const {
    if (!HasRequiredRootFields()) {
      return false;
    }

    std::vector<std::vector<std::uint8_t>> buffers;
    if (!LoadBuffers(buffers)) {
      return false;
    }

    outMesh.vertices.clear();
    AppendFromSceneGraph(buffers, outMesh);
    if (outMesh.vertices.empty()) {
      AppendFromAllMeshes(buffers, outMesh);
    }
    return !outMesh.vertices.empty();
  }

 private:
  bool HasRequiredRootFields() const {
    return root_.contains("buffers") && root_["buffers"].is_array() &&
           root_.contains("bufferViews") && root_["bufferViews"].is_array() &&
           root_.contains("accessors") && root_["accessors"].is_array() &&
           root_.contains("meshes") && root_["meshes"].is_array() &&
           !root_["meshes"].empty();
  }

  bool LoadBuffers(std::vector<std::vector<std::uint8_t>>& outBuffers) const {
    outBuffers.resize(root_["buffers"].size());

    for (size_t i = 0; i < root_["buffers"].size(); ++i) {
      const auto& bufferNode = root_["buffers"][i];
      if (bufferNode.contains("uri") && bufferNode["uri"].is_string()) {
        if (!cad_import::LoadBufferUri(modelPath_, bufferNode["uri"].get<std::string>(), outBuffers[i])) {
          return false;
        }
      } else if (i == 0 && !embeddedGlbBinChunk_.empty()) {
        outBuffers[i] = embeddedGlbBinChunk_;
      } else {
        return false;
      }

      if (bufferNode.contains("byteLength") && bufferNode["byteLength"].is_number_integer()) {
        const size_t expectedLength = static_cast<size_t>(bufferNode["byteLength"].get<int>());
        if (outBuffers[i].size() < expectedLength) {
          return false;
        }
      }
    }
    return true;
  }

  void AppendFromSceneGraph(
      const std::vector<std::vector<std::uint8_t>>& buffers,
      PositionNormalMesh& outMesh) const {
    const bool hasSceneGraph = root_.contains("scenes") && root_["scenes"].is_array() &&
                               root_.contains("nodes") && root_["nodes"].is_array();
    if (!hasSceneGraph) {
      return;
    }

    int sceneIndex = 0;
    if (root_.contains("scene") && root_["scene"].is_number_integer()) {
      sceneIndex = root_["scene"].get<int>();
    }
    if (sceneIndex < 0 || sceneIndex >= static_cast<int>(root_["scenes"].size())) {
      return;
    }

    const auto& scene = root_["scenes"][static_cast<size_t>(sceneIndex)];
    if (!scene.contains("nodes") || !scene["nodes"].is_array()) {
      return;
    }

    std::vector<int> visitState(root_["nodes"].size(), 0);
    for (const auto& nodeIndexNode : scene["nodes"]) {
      if (nodeIndexNode.is_number_integer()) {
        cad_import::TraverseGltfNode(
            root_,
            buffers,
            nodeIndexNode.get<int>(),
            glm::mat4(1.0F),
            visitState,
            outMesh);
      }
    }
  }

  void AppendFromAllMeshes(
      const std::vector<std::vector<std::uint8_t>>& buffers,
      PositionNormalMesh& outMesh) const {
    for (const auto& mesh : root_["meshes"]) {
      if (!mesh.is_object() || !mesh.contains("primitives") || !mesh["primitives"].is_array()) {
        continue;
      }
      for (const auto& primitive : mesh["primitives"]) {
        if (primitive.is_object()) {
          cad_import::AppendPrimitiveVertices(root_, buffers, primitive, glm::mat4(1.0F), outMesh);
        }
      }
    }
  }

  nlohmann::json root_;
  std::vector<std::uint8_t> embeddedGlbBinChunk_;
  std::filesystem::path modelPath_;
};

}  // namespace

bool GltfCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  nlohmann::json root;
  std::vector<std::uint8_t> embeddedGlbBinChunk;
  if (!cad_import::LoadGltfRoot(resolvedAssetPath, root, embeddedGlbBinChunk)) {
    return false;
  }
  const GltfSceneAssembler assembler(std::move(root), std::move(embeddedGlbBinChunk), std::filesystem::path(resolvedAssetPath));
  return assembler.Build(outMesh);
}

}  // namespace repulsor3d
