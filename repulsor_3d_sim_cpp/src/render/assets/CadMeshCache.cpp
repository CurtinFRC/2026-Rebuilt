#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <utility>

namespace repulsor3d {
namespace {

constexpr char kMeshCacheMagicV1[] = "R3DMESH1";
constexpr char kMeshCacheMagicV2[] = "R3DMESH2";

}  // namespace

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

  char magic[sizeof(kMeshCacheMagicV2)]{};
  in.read(magic, static_cast<std::streamsize>(sizeof(kMeshCacheMagicV2) - 1));
  if (!in) {
    return false;
  }
  const bool isV1 = std::strncmp(magic, kMeshCacheMagicV1, sizeof(kMeshCacheMagicV1) - 1) == 0;
  const bool isV2 = std::strncmp(magic, kMeshCacheMagicV2, sizeof(kMeshCacheMagicV2) - 1) == 0;
  if (!isV1 && !isV2) {
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
    if (isV2) {
      in.read(reinterpret_cast<char*>(&v.color.r), sizeof(float));
      in.read(reinterpret_cast<char*>(&v.color.g), sizeof(float));
      in.read(reinterpret_cast<char*>(&v.color.b), sizeof(float));
      in.read(reinterpret_cast<char*>(&v.color.a), sizeof(float));
    } else {
      v.color = glm::vec4{1.0F, 1.0F, 1.0F, 1.0F};
    }
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

  out.write(kMeshCacheMagicV2, static_cast<std::streamsize>(sizeof(kMeshCacheMagicV2) - 1));
  const std::uint32_t count = static_cast<std::uint32_t>(mesh.vertices.size());
  out.write(reinterpret_cast<const char*>(&count), sizeof(std::uint32_t));
  for (const auto& v : mesh.vertices) {
    out.write(reinterpret_cast<const char*>(&v.position.x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.position.y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.position.z), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.normal.z), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.color.r), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.color.g), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.color.b), sizeof(float));
    out.write(reinterpret_cast<const char*>(&v.color.a), sizeof(float));
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

std::unique_ptr<ICadMeshCache> CreateDefaultCadMeshCache() {
  return std::make_unique<CompositeCadMeshCache>(
      std::make_unique<InMemoryCadMeshCache>(),
      std::make_unique<FileCadMeshCache>(".repulsor_cache/cad_mesh"));
}

}  // namespace repulsor3d
