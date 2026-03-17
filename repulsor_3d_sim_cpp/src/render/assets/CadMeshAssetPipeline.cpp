#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {

void PassThroughCadMeshCooker::Cook(PositionNormalMesh& /*mesh*/) {}

AdaptiveCadMeshCooker::AdaptiveCadMeshCooker(const std::size_t maxVertices) : maxVertices_(maxVertices) {}

void AdaptiveCadMeshCooker::Cook(PositionNormalMesh& mesh) {
  if (maxVertices_ < 3 || mesh.vertices.size() <= maxVertices_) {
    return;
  }

  const std::size_t triangleCount = mesh.vertices.size() / 3;
  if (triangleCount <= 1) {
    return;
  }

  const std::size_t targetTriangles = std::max<std::size_t>(1, maxVertices_ / 3);
  const std::size_t step = (triangleCount + targetTriangles - 1) / targetTriangles;
  if (step <= 1) {
    return;
  }

  std::vector<PositionNormalMesh::Vertex> reduced;
  reduced.reserve(((triangleCount + step - 1) / step) * 3);
  for (std::size_t tri = 0; tri < triangleCount; tri += step) {
    const std::size_t base = tri * 3;
    reduced.push_back(mesh.vertices[base + 0]);
    reduced.push_back(mesh.vertices[base + 1]);
    reduced.push_back(mesh.vertices[base + 2]);
  }
  if (!reduced.empty()) {
    mesh.vertices.swap(reduced);
  }
}

std::unique_ptr<ICadMeshCooker> CreateDefaultCadMeshCooker() {
  if (const char* env = std::getenv("CAD_MAX_VERTICES"); env != nullptr) {
    try {
      const unsigned long long parsed = std::stoull(env);
      if (parsed >= 3ULL) {
        return std::make_unique<AdaptiveCadMeshCooker>(static_cast<std::size_t>(parsed));
      }
    } catch (...) {
    }
  }
  return std::make_unique<PassThroughCadMeshCooker>();
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

  const auto totalStart = std::chrono::steady_clock::now();
  const std::string resolved = resolver_.ResolveFilePath(assetPath);
  if (resolved.empty()) {
    return false;
  }

  std::string cacheKey = resolved;
  {
    std::error_code ec;
    const auto writeTime = std::filesystem::last_write_time(resolved, ec);
    if (!ec) {
      cacheKey += "|wt=" + std::to_string(writeTime.time_since_epoch().count());
    }
    ec.clear();
    const auto fileSize = std::filesystem::file_size(resolved, ec);
    if (!ec) {
      cacheKey += "|sz=" + std::to_string(static_cast<unsigned long long>(fileSize));
    }
    cacheKey += "|fmt_v=4";
  }

  auto emitTiming = [&](const std::string& eventName, const std::chrono::steady_clock::duration elapsed) {
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    telemetryEvents_.push_back(
        {eventName,
         resolved,
         std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed).count()});
  };

  const auto cacheLookupStart = std::chrono::steady_clock::now();
  const bool cacheHit = cache_->TryLoad(cacheKey, outMesh);
  emitTiming(
      cacheHit ? "cache_lookup_hit" : "cache_lookup_miss",
      std::chrono::steady_clock::now() - cacheLookupStart);
  if (cacheHit) {
    emitTiming("load_total", std::chrono::steady_clock::now() - totalStart);
    return true;
  }

  PositionNormalMesh mesh;
  const auto importStart = std::chrono::steady_clock::now();
  if (!importer_->Import(resolved, mesh)) {
    emitTiming("import_failed", std::chrono::steady_clock::now() - importStart);
    emitTiming("load_total", std::chrono::steady_clock::now() - totalStart);
    return false;
  }
  emitTiming("import", std::chrono::steady_clock::now() - importStart);

  const auto cookStart = std::chrono::steady_clock::now();
  cooker_->Cook(mesh);
  emitTiming("cook", std::chrono::steady_clock::now() - cookStart);

  const auto storeStart = std::chrono::steady_clock::now();
  cache_->Store(cacheKey, mesh);
  emitTiming("cache_store", std::chrono::steady_clock::now() - storeStart);

  emitTiming("load_total", std::chrono::steady_clock::now() - totalStart);
  outMesh = std::move(mesh);
  return true;
}

std::vector<CadMeshAssetPipeline::TelemetryEvent> CadMeshAssetPipeline::ConsumeTelemetryEvents() {
  std::lock_guard<std::mutex> lock(telemetryMutex_);
  std::vector<TelemetryEvent> out;
  out.swap(telemetryEvents_);
  return out;
}

}  // namespace repulsor3d
