#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {
namespace {

constexpr std::uint64_t kFnv64OffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnv64Prime = 1099511628211ULL;

void FnvMixBytes(std::uint64_t& hash, const std::uint8_t* data, const std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    hash ^= static_cast<std::uint64_t>(data[i]);
    hash *= kFnv64Prime;
  }
}

std::string BuildStableFileFingerprint(const std::string& filePath) {
  constexpr std::size_t kSampleBytes = 128 * 1024;
  std::ifstream in(filePath, std::ios::binary);
  if (!in) {
    return "";
  }

  in.seekg(0, std::ios::end);
  const std::streamoff streamSize = in.tellg();
  if (streamSize <= 0) {
    return "";
  }
  const std::size_t fileSize = static_cast<std::size_t>(streamSize);
  in.seekg(0, std::ios::beg);

  std::uint64_t hash = kFnv64OffsetBasis;
  const auto mixInteger = [&](const std::uint64_t value) {
    std::array<std::uint8_t, sizeof(std::uint64_t)> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      bytes[i] = static_cast<std::uint8_t>((value >> (i * 8U)) & 0xFFU);
    }
    FnvMixBytes(hash, bytes.data(), bytes.size());
  };

  const std::string filename = std::filesystem::path(filePath).filename().string();
  FnvMixBytes(hash, reinterpret_cast<const std::uint8_t*>(filename.data()), filename.size());
  mixInteger(static_cast<std::uint64_t>(fileSize));

  const std::size_t headBytes = std::min<std::size_t>(fileSize, kSampleBytes);
  std::vector<std::uint8_t> buffer(headBytes);
  in.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
  if (!in) {
    return "";
  }
  FnvMixBytes(hash, buffer.data(), buffer.size());

  if (fileSize > kSampleBytes) {
    const std::size_t tailBytes = std::min<std::size_t>(fileSize - headBytes, kSampleBytes);
    std::vector<std::uint8_t> tail(tailBytes);
    in.clear();
    in.seekg(static_cast<std::streamoff>(fileSize - tailBytes), std::ios::beg);
    in.read(reinterpret_cast<char*>(tail.data()), static_cast<std::streamsize>(tail.size()));
    if (!in) {
      return "";
    }
    FnvMixBytes(hash, tail.data(), tail.size());
  }

  std::ostringstream out;
  out << std::hex << hash;
  return out.str();
}

}  // namespace

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

  std::string cacheKey = "cadmesh";
  {
    const std::string fingerprint = BuildStableFileFingerprint(resolved);
    if (!fingerprint.empty()) {
      cacheKey += "|fp=" + fingerprint;
    } else {
      cacheKey += "|path=" + std::filesystem::path(resolved).filename().string();
    }
    std::error_code ec;
    const auto fileSize = std::filesystem::file_size(resolved, ec);
    if (!ec) {
      cacheKey += "|sz=" + std::to_string(static_cast<unsigned long long>(fileSize));
    }
    cacheKey += "|fmt_v=5";
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
  std::cerr << "CAD cache " << (cacheHit ? "hit" : "miss") << ": " << resolved << "\n";
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
