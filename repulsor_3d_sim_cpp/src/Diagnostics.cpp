#include "repulsor3d/Diagnostics.hpp"

namespace repulsor3d {
namespace {

constexpr double kAverageAlpha = 0.15;

}

void DiagnosticsService::BeginFrame() {
  current_.frameMilliseconds = 0.0;
  current_.frameAverageMilliseconds = 0.0;
  current_.passTimings.clear();
  current_.assetTimings.clear();
  current_.gpuTimings.clear();
}

void DiagnosticsService::RecordPassTime(const std::string& passName, const double milliseconds) {
  current_.passTimings.push_back({passName, milliseconds});
}

void DiagnosticsService::RecordAssetTime(const std::string& assetEventName, const double milliseconds) {
  current_.assetTimings.push_back({assetEventName, milliseconds});
}

void DiagnosticsService::RecordGpuTime(const std::string& passName, const double milliseconds) {
  current_.gpuTimings.push_back({passName, milliseconds});
}

void DiagnosticsService::UpdateAverages(
    std::vector<TimingSample>& samples,
    std::unordered_map<std::string, double>& averages) {
  for (auto& sample : samples) {
    const auto it = averages.find(sample.name);
    if (it == averages.end()) {
      averages[sample.name] = sample.milliseconds;
      sample.averageMilliseconds = sample.milliseconds;
      continue;
    }

    const double nextAverage = (1.0 - kAverageAlpha) * it->second + kAverageAlpha * sample.milliseconds;
    it->second = nextAverage;
    sample.averageMilliseconds = nextAverage;
  }
}

void DiagnosticsService::EndFrame(const double frameMilliseconds) {
  current_.frameMilliseconds = frameMilliseconds;
  if (!frameAverageInitialized_) {
    frameAverageMilliseconds_ = frameMilliseconds;
    frameAverageInitialized_ = true;
  } else {
    frameAverageMilliseconds_ = (1.0 - kAverageAlpha) * frameAverageMilliseconds_ + kAverageAlpha * frameMilliseconds;
  }
  current_.frameAverageMilliseconds = frameAverageMilliseconds_;

  UpdateAverages(current_.passTimings, passAverages_);
  UpdateAverages(current_.assetTimings, assetAverages_);
  UpdateAverages(current_.gpuTimings, gpuAverages_);

  latest_ = current_;
}

const DiagnosticsSnapshot& DiagnosticsService::Latest() const {
  return latest_;
}

}  // namespace repulsor3d
