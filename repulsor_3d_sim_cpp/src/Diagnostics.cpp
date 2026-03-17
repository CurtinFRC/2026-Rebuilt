#include "repulsor3d/Diagnostics.hpp"

namespace repulsor3d {

void DiagnosticsService::BeginFrame() {
  current_.frameMilliseconds = 0.0;
  current_.featureTimings.clear();
}

void DiagnosticsService::RecordFeatureTime(const std::string& featureName, const double milliseconds) {
  current_.featureTimings.push_back({featureName, milliseconds});
}

void DiagnosticsService::EndFrame(const double frameMilliseconds) {
  current_.frameMilliseconds = frameMilliseconds;
  latest_ = current_;
}

const DiagnosticsSnapshot& DiagnosticsService::Latest() const {
  return latest_;
}

}  // namespace repulsor3d
