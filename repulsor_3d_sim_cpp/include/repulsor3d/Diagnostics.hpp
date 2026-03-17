#pragma once

#include <string>
#include <vector>

namespace repulsor3d {

struct FeatureTiming {
  std::string name;
  double milliseconds = 0.0;
};

struct DiagnosticsSnapshot {
  double frameMilliseconds = 0.0;
  std::vector<FeatureTiming> featureTimings;
};

class DiagnosticsService {
 public:
  void BeginFrame();
  void RecordFeatureTime(const std::string& featureName, double milliseconds);
  void EndFrame(double frameMilliseconds);

  const DiagnosticsSnapshot& Latest() const;

 private:
  DiagnosticsSnapshot current_;
  DiagnosticsSnapshot latest_;
};

}  // namespace repulsor3d
