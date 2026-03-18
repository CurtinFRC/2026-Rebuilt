#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace repulsor3d {

struct TimingSample {
  std::string name;
  double milliseconds = 0.0;
  double averageMilliseconds = 0.0;
};

struct CounterSample {
  std::string name;
  double value = 0.0;
  double averageValue = 0.0;
};

struct DiagnosticsSnapshot {
  double frameMilliseconds = 0.0;
  double frameAverageMilliseconds = 0.0;
  std::vector<TimingSample> passTimings;
  std::vector<TimingSample> assetTimings;
  std::vector<TimingSample> gpuTimings;
  std::vector<CounterSample> counters;
};

class DiagnosticsService {
 public:
  void BeginFrame();
  void RecordPassTime(const std::string& passName, double milliseconds);
  void RecordAssetTime(const std::string& assetEventName, double milliseconds);
  void RecordGpuTime(const std::string& passName, double milliseconds);
  void RecordCounter(const std::string& counterName, double value);
  void EndFrame(double frameMilliseconds);

  const DiagnosticsSnapshot& Latest() const;

 private:
  static void UpdateAverages(std::vector<TimingSample>& samples, std::unordered_map<std::string, double>& averages);
  static void UpdateCounterAverages(std::vector<CounterSample>& samples, std::unordered_map<std::string, double>& averages);

  DiagnosticsSnapshot current_;
  DiagnosticsSnapshot latest_;
  std::unordered_map<std::string, double> passAverages_;
  std::unordered_map<std::string, double> assetAverages_;
  std::unordered_map<std::string, double> gpuAverages_;
  std::unordered_map<std::string, double> counterAverages_;
  double frameAverageMilliseconds_ = 0.0;
  bool frameAverageInitialized_ = false;
};

}  // namespace repulsor3d
