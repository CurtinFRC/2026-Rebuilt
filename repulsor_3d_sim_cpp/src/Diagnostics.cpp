#include "repulsor3d/Diagnostics.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace repulsor3d {
namespace {

constexpr double kAverageAlpha = 0.15;

#ifndef REPULSOR_DIAGNOSTICS_FILE_LOGGING
#define REPULSOR_DIAGNOSTICS_FILE_LOGGING 0
#endif

#if REPULSOR_DIAGNOSTICS_FILE_LOGGING

bool ParseBoolEnv(const char* name, const bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  const std::string text(value);
  if (text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF") {
    return false;
  }
  if (text == "1" || text == "true" || text == "TRUE" || text == "on" || text == "ON") {
    return true;
  }
  return fallback;
}

std::string ParseStringEnv(const char* name, std::string fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return std::string(value);
}

std::string EscapeJson(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

class DiagnosticsFileLogger {
 public:
  void WriteFrame(const DiagnosticsSnapshot& snapshot) {
    EnsureInitialized();
    if (!enabled_ || !stream_.is_open()) {
      return;
    }

    stream_ << std::fixed << std::setprecision(3);
    stream_ << "{";
    stream_ << "\"frame_ms\":" << snapshot.frameMilliseconds << ",";
    stream_ << "\"frame_avg_ms\":" << snapshot.frameAverageMilliseconds << ",";
    WriteTimingArray("pass_timings", snapshot.passTimings);
    stream_ << ",";
    WriteTimingArray("asset_timings", snapshot.assetTimings);
    stream_ << ",";
    WriteTimingArray("gpu_timings", snapshot.gpuTimings);
    stream_ << ",";
    WriteCounterArray("counters", snapshot.counters);
    stream_ << ",";
    WriteMessages(snapshot.messages);
    stream_ << "}\n";

    ++framesSinceFlush_;
    if (framesSinceFlush_ >= flushEveryFrames_) {
      stream_.flush();
      framesSinceFlush_ = 0;
    }
  }

 private:
  void EnsureInitialized() {
    if (initialized_) {
      return;
    }
    initialized_ = true;
    enabled_ = ParseBoolEnv("DIAGNOSTICS_LOG_ENABLED", true);
    if (!enabled_) {
      return;
    }
    const std::string path = ParseStringEnv("DIAGNOSTICS_LOG_PATH", "repulsor_diagnostics.log");
    stream_.open(path, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
      enabled_ = false;
      return;
    }
    flushEveryFrames_ = std::max(1, ParseFlushEveryFrames());
  }

  int ParseFlushEveryFrames() const {
    const char* value = std::getenv("DIAGNOSTICS_LOG_FLUSH_EVERY_FRAMES");
    if (value == nullptr || *value == '\0') {
      return 20;
    }
    try {
      return std::stoi(value);
    } catch (...) {
      return 20;
    }
  }

  void WriteTimingArray(const char* fieldName, const std::vector<TimingSample>& timings) {
    stream_ << "\"" << fieldName << "\":[";
    for (std::size_t i = 0; i < timings.size(); ++i) {
      const auto& t = timings[i];
      if (i > 0) {
        stream_ << ",";
      }
      stream_ << "{";
      stream_ << "\"name\":\"" << EscapeJson(t.name) << "\",";
      stream_ << "\"ms\":" << t.milliseconds << ",";
      stream_ << "\"avg_ms\":" << t.averageMilliseconds;
      stream_ << "}";
    }
    stream_ << "]";
  }

  void WriteCounterArray(const char* fieldName, const std::vector<CounterSample>& counters) {
    stream_ << "\"" << fieldName << "\":[";
    for (std::size_t i = 0; i < counters.size(); ++i) {
      const auto& c = counters[i];
      if (i > 0) {
        stream_ << ",";
      }
      stream_ << "{";
      stream_ << "\"name\":\"" << EscapeJson(c.name) << "\",";
      stream_ << "\"value\":" << c.value << ",";
      stream_ << "\"avg_value\":" << c.averageValue;
      stream_ << "}";
    }
    stream_ << "]";
  }

  void WriteMessages(const std::vector<std::string>& messages) {
    stream_ << "\"messages\":[";
    for (std::size_t i = 0; i < messages.size(); ++i) {
      if (i > 0) {
        stream_ << ",";
      }
      stream_ << "\"" << EscapeJson(messages[i]) << "\"";
    }
    stream_ << "]";
  }

  bool initialized_ = false;
  bool enabled_ = true;
  std::ofstream stream_;
  int flushEveryFrames_ = 20;
  int framesSinceFlush_ = 0;
};

DiagnosticsFileLogger& GetDiagnosticsFileLogger() {
  static DiagnosticsFileLogger logger;
  return logger;
}

#endif  // REPULSOR_DIAGNOSTICS_FILE_LOGGING

}  // namespace

void DiagnosticsService::BeginFrame() {
  current_.frameMilliseconds = 0.0;
  current_.frameAverageMilliseconds = 0.0;
  current_.passTimings.clear();
  current_.assetTimings.clear();
  current_.gpuTimings.clear();
  current_.counters.clear();
  current_.messages.clear();
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

void DiagnosticsService::RecordCounter(const std::string& counterName, const double value) {
  current_.counters.push_back({counterName, value, value});
}

void DiagnosticsService::RecordMessage(const std::string& message) {
  if (!message.empty()) {
    current_.messages.push_back(message);
  }
}

void DiagnosticsService::MarkSceneReady() {
  sceneReady_ = true;
}

bool DiagnosticsService::IsSceneReady() const {
  return sceneReady_;
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

void DiagnosticsService::UpdateCounterAverages(
    std::vector<CounterSample>& samples,
    std::unordered_map<std::string, double>& averages) {
  for (auto& sample : samples) {
    const auto it = averages.find(sample.name);
    if (it == averages.end()) {
      averages[sample.name] = sample.value;
      sample.averageValue = sample.value;
      continue;
    }

    const double nextAverage = (1.0 - kAverageAlpha) * it->second + kAverageAlpha * sample.value;
    it->second = nextAverage;
    sample.averageValue = nextAverage;
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
  UpdateCounterAverages(current_.counters, counterAverages_);

  latest_ = current_;

#if REPULSOR_DIAGNOSTICS_FILE_LOGGING
  if (sceneReady_) {
    GetDiagnosticsFileLogger().WriteFrame(latest_);
  }
#endif
}

const DiagnosticsSnapshot& DiagnosticsService::Latest() const {
  return latest_;
}

}  // namespace repulsor3d
