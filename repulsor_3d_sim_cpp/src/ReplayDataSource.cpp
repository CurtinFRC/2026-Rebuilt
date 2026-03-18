#include "repulsor3d/ReplayDataSource.hpp"

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "repulsor3d/SnapshotIO.hpp"

namespace repulsor3d {

ReplayDataSource::ReplayDataSource(std::string snapshotPath, const bool loopPlayback)
    : snapshotPath_(std::move(snapshotPath)), loopPlayback_(loopPlayback) {}

SnapshotBundle ReplayDataSource::Read() {
  std::scoped_lock lock(mutex_);
  if (!loaded_) {
    std::string loadError;
    if (!LoadSnapshotFile(&loadError)) {
      if (!loadError.empty()) {
        std::cerr << "[ReplayDataSource] " << loadError << "\n";
      }
      return SnapshotBundle{};
    }
    loaded_ = true;
  }

  if (frames_.empty()) {
    return SnapshotBundle{};
  }

  if (cursor_ >= frames_.size()) {
    if (loopPlayback_) {
      cursor_ = 0;
    } else {
      cursor_ = frames_.size() - 1;
      return frames_[cursor_];
    }
  }

  const SnapshotBundle out = frames_[cursor_];
  cursor_ += 1;
  return out;
}

bool ReplayDataSource::LoadSnapshotFile(std::string* error) {
  frames_.clear();
  cursor_ = 0;

  if (snapshotPath_.empty()) {
    if (error != nullptr) {
      *error = "replay snapshot path is empty";
    }
    return false;
  }

  std::ifstream in(snapshotPath_);
  if (!in.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay snapshot file: " + snapshotPath_;
    }
    return false;
  }

  std::string line;
  std::size_t lineIndex = 0;
  while (std::getline(in, line)) {
    lineIndex += 1;
    if (line.empty()) {
      continue;
    }

    try {
      const auto jsonValue = nlohmann::json::parse(line);
      SnapshotBundle bundle;
      std::string parseError;
      if (SnapshotBundleFromJson(jsonValue, bundle, &parseError)) {
        frames_.push_back(std::move(bundle));
      } else if (error != nullptr) {
        *error = "failed to parse snapshot at line " + std::to_string(lineIndex) + ": " + parseError;
        return false;
      }
    } catch (const std::exception& ex) {
      if (error != nullptr) {
        *error = "invalid JSON in replay file at line " + std::to_string(lineIndex) + ": " + ex.what();
      }
      return false;
    }
  }

  if (frames_.empty()) {
    if (error != nullptr) {
      *error = "no valid replay snapshots found in: " + snapshotPath_;
    }
    return false;
  }

  return true;
}

}  // namespace repulsor3d

