#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "repulsor3d/DataSource.hpp"

namespace repulsor3d {

class ReplayDataSource final : public ISnapshotSource {
 public:
  ReplayDataSource(std::string snapshotPath, bool loopPlayback);
  ~ReplayDataSource() override = default;

  SnapshotBundle Read() override;

 private:
  bool LoadSnapshotFile(std::string* error);

  std::string snapshotPath_;
  bool loopPlayback_ = true;
  bool loaded_ = false;
  std::vector<SnapshotBundle> frames_;
  std::size_t cursor_ = 0;
  mutable std::mutex mutex_;
};

}  // namespace repulsor3d

