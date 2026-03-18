#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <thread>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSource.hpp"
#include "repulsor3d/TruthSocketReceiver.hpp"

namespace repulsor3d {

class SnapshotWorker {
 public:
  SnapshotWorker(
      ISnapshotSource& source,
      double hz,
      const ViewerConfig& cfg,
      const TruthSocketReceiver* truthReceiver = nullptr);
  ~SnapshotWorker();

  SnapshotWorker(const SnapshotWorker&) = delete;
  SnapshotWorker& operator=(const SnapshotWorker&) = delete;

  void Start();
  void Stop();

  SnapshotBundle Latest() const;

 private:
  void Run();

  ISnapshotSource& source_;
  double periodS_;
  ViewerConfig cfg_;
  const TruthSocketReceiver* truthReceiver_;

  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::thread thread_;

  mutable std::mutex mutex_;
  SnapshotBundle latest_;
  std::ofstream recordStream_;
  bool recordEnabled_ = false;
};

}  // namespace repulsor3d
