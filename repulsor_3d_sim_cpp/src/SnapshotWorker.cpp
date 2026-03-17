#include "repulsor3d/SnapshotWorker.hpp"

#include <algorithm>
#include <chrono>

namespace repulsor3d {

SnapshotWorker::SnapshotWorker(ISnapshotSource& source, const double hz, const TruthSocketReceiver* truthReceiver)
    : source_(source),
      periodS_(1.0 / std::max(1.0, hz)),
      truthReceiver_(truthReceiver) {}

SnapshotWorker::~SnapshotWorker() { Stop(); }

void SnapshotWorker::Start() {
  if (running_.exchange(true)) {
    return;
  }

  stop_.store(false);
  thread_ = std::thread(&SnapshotWorker::Run, this);
}

void SnapshotWorker::Stop() {
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
  running_.store(false);
}

SnapshotBundle SnapshotWorker::Latest() const {
  std::scoped_lock lock(mutex_);
  return latest_;
}

void SnapshotWorker::Run() {
  auto nextTick = std::chrono::steady_clock::now();

  while (!stop_.load()) {
    const auto now = std::chrono::steady_clock::now();
    if (now < nextTick) {
      std::this_thread::sleep_for(nextTick - now);
      continue;
    }

    SnapshotBundle bundle = source_.Read();
    if (truthReceiver_ != nullptr) {
      bundle.snapshot.truth = truthReceiver_->Latest();
    }

    {
      std::scoped_lock lock(mutex_);
      latest_ = std::move(bundle);
    }

    nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(periodS_));

    const auto floorTick = std::chrono::steady_clock::now() + std::chrono::microseconds(500);
    if (nextTick < floorTick) {
      nextTick = floorTick;
    }
  }
}

}  // namespace repulsor3d
