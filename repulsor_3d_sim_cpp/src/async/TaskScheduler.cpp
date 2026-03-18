#include "repulsor3d/async/TaskScheduler.hpp"

namespace repulsor3d {

TaskScheduler::~TaskScheduler() {
  Stop();
}

void TaskScheduler::Start(std::size_t threadCount) {
  Stop();
  if (threadCount == 0) {
    threadCount = 1;
  }

  stopRequested_ = false;
  workers_.reserve(threadCount);
  for (std::size_t i = 0; i < threadCount; ++i) {
    workers_.emplace_back([this]() {
      for (;;) {
        std::packaged_task<void()> task;
        {
          std::unique_lock<std::mutex> lock(queueMutex_);
          queueCv_.wait(lock, [this]() { return stopRequested_ || !queue_.empty(); });
          if (stopRequested_ && queue_.empty()) {
            return;
          }
          task = std::move(queue_.front());
          queue_.pop_front();
        }
        if (task.valid()) {
          task();
        }
      }
    });
  }
}

void TaskScheduler::Stop() {
  {
    std::scoped_lock lock(queueMutex_);
    stopRequested_ = true;
  }
  queueCv_.notify_all();

  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  workers_.clear();

  std::scoped_lock lock(queueMutex_);
  queue_.clear();
}

void TaskScheduler::Enqueue(std::packaged_task<void()>&& task) {
  {
    std::scoped_lock lock(queueMutex_);
    queue_.push_back(std::move(task));
  }
  queueCv_.notify_one();
}

}  // namespace repulsor3d
