#include "repulsor3d/async/TaskScheduler.hpp"

namespace repulsor3d {

bool TaskScheduler::CompareTaskItem(const TaskItem& lhs, const TaskItem& rhs) {
  if (lhs.priority != rhs.priority) {
    return static_cast<int>(lhs.priority) < static_cast<int>(rhs.priority);
  }
  return lhs.sequence < rhs.sequence;
}

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
        TaskItem item;
        {
          std::unique_lock<std::mutex> lock(queueMutex_);
          queueCv_.wait(lock, [this]() { return stopRequested_ || !queue_.empty(); });
          if (stopRequested_ && queue_.empty()) {
            return;
          }
          auto best = queue_.begin();
          for (auto it = queue_.begin(); it != queue_.end(); ++it) {
            if (CompareTaskItem(*it, *best)) {
              best = it;
            }
          }
          item = std::move(*best);
          queue_.erase(best);
        }
        if (item.token.IsCanceled()) {
          continue;
        }
        if (item.task.valid()) {
          item.task();
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

void TaskScheduler::Enqueue(
    std::packaged_task<void()>&& task,
    const TaskPriority priority,
    CancellationToken token) {
  {
    std::scoped_lock lock(queueMutex_);
    queue_.push_back(TaskItem{
        .priority = priority,
        .sequence = nextSequence_++,
        .token = std::move(token),
        .task = std::move(task),
    });
  }
  queueCv_.notify_one();
}

}  // namespace repulsor3d
