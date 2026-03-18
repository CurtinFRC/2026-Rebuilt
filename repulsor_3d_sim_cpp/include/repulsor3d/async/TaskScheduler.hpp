#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace repulsor3d {

class TaskScheduler {
 public:
  TaskScheduler() = default;
  ~TaskScheduler();

  TaskScheduler(const TaskScheduler&) = delete;
  TaskScheduler& operator=(const TaskScheduler&) = delete;

  void Start(std::size_t threadCount);
  void Stop();

  void Enqueue(std::packaged_task<void()>&& task);

  template <typename TResult>
  void Enqueue(std::packaged_task<TResult()>&& task) {
    std::packaged_task<void()> wrapper([wrapped = std::move(task)]() mutable {
      if (wrapped.valid()) {
        wrapped();
      }
    });
    Enqueue(std::move(wrapper));
  }

 private:
  std::vector<std::thread> workers_;
  std::mutex queueMutex_;
  std::condition_variable queueCv_;
  bool stopRequested_ = false;
  std::deque<std::packaged_task<void()>> queue_;
};

}  // namespace repulsor3d
