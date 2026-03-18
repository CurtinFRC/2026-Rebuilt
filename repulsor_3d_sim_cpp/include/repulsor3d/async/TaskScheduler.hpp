#pragma once

#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <future>
#include <mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace repulsor3d {

class TaskScheduler {
 public:
  enum class TaskPriority : std::uint8_t {
    High = 0,
    Normal = 1,
    Low = 2,
  };

  class CancellationToken {
   public:
    CancellationToken() = default;
    explicit CancellationToken(std::shared_ptr<std::atomic<bool>> flag) : flag_(std::move(flag)) {}

    static CancellationToken Create() { return CancellationToken{std::make_shared<std::atomic<bool>>(false)}; }
    void Cancel() const {
      if (flag_ != nullptr) {
        flag_->store(true);
      }
    }
    bool IsCanceled() const { return flag_ != nullptr && flag_->load(); }
    bool Valid() const { return flag_ != nullptr; }

   private:
    std::shared_ptr<std::atomic<bool>> flag_;
  };

  TaskScheduler() = default;
  ~TaskScheduler();

  TaskScheduler(const TaskScheduler&) = delete;
  TaskScheduler& operator=(const TaskScheduler&) = delete;

  void Start(std::size_t threadCount);
  void Stop();

  void Enqueue(
      std::packaged_task<void()>&& task,
      TaskPriority priority = TaskPriority::Normal,
      CancellationToken token = {});

  template <typename TResult>
  void Enqueue(
      std::packaged_task<TResult()>&& task,
      TaskPriority priority = TaskPriority::Normal,
      CancellationToken token = {}) {
    std::packaged_task<void()> wrapper([wrapped = std::move(task)]() mutable {
      if (wrapped.valid()) {
        wrapped();
      }
    });
    Enqueue(std::move(wrapper), priority, std::move(token));
  }

 private:
  struct TaskItem {
    TaskPriority priority = TaskPriority::Normal;
    std::uint64_t sequence = 0;
    CancellationToken token;
    std::packaged_task<void()> task;
  };

  static bool CompareTaskItem(const TaskItem& lhs, const TaskItem& rhs);

  std::vector<std::thread> workers_;
  std::mutex queueMutex_;
  std::condition_variable queueCv_;
  bool stopRequested_ = false;
  std::deque<TaskItem> queue_;
  std::uint64_t nextSequence_ = 0;
};

}  // namespace repulsor3d
