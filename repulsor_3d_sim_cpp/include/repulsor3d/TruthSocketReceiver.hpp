#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "repulsor3d/Model.hpp"

namespace repulsor3d {

class TruthSocketReceiver {
 public:
  TruthSocketReceiver(std::string host, int port);
  ~TruthSocketReceiver();

  TruthSocketReceiver(const TruthSocketReceiver&) = delete;
  TruthSocketReceiver& operator=(const TruthSocketReceiver&) = delete;

  void Start();
  void Stop();

  std::vector<FieldVisionObject> Latest() const;

 private:
  void Run();

  std::string host_;
  int port_;

  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::thread thread_;

  mutable std::mutex mutex_;
  std::vector<FieldVisionObject> latest_;
};

}  // namespace repulsor3d
