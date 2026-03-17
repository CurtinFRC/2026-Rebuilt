#pragma once

#include <chrono>

namespace repulsor3d {

class ITimeSource {
 public:
  virtual ~ITimeSource() = default;
  virtual std::chrono::steady_clock::time_point Now() const = 0;
};

class SteadyTimeSource final : public ITimeSource {
 public:
  std::chrono::steady_clock::time_point Now() const override;
};

}  // namespace repulsor3d
