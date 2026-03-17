#include "repulsor3d/time/TimeSource.hpp"

namespace repulsor3d {

std::chrono::steady_clock::time_point SteadyTimeSource::Now() const {
  return std::chrono::steady_clock::now();
}

}  // namespace repulsor3d
