#pragma once

#include "repulsor3d/DataSource.hpp"

#if defined(REPULSOR_HAS_NTCORE)

#include <memory>
#include <string>
#include <unordered_map>

#include "repulsor3d/Config.hpp"

namespace nt {
class NetworkTableInstance;
class DoubleSubscriber;
class BooleanSubscriber;
class StringSubscriber;
}  // namespace nt

namespace repulsor3d {

class NtDataSource final : public ISnapshotSource {
 public:
  explicit NtDataSource(const ViewerConfig& cfg);
  ~NtDataSource() override;

  SnapshotBundle Read() override;

 private:
  struct FvSubs;
  struct RvSubs;
  struct CamSubs;

  void DiscoverDynamicTopics();
  std::optional<Pose2D> ReadPose() const;
  std::optional<Pose2D> ReadPoseSubscriber(const nt::DoubleSubscriber* xSub, const nt::DoubleSubscriber* ySub,
                                           const nt::DoubleSubscriber* thetaSub) const;

  ViewerConfig cfg_;

  std::unique_ptr<nt::NetworkTableInstance> inst_;

  std::unordered_map<std::string, std::unique_ptr<FvSubs>> fvSubs_;
  std::unordered_map<std::string, std::unique_ptr<RvSubs>> rvSubs_;
  std::unordered_map<std::string, std::unique_ptr<CamSubs>> camSubs_;

  std::unique_ptr<nt::DoubleSubscriber> poseX_;
  std::unique_ptr<nt::DoubleSubscriber> poseY_;
  std::unique_ptr<nt::DoubleSubscriber> poseTheta_;

  std::unique_ptr<nt::DoubleSubscriber> activeX_;
  std::unique_ptr<nt::DoubleSubscriber> activeY_;
  std::unique_ptr<nt::DoubleSubscriber> activeTheta_;

  std::unique_ptr<nt::DoubleSubscriber> chosenX_;
  std::unique_ptr<nt::DoubleSubscriber> chosenY_;
  std::unique_ptr<nt::DoubleSubscriber> chosenTheta_;

  std::unique_ptr<nt::DoubleSubscriber> finalX_;
  std::unique_ptr<nt::DoubleSubscriber> finalY_;
  std::unique_ptr<nt::DoubleSubscriber> finalTheta_;

  std::unique_ptr<nt::DoubleSubscriber> ex_;
  std::unique_ptr<nt::DoubleSubscriber> ey_;
  std::unique_ptr<nt::DoubleSubscriber> ez_;
  std::unique_ptr<nt::DoubleSubscriber> er_;
  std::unique_ptr<nt::DoubleSubscriber> ep_;
  std::unique_ptr<nt::DoubleSubscriber> eyaw_;

  std::unique_ptr<nt::DoubleSubscriber> pieces_;
  std::unique_ptr<nt::StringSubscriber> method_;

  double lastDiscoveryS_ = 0.0;
  double discoveryPeriodS_ = 0.25;
};

}  // namespace repulsor3d

#endif  // defined(REPULSOR_HAS_NTCORE)
