#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/NtDataSource.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <networktables/NetworkTableInstance.h>

#include "repulsor3d/nt/DefaultSchemas.hpp"
#include "repulsor3d/nt/DomainMappers.hpp"
#include "repulsor3d/nt/EntityStream.hpp"
#include "repulsor3d/nt/SubscriberCollection.hpp"
#include "repulsor3d/nt/TopicPath.hpp"

namespace repulsor3d {
namespace {

bool IsFinite(const double value) {
  return std::isfinite(value) != 0;
}

std::optional<Pose2D> ReadPose2D(const nt::SubscriberCollection& subs) {
  if (!subs.ContainsKey("x") || !subs.ContainsKey("y") || !subs.ContainsKey("theta")) {
    return std::nullopt;
  }
  const double x = subs.GetDouble("x");
  const double y = subs.GetDouble("y");
  const double theta = subs.GetDouble("theta");
  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(theta)) {
    return std::nullopt;
  }
  return Pose2D{x, y, theta};
}

}  // namespace

class NtDataSource::Impl {
 public:
  explicit Impl(const ViewerConfig& cfg)
      : cfg_(cfg),
        inst_(std::make_unique<::nt::NetworkTableInstance>(::nt::NetworkTableInstance::GetDefault())),
        pose_(inst_.get()),
        activeGoal_(inst_.get()),
        chosenCollect_(inst_.get()),
        finalCollect_(inst_.get()),
        extrinsics_(inst_.get()),
        scalars_(inst_.get()),
        fieldVisionStream_(inst_.get(), nt::MakeFieldVisionObjectSchema(cfg_), nt::TryMapFieldVisionObject),
        repulsorStream_(inst_.get(), nt::MakeRepulsorObstacleSchema(cfg_), nt::TryMapRepulsorObstacle),
        cameraStream_(inst_.get(), nt::MakeCameraSchema(cfg_), nt::TryMapCameraInfo) {
    inst_->StopClient();
    inst_->StartClient4(cfg_.ntClientName);
    inst_->SetServerTeam(4788);
    inst_->SetServer(cfg_.ntServer, ::nt::NetworkTableInstance::kDefaultPort4);

    BindPoseSubscriptions();
    BindStaticSubscriptions();
  }

  SnapshotBundle Read() {
    DiscoverDynamicTopics();

    SnapshotBundle out;
    out.connected = inst_->IsConnected();
    out.pieces = static_cast<int>(scalars_.GetDouble("pieces", 0.0));
    out.method = scalars_.GetString("method", "N/A");

    out.snapshot.pose = ReadPose2D(pose_);
    out.snapshot.activeGoal = ReadPose2D(activeGoal_);
    out.snapshot.chosenCollect = ReadPose2D(chosenCollect_);
    out.snapshot.finalCollect = ReadPose2D(finalCollect_);

    out.snapshot.extrinsics = {
        extrinsics_.GetDouble("x"),
        extrinsics_.GetDouble("y"),
        extrinsics_.GetDouble("z"),
        extrinsics_.GetDouble("roll"),
        extrinsics_.GetDouble("pitch"),
        extrinsics_.GetDouble("yaw"),
    };

    fieldVisionStream_.AppendTo(out.snapshot.fieldVision);
    repulsorStream_.AppendTo(out.snapshot.repulsorVision);
    cameraStream_.AppendTo(out.snapshot.cameras);
    return out;
  }

 private:
  void BindPoseSubscriptions() {
    const nt::TopicPathBuilder poseBase(cfg_.poseBasePath);
    const std::string poseRoot = nt::JoinTopic(poseBase.Prefix(), cfg_.poseStructKey);

    BindPose(pose_, poseRoot);
    BindPose(activeGoal_, "/AdvantageKit/RealOutputs/ActiveGoal");
    BindPose(chosenCollect_, "/AdvantageKit/RealOutputs/Repulsor/ChosenCollect");
    BindPose(finalCollect_, "/AdvantageKit/RealOutputs/FinalCollect");
  }

  void BindPose(nt::SubscriberCollection& group, const std::string& rootTopic) {
    group.AddDouble("x", nt::JoinTopic(rootTopic, "translation/x"));
    group.AddDouble("y", nt::JoinTopic(rootTopic, "translation/y"));
    group.AddDouble("theta", nt::JoinTopic(rootTopic, "rotation/value"));
  }

  void BindStaticSubscriptions() {
    const nt::TopicPathBuilder fieldVision(cfg_.fieldVisionPath);
    extrinsics_.AddDouble("x", fieldVision.At("extrinsics/x"));
    extrinsics_.AddDouble("y", fieldVision.At("extrinsics/y"));
    extrinsics_.AddDouble("z", fieldVision.At("extrinsics/z"));
    extrinsics_.AddDouble("roll", fieldVision.At("extrinsics/roll"));
    extrinsics_.AddDouble("pitch", fieldVision.At("extrinsics/pitch"));
    extrinsics_.AddDouble("yaw", fieldVision.At("extrinsics/yaw"));

    scalars_.AddDouble("pieces", "/PieceCount");
    scalars_.AddString("method", "/AdvantageKit/RealOutputs/Method", "N/A");
  }

  void DiscoverDynamicTopics() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const double nowS = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
    if (nowS - lastDiscoveryS_ < discoveryPeriodS_) {
      return;
    }
    lastDiscoveryS_ = nowS;

    fieldVisionStream_.Discover();
    repulsorStream_.Discover();
    cameraStream_.Discover();
  }

  ViewerConfig cfg_;
  std::unique_ptr<::nt::NetworkTableInstance> inst_;

  nt::SubscriberCollection pose_;
  nt::SubscriberCollection activeGoal_;
  nt::SubscriberCollection chosenCollect_;
  nt::SubscriberCollection finalCollect_;
  nt::SubscriberCollection extrinsics_;
  nt::SubscriberCollection scalars_;

  nt::EntityStream<FieldVisionObject> fieldVisionStream_;
  nt::EntityStream<RepulsorVisionObstacle> repulsorStream_;
  nt::EntityStream<CameraInfo> cameraStream_;

  double lastDiscoveryS_ = 0.0;
  double discoveryPeriodS_ = 0.25;
};

NtDataSource::NtDataSource(const ViewerConfig& cfg) : impl_(std::make_unique<Impl>(cfg)) {}

NtDataSource::~NtDataSource() = default;

SnapshotBundle NtDataSource::Read() {
  return impl_->Read();
}

}  // namespace repulsor3d

#endif  // defined(REPULSOR_HAS_NTCORE)
