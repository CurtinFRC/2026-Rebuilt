#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/NtDataSource.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <networktables/NetworkTableInstance.h>

#include "repulsor3d/nt/DefaultSchemas.hpp"
#include "repulsor3d/nt/DomainMappers.hpp"
#include "repulsor3d/nt/PoseBinding.hpp"
#include "repulsor3d/nt/SchemaFileLoader.hpp"
#include "repulsor3d/nt/SnapshotAppender.hpp"
#include "repulsor3d/nt/SubscriberCollection.hpp"
#include "repulsor3d/nt/TopicPath.hpp"

namespace repulsor3d {

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
        schemaSet_(nt::MakeDefaultSchemaSet(cfg_)) {
    inst_->StopClient();
    inst_->StartClient4(cfg_.ntClientName);
    inst_->SetServerTeam(4788);
    inst_->SetServer(cfg_.ntServer, ::nt::NetworkTableInstance::kDefaultPort4);

    BindPoseSubscriptions();
    BindStaticSubscriptions();
    TryLoadSchemaSetFromFile(/*forceRebuild=*/false);
    BuildEntityAppenders();
  }

  SnapshotBundle Read() {
    DiscoverDynamicTopics();

    SnapshotBundle out;
    out.connected = inst_->IsConnected();
    out.pieces = static_cast<int>(scalars_.GetDouble("pieces", 0.0));
    out.method = scalars_.GetString("method", "N/A");

    out.snapshot.pose = pose_.ReadPose2D();
    out.snapshot.activeGoal = activeGoal_.ReadPose2D();
    out.snapshot.chosenCollect = chosenCollect_.ReadPose2D();
    out.snapshot.finalCollect = finalCollect_.ReadPose2D();

    out.snapshot.extrinsics = {
        extrinsics_.GetDouble("x"),
        extrinsics_.GetDouble("y"),
        extrinsics_.GetDouble("z"),
        extrinsics_.GetDouble("roll"),
        extrinsics_.GetDouble("pitch"),
        extrinsics_.GetDouble("yaw"),
    };

    for (const auto& appender : appenders_) {
      appender->Append(out.snapshot);
    }
    return out;
  }

 private:
  template <typename TObject>
  void AddEntityAppender(
      nt::EntityGroupSchema schema,
      typename nt::EntityStream<TObject>::Mapper mapper,
      std::vector<TObject> WorldSnapshot::*target) {
    appenders_.push_back(
        std::make_unique<nt::VectorChannelAppender<TObject>>(inst_.get(), std::move(schema), std::move(mapper), target));
  }

  void BindPoseSubscriptions() {
    const nt::TopicPathBuilder poseBase(cfg_.poseBasePath);
    const std::string poseRoot = nt::JoinTopic(poseBase.Prefix(), cfg_.poseStructKey);

    pose_.BindPose2D(poseRoot);
    activeGoal_.BindPose2D("/AdvantageKit/RealOutputs/ActiveGoal");
    chosenCollect_.BindPose2D("/AdvantageKit/RealOutputs/Repulsor/ChosenCollect");
    finalCollect_.BindPose2D("/AdvantageKit/RealOutputs/FinalCollect");
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

  void BuildEntityAppenders() {
    appenders_.clear();
    AddEntityAppender<FieldVisionObject>(
        schemaSet_.fieldVision, nt::TryMapFieldVisionObject, &WorldSnapshot::fieldVision);
    AddEntityAppender<RepulsorVisionObstacle>(
        schemaSet_.repulsor, nt::TryMapRepulsorObstacle, &WorldSnapshot::repulsorVision);
    AddEntityAppender<CameraInfo>(schemaSet_.cameras, nt::TryMapCameraInfo, &WorldSnapshot::cameras);

    for (const auto& dynamicChannel : schemaSet_.dynamicChannels) {
      appenders_.push_back(std::make_unique<nt::NamedVectorMapChannelAppender<DynamicEntityRecord>>(
          inst_.get(),
          dynamicChannel.channel,
          dynamicChannel.schema,
          nt::TryMapDynamicEntityRecord,
          &WorldSnapshot::dynamicEntityGroups));
    }
  }

  void TryLoadSchemaSetFromFile(const bool forceRebuild) {
    if (cfg_.ntSchemaPath.empty()) {
      return;
    }

    nt::NtSchemaSet candidate = nt::MakeDefaultSchemaSet(cfg_);
    std::string error;
    if (!nt::LoadSchemaSetFromFile(cfg_.ntSchemaPath, candidate, &error)) {
      if (forceRebuild) {
        std::cerr << "[NtDataSource] failed to load schema file '" << cfg_.ntSchemaPath << "': " << error << "\n";
      }
      return;
    }

    schemaSet_ = std::move(candidate);
    schemaLoadedFromFile_ = true;
    if (forceRebuild) {
      BuildEntityAppenders();
    }
  }

  void MaybeHotReloadSchemas(const double nowS) {
    if (cfg_.ntSchemaPath.empty()) {
      return;
    }
    if (!cfg_.hotReloadNtSchema && schemaLoadedFromFile_) {
      return;
    }
    if (nowS - lastSchemaCheckS_ < schemaCheckPeriodS_) {
      return;
    }
    lastSchemaCheckS_ = nowS;

    std::error_code ec;
    const auto stamp = std::filesystem::last_write_time(cfg_.ntSchemaPath, ec);
    if (ec) {
      return;
    }
    if (!schemaWriteTimeValid_ || stamp != schemaWriteTime_) {
      schemaWriteTime_ = stamp;
      schemaWriteTimeValid_ = true;
      TryLoadSchemaSetFromFile(/*forceRebuild=*/true);
    }
  }

  void DiscoverDynamicTopics() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const double nowS = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
    if (nowS - lastDiscoveryS_ < discoveryPeriodS_) {
      return;
    }
    lastDiscoveryS_ = nowS;

    MaybeHotReloadSchemas(nowS);

    for (const auto& appender : appenders_) {
      appender->Discover();
    }
  }

  ViewerConfig cfg_;
  std::unique_ptr<::nt::NetworkTableInstance> inst_;

  nt::PoseBinding pose_;
  nt::PoseBinding activeGoal_;
  nt::PoseBinding chosenCollect_;
  nt::PoseBinding finalCollect_;
  nt::SubscriberCollection extrinsics_;
  nt::SubscriberCollection scalars_;
  nt::NtSchemaSet schemaSet_;

  std::vector<std::unique_ptr<nt::IWorldSnapshotAppender>> appenders_;
  bool schemaLoadedFromFile_ = false;
  bool schemaWriteTimeValid_ = false;
  std::filesystem::file_time_type schemaWriteTime_{};

  double lastDiscoveryS_ = 0.0;
  double discoveryPeriodS_ = 0.25;
  double lastSchemaCheckS_ = 0.0;
  double schemaCheckPeriodS_ = 0.5;
};

NtDataSource::NtDataSource(const ViewerConfig& cfg) : impl_(std::make_unique<Impl>(cfg)) {}

NtDataSource::~NtDataSource() = default;

SnapshotBundle NtDataSource::Read() {
  return impl_->Read();
}

}  // namespace repulsor3d

#endif  // defined(REPULSOR_HAS_NTCORE)
