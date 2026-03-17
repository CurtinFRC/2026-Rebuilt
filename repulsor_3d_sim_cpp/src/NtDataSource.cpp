#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/NtDataSource.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/Topic.h>

namespace repulsor3d {
namespace {

std::string TrimSlashes(std::string p) {
  while (!p.empty() && p.front() == '/') {
    p.erase(p.begin());
  }
  while (!p.empty() && p.back() == '/') {
    p.pop_back();
  }
  return p;
}

std::string TopicPrefix(const std::string& tablePath) {
  const std::string p = TrimSlashes(tablePath);
  return p.empty() ? "/" : "/" + p;
}

std::string JoinTopic(const std::string& prefix, const std::string& leaf) {
  std::string p = prefix;
  while (!p.empty() && p.back() == '/') {
    p.pop_back();
  }
  std::string l = leaf;
  while (!l.empty() && l.front() == '/') {
    l.erase(l.begin());
  }
  if (p.empty()) {
    return "/" + l;
  }
  return p + "/" + l;
}

std::optional<std::string> ExtractIdFromTopic(const std::string& topicName, const std::string& tablePrefix,
                                              const std::string& idPrefix) {
  std::string base = tablePrefix;
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }

  if (topicName.rfind(base + "/", 0) != 0) {
    return std::nullopt;
  }

  const std::string rel = topicName.substr(base.size() + 1);
  const auto slash = rel.find('/');
  const std::string head = (slash == std::string::npos) ? rel : rel.substr(0, slash);
  if (head.rfind(idPrefix, 0) != 0) {
    return std::nullopt;
  }
  return head.substr(idPrefix.size());
}

bool IsFinite(const double v) {
  return std::isfinite(v) != 0;
}

}  // namespace

struct NtDataSource::FvSubs {
  nt::BooleanSubscriber alive;
  nt::StringSubscriber type;
  nt::DoubleSubscriber x;
  nt::DoubleSubscriber y;
  nt::DoubleSubscriber z;
  nt::DoubleSubscriber roll;
  nt::DoubleSubscriber pitch;
  nt::DoubleSubscriber yaw;

  FvSubs(nt::BooleanSubscriber aliveIn, nt::StringSubscriber typeIn, nt::DoubleSubscriber xIn, nt::DoubleSubscriber yIn,
         nt::DoubleSubscriber zIn, nt::DoubleSubscriber rollIn, nt::DoubleSubscriber pitchIn, nt::DoubleSubscriber yawIn)
      : alive(std::move(aliveIn)),
        type(std::move(typeIn)),
        x(std::move(xIn)),
        y(std::move(yIn)),
        z(std::move(zIn)),
        roll(std::move(rollIn)),
        pitch(std::move(pitchIn)),
        yaw(std::move(yawIn)) {}
};

struct NtDataSource::RvSubs {
  nt::BooleanSubscriber alive;
  nt::StringSubscriber kind;
  nt::DoubleSubscriber x;
  nt::DoubleSubscriber y;
  nt::DoubleSubscriber sx;
  nt::DoubleSubscriber sy;

  RvSubs(nt::BooleanSubscriber aliveIn, nt::StringSubscriber kindIn, nt::DoubleSubscriber xIn, nt::DoubleSubscriber yIn,
         nt::DoubleSubscriber sxIn, nt::DoubleSubscriber syIn)
      : alive(std::move(aliveIn)),
        kind(std::move(kindIn)),
        x(std::move(xIn)),
        y(std::move(yIn)),
        sx(std::move(sxIn)),
        sy(std::move(syIn)) {}
};

struct NtDataSource::CamSubs {
  nt::BooleanSubscriber alive;
  nt::DoubleSubscriber x;
  nt::DoubleSubscriber y;
  nt::DoubleSubscriber z;
  nt::DoubleSubscriber yawDeg;
  nt::DoubleSubscriber pitchDeg;
  nt::DoubleSubscriber rollDeg;
  nt::DoubleSubscriber hfovDeg;
  nt::DoubleSubscriber vfovDeg;
  nt::DoubleSubscriber maxRange;

  CamSubs(nt::BooleanSubscriber aliveIn, nt::DoubleSubscriber xIn, nt::DoubleSubscriber yIn, nt::DoubleSubscriber zIn,
          nt::DoubleSubscriber yawDegIn, nt::DoubleSubscriber pitchDegIn, nt::DoubleSubscriber rollDegIn,
          nt::DoubleSubscriber hfovDegIn, nt::DoubleSubscriber vfovDegIn, nt::DoubleSubscriber maxRangeIn)
      : alive(std::move(aliveIn)),
        x(std::move(xIn)),
        y(std::move(yIn)),
        z(std::move(zIn)),
        yawDeg(std::move(yawDegIn)),
        pitchDeg(std::move(pitchDegIn)),
        rollDeg(std::move(rollDegIn)),
        hfovDeg(std::move(hfovDegIn)),
        vfovDeg(std::move(vfovDegIn)),
        maxRange(std::move(maxRangeIn)) {}
};

NtDataSource::NtDataSource(const ViewerConfig& cfg) : cfg_(cfg) {
  inst_ = std::make_unique<nt::NetworkTableInstance>(nt::NetworkTableInstance::GetDefault());
  inst_->StopClient();
  inst_->StartClient4(cfg.ntClientName);
  inst_->SetServerTeam(4788);
  inst_->SetServer(cfg.ntServer, nt::NetworkTableInstance::kDefaultPort4);

  const std::string poseRoot = JoinTopic(TopicPrefix(cfg.poseBasePath), cfg.poseStructKey);

  poseX_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(poseRoot, "translation/x")).Subscribe(0.0));
  poseY_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(poseRoot, "translation/y")).Subscribe(0.0));
  poseTheta_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(poseRoot, "rotation/value")).Subscribe(0.0));

  activeX_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/ActiveGoal/translation/x").Subscribe(0.0));
  activeY_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/ActiveGoal/translation/y").Subscribe(0.0));
  activeTheta_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/ActiveGoal/rotation/value").Subscribe(0.0));

  chosenX_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/Repulsor/ChosenCollect/translation/x").Subscribe(0.0));
  chosenY_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/Repulsor/ChosenCollect/translation/y").Subscribe(0.0));
  chosenTheta_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/Repulsor/ChosenCollect/rotation/value").Subscribe(0.0));

  finalX_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/FinalCollect/translation/x").Subscribe(0.0));
  finalY_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/FinalCollect/translation/y").Subscribe(0.0));
  finalTheta_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/AdvantageKit/RealOutputs/FinalCollect/rotation/value").Subscribe(0.0));

  const std::string fvPrefix = TopicPrefix(cfg.fieldVisionPath);
  ex_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/x")).Subscribe(0.0));
  ey_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/y")).Subscribe(0.0));
  ez_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/z")).Subscribe(0.0));
  er_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/roll")).Subscribe(0.0));
  ep_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/pitch")).Subscribe(0.0));
  eyaw_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic(JoinTopic(fvPrefix, "extrinsics/yaw")).Subscribe(0.0));

  pieces_ = std::make_unique<nt::DoubleSubscriber>(inst_->GetDoubleTopic("/PieceCount").Subscribe(0.0));
  method_ = std::make_unique<nt::StringSubscriber>(inst_->GetStringTopic("/AdvantageKit/RealOutputs/Method").Subscribe("N/A"));
}

NtDataSource::~NtDataSource() = default;

SnapshotBundle NtDataSource::Read() {
  DiscoverDynamicTopics();

  SnapshotBundle out;
  out.connected = inst_->IsConnected();
  out.pieces = static_cast<int>(pieces_->Get());
  out.method = method_->Get();

  out.snapshot.pose = ReadPoseSubscriber(poseX_.get(), poseY_.get(), poseTheta_.get());
  out.snapshot.activeGoal = ReadPoseSubscriber(activeX_.get(), activeY_.get(), activeTheta_.get());
  out.snapshot.chosenCollect = ReadPoseSubscriber(chosenX_.get(), chosenY_.get(), chosenTheta_.get());
  out.snapshot.finalCollect = ReadPoseSubscriber(finalX_.get(), finalY_.get(), finalTheta_.get());

  out.snapshot.extrinsics = {
      ex_->Get(),
      ey_->Get(),
      ez_->Get(),
      er_->Get(),
      ep_->Get(),
      eyaw_->Get(),
  };

  for (const auto& [id, subs] : fvSubs_) {
    if (!subs->alive.Get()) {
      continue;
    }

    FieldVisionObject obj;
    obj.oid = id;
    obj.type = subs->type.Get();
    obj.x = subs->x.Get();
    obj.y = subs->y.Get();
    obj.z = subs->z.Get();
    obj.roll = subs->roll.Get();
    obj.pitch = subs->pitch.Get();
    obj.yaw = subs->yaw.Get();
    out.snapshot.fieldVision.push_back(std::move(obj));
  }

  for (const auto& [id, subs] : rvSubs_) {
    if (!subs->alive.Get()) {
      continue;
    }

    RepulsorVisionObstacle obs;
    obs.oid = id;
    obs.kind = subs->kind.Get();
    obs.x = subs->x.Get();
    obs.y = subs->y.Get();
    obs.sizeX = subs->sx.Get();
    obs.sizeY = subs->sy.Get();
    out.snapshot.repulsorVision.push_back(std::move(obs));
  }

  for (const auto& [name, subs] : camSubs_) {
    if (!subs->alive.Get()) {
      continue;
    }

    CameraInfo cam;
    cam.name = name;
    cam.x = subs->x.Get();
    cam.y = subs->y.Get();
    cam.z = subs->z.Get();
    cam.yawDeg = subs->yawDeg.Get();
    cam.pitchDeg = subs->pitchDeg.Get();
    cam.rollDeg = subs->rollDeg.Get();
    cam.hfovDeg = subs->hfovDeg.Get();
    cam.vfovDeg = subs->vfovDeg.Get();
    cam.maxRange = subs->maxRange.Get();
    out.snapshot.cameras.push_back(std::move(cam));
  }

  return out;
}

void NtDataSource::DiscoverDynamicTopics() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const double nowS = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
  if (nowS - lastDiscoveryS_ < discoveryPeriodS_) {
    return;
  }
  lastDiscoveryS_ = nowS;

  const std::string fvPrefix = TopicPrefix(cfg_.fieldVisionPath);
  const std::string rvPrefix = TopicPrefix(cfg_.repulsorVisionPath);

  const auto fvTopics = inst_->GetTopics(fvPrefix);
  const auto rvTopics = inst_->GetTopics(rvPrefix);
  const auto camTopics = inst_->GetTopics(fvPrefix);

  std::vector<std::string> fvIds;
  std::vector<std::string> rvIds;
  std::vector<std::string> camIds;

  for (const auto& t : fvTopics) {
    const auto id = ExtractIdFromTopic(t.GetName(), fvPrefix, "object_");
    if (id.has_value()) {
      fvIds.push_back(*id);
    }
  }

  for (const auto& t : rvTopics) {
    const auto id = ExtractIdFromTopic(t.GetName(), rvPrefix, "obs_");
    if (id.has_value()) {
      rvIds.push_back(*id);
    }
  }

  for (const auto& t : camTopics) {
    const auto id = ExtractIdFromTopic(t.GetName(), fvPrefix, "camera_");
    if (id.has_value()) {
      camIds.push_back(*id);
    }
  }

  std::sort(fvIds.begin(), fvIds.end());
  fvIds.erase(std::unique(fvIds.begin(), fvIds.end()), fvIds.end());
  std::sort(rvIds.begin(), rvIds.end());
  rvIds.erase(std::unique(rvIds.begin(), rvIds.end()), rvIds.end());
  std::sort(camIds.begin(), camIds.end());
  camIds.erase(std::unique(camIds.begin(), camIds.end()), camIds.end());

  for (const auto& id : fvIds) {
    if (fvSubs_.contains(id)) {
      continue;
    }

    const std::string base = "object_" + id;
    const std::string baseTopic = JoinTopic(fvPrefix, base);
    fvSubs_[id] = std::make_unique<FvSubs>(
        inst_->GetBooleanTopic(baseTopic).Subscribe(false),
        inst_->GetStringTopic(JoinTopic(fvPrefix, base + "/type")).Subscribe(""),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/x")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/y")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/z")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/roll")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/pitch")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/yaw")).Subscribe(0.0));
  }

  for (const auto& id : rvIds) {
    if (rvSubs_.contains(id)) {
      continue;
    }

    const std::string base = "obs_" + id;
    const std::string baseTopic = JoinTopic(rvPrefix, base);
    rvSubs_[id] = std::make_unique<RvSubs>(
        inst_->GetBooleanTopic(baseTopic).Subscribe(false),
        inst_->GetStringTopic(JoinTopic(rvPrefix, base + "/kind")).Subscribe(""),
        inst_->GetDoubleTopic(JoinTopic(rvPrefix, base + "/x")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(rvPrefix, base + "/y")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(rvPrefix, base + "/size_x")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(rvPrefix, base + "/size_y")).Subscribe(0.0));
  }

  for (const auto& id : camIds) {
    if (camSubs_.contains(id)) {
      continue;
    }

    const std::string base = "camera_" + id;
    const std::string baseTopic = JoinTopic(fvPrefix, base);
    camSubs_[id] = std::make_unique<CamSubs>(
        inst_->GetBooleanTopic(baseTopic).Subscribe(false),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/x")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/y")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/z")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/yaw_deg")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/pitch_deg")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/roll_deg")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/hfov_deg")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/vfov_deg")).Subscribe(0.0),
        inst_->GetDoubleTopic(JoinTopic(fvPrefix, base + "/max_range")).Subscribe(0.0));
  }
}

std::optional<Pose2D> NtDataSource::ReadPose() const {
  return ReadPoseSubscriber(poseX_.get(), poseY_.get(), poseTheta_.get());
}

std::optional<Pose2D> NtDataSource::ReadPoseSubscriber(const nt::DoubleSubscriber* xSub, const nt::DoubleSubscriber* ySub,
                                                       const nt::DoubleSubscriber* thetaSub) const {
  if (xSub == nullptr || ySub == nullptr || thetaSub == nullptr) {
    return std::nullopt;
  }

  const double x = xSub->Get();
  const double y = ySub->Get();
  const double th = thetaSub->Get();

  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(th)) {
    return std::nullopt;
  }

  return Pose2D{x, y, th};
}

}  // namespace repulsor3d

#endif  // defined(REPULSOR_HAS_NTCORE)
