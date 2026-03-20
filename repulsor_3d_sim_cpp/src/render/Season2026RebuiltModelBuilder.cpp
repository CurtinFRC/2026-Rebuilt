#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/scenegraph/SceneGraphBuilder.hpp"
#include "repulsor3d/render/MeshCulling.hpp"

namespace repulsor3d {
namespace {

RenderEntity MakeStaticEntity(
    std::string id,
    const RenderPass pass,
    RenderEntityPayload payload,
    const EntityCulling culling = {.enabled = true, .boundsRadius = 0.5F}) {
  return RenderEntity{
      .id = std::move(id),
      .pass = pass,
      .payload = std::move(payload),
      .parentId = "",
      .transform = {},
      .hasTransform = false,
      .culling = culling,
  };
}

}  // namespace

Season2026RebuiltModelBuilder::Season2026RebuiltModelBuilder(const ViewerConfig& cfg) : cfg_(cfg) {
  fieldZ_ = cfg.fieldZM;
  fieldLength_ = cfg.fieldLengthM;
  fieldWidth_ = cfg.fieldWidthM;
  fuelRadius_ = cfg.ballRadiusM;
  obsSide_ = cfg.obsBoxSideM;
  robotL_ = cfg.robotBoxLM;
  robotW_ = cfg.robotBoxWM;
  robotH_ = cfg.robotBoxHM;

  showRobotCadModel_ = cfg.showRobotCadModel;
  robotCadModelPath_ = cfg.robotCadModelPath;
  robotCadScaleM_ = cfg.robotCadScaleM;
  robotCadZOffsetM_ = cfg.robotCadZOffsetM;

  showFieldCadModel_ = cfg.showFieldCadModel;
  fieldCadModelPath_ = cfg.fieldCadModelPath;
  fieldCadScaleM_ = cfg.fieldCadScaleM;
  fieldCadFlipX_ = cfg.fieldCadFlipX;
  fieldCadZOffsetM_ = cfg.fieldCadZOffsetM;
  fieldCadOffsetXM_ = cfg.fieldCadOffsetXM;
  fieldCadOffsetYM_ = cfg.fieldCadOffsetYM;
  maxRenderFuel_ = std::max(0, cfg.maxRenderFuel);
  maxRenderTruthFuel_ = std::max(0, cfg.maxRenderTruthFuel);
  maxCameraDebugRaysPerCamera_ = std::max(0, cfg.maxCameraDebugRaysPerCamera);
}

RenderSceneFrame Season2026RebuiltModelBuilder::BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) {
  RenderSceneFrame frame;
  scenegraph::SceneGraphBuilder sceneGraph;
  // frame.drawFieldImage = toggles.showFieldImage;
  frame.drawFieldImage = false;

  frame.overlayLines.push_back({std::string("[C] Camera debug: ") + (toggles.showCameraDebug ? "ON" : "OFF")});
  frame.overlayLines.push_back({std::string("[T] Truth fuel: ") + (toggles.showTruthFuel ? "ON" : "OFF")});
  frame.overlayLines.push_back({std::string("[A] Age filter: ") + (toggles.showAgeFilteredFuel ? "ON" : "OFF")});
  frame.overlayLines.push_back({"Field image: DISABLED (CAD field model in use)"});
  frame.overlayLines.push_back(
      {std::string("CAD robot: ") + (showRobotCadModel_ && !robotCadModelPath_.empty() ? "ON" : "OFF")});
  frame.overlayLines.push_back(
      {std::string("CAD field: ") + (showFieldCadModel_ && !fieldCadModelPath_.empty() ? "ON" : "OFF")});
  frame.overlayLines.push_back({"Pieces: " + std::to_string(bundle.pieces)});
  frame.overlayLines.push_back({"Method: " + bundle.method});

  const WorldSnapshot& snap = bundle.snapshot;
  if (toggles.showTruthFuel) {
    AppendTruthFuelPrimitives(sceneGraph, snap);
  }

  AppendFuelPrimitives(sceneGraph, snap, toggles.showAgeFilteredFuel);
  AppendObstaclePrimitives(sceneGraph, snap);
  AppendRobotPrimitives(sceneGraph, snap);
  AppendCadModelPrimitives(sceneGraph, snap);
  if (toggles.showCameraDebug) {
    AppendCameraPrimitives(sceneGraph, snap);
  }
  frame.entities = sceneGraph.ConsumeNodes();

  return frame;
}

void Season2026RebuiltModelBuilder::AppendFuelPrimitives(
    scenegraph::SceneGraphBuilder& sceneGraph,
    const WorldSnapshot& snap,
    const bool showAgeFilteredFuel) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const double nowS = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();

  for (const auto& o : snap.fieldVision) {
    if (NormalizeType(o.type) != "fuel") {
      continue;
    }
    fuelCache_[o.oid] = o;
    fuelLastSeen_[o.oid] = nowS;
  }

  if (fuelCache_.size() > 1500) {
    const double ttl = std::max(1.5, static_cast<double>(cfg_.resourceHardMaxAgeS * 3.0F));
    std::vector<std::string> eraseKeys;
    eraseKeys.reserve(fuelLastSeen_.size());

    for (const auto& [oid, seen] : fuelLastSeen_) {
      if (nowS - seen > ttl) {
        eraseKeys.push_back(oid);
      }
    }

    for (const auto& k : eraseKeys) {
      fuelLastSeen_.erase(k);
      fuelCache_.erase(k);
    }
  }

  if (showAgeFilteredFuel) {
    const std::size_t maxCount = static_cast<std::size_t>(std::max(0, maxRenderFuel_));
    const std::size_t total = fuelCache_.size();
    const std::size_t step = (maxCount > 0 && total > maxCount) ? ((total + maxCount - 1) / maxCount) : 1;
    std::size_t sampleIndex = 0;
    for (const auto& [oid, o] : fuelCache_) {
      if (step > 1 && (sampleIndex % step) != 0) {
        ++sampleIndex;
        continue;
      }
      ++sampleIndex;
      const double age = nowS - fuelLastSeen_[oid];
      if (age > static_cast<double>(cfg_.resourceHardMaxAgeS)) {
        continue;
      }

      const float w = std::exp(-cfg_.collectAgeDecay * static_cast<float>(std::max(0.0, age)));
      const float scale = 0.35F + 0.65F * w;
      SpherePrimitive sphere{
          .center = glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
          .radius = fuelRadius_,
          .color = colFuel_ * scale,
          .pass = RenderPass::Opaque,
      };
      sceneGraph.AddNode(MakeStaticEntity(
          "fuel_cached_" + oid,
          RenderPass::Opaque,
          sphere,
          {.enabled = true, .boundsRadius = std::max(fuelRadius_, 0.05F)}));
    }
    return;
  }

  const std::size_t maxCount = static_cast<std::size_t>(std::max(0, maxRenderFuel_));
  const std::size_t total = snap.fieldVision.size();
  const std::size_t step = (maxCount > 0 && total > maxCount) ? ((total + maxCount - 1) / maxCount) : 1;
  std::size_t sampleIndex = 0;
  for (const auto& o : snap.fieldVision) {
    if (step > 1 && (sampleIndex % step) != 0) {
      ++sampleIndex;
      continue;
    }
    ++sampleIndex;
    if (NormalizeType(o.type) != "fuel") {
      continue;
    }

    SpherePrimitive sphere{
        .center = glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
        .radius = fuelRadius_,
        .color = colFuel_,
        .pass = RenderPass::Opaque,
    };
    const std::string id = o.oid.empty() ? ("fuel_live_" + std::to_string(sampleIndex)) : ("fuel_live_" + o.oid);
    sceneGraph.AddNode(MakeStaticEntity(
        id,
        RenderPass::Opaque,
        sphere,
        {.enabled = true, .boundsRadius = std::max(fuelRadius_, 0.05F)}));
  }
}

void Season2026RebuiltModelBuilder::AppendTruthFuelPrimitives(scenegraph::SceneGraphBuilder& sceneGraph, const WorldSnapshot& snap) {
  const std::size_t maxCount = static_cast<std::size_t>(std::max(0, maxRenderTruthFuel_));
  const std::size_t total = snap.truth.size();
  const std::size_t step = (maxCount > 0 && total > maxCount) ? ((total + maxCount - 1) / maxCount) : 1;
  std::size_t sampleIndex = 0;
  for (const auto& o : snap.truth) {
    if (step > 1 && (sampleIndex % step) != 0) {
      ++sampleIndex;
      continue;
    }
    ++sampleIndex;
    SpherePrimitive sphere{
        .center = glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
        .radius = fuelRadius_ * 0.85F,
        .color = colTruthFuel_,
        .pass = RenderPass::Opaque,
    };
    const std::string id = o.oid.empty() ? ("fuel_truth_" + std::to_string(sampleIndex)) : ("fuel_truth_" + o.oid);
    sceneGraph.AddNode(MakeStaticEntity(
        id,
        RenderPass::Opaque,
        sphere,
        {.enabled = true, .boundsRadius = std::max(fuelRadius_ * 0.85F, 0.05F)}));
  }
}

void Season2026RebuiltModelBuilder::AppendObstaclePrimitives(scenegraph::SceneGraphBuilder& sceneGraph, const WorldSnapshot& snap) const {
  const float hz = fieldZ_ + obsSide_ * 0.5F;
  for (const auto& o : snap.repulsorVision) {
    BoxPrimitive box{
        .center = glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), hz},
        .size = glm::vec3{obsSide_, obsSide_, obsSide_},
        .yawDeg = 0.0F,
        .color = colOther_,
        .pass = RenderPass::Opaque,
    };
    const std::string id = o.oid.empty() ? ("obstacle_" + std::to_string(sceneGraph.Size())) : ("obstacle_" + o.oid);
    sceneGraph.AddNode(MakeStaticEntity(
        id,
        RenderPass::Opaque,
        box,
        {.enabled = true, .boundsRadius = std::max(obsSide_, 0.1F)}));
  }
}

void Season2026RebuiltModelBuilder::AppendRobotPrimitives(
    scenegraph::SceneGraphBuilder& sceneGraph,
    const WorldSnapshot& snap) const {
  if (snap.pose.has_value()) {
    const Pose2D& p = snap.pose.value();
    BoxPrimitive robotBody{
        .center = glm::vec3{0.0F, 0.0F, fieldZ_ + robotH_ * 0.5F},
        .size = glm::vec3{robotL_, robotW_, robotH_},
        .yawDeg = 0.0F,
        .color = colUs_,
    };
    RenderEntity robotEntity{
        .id = "robot_pose_live",
        .pass = RenderPass::Opaque,
        .payload = robotBody,
        .parentId = "",
        .transform =
            Transform3D{
                .position = glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), 0.0F},
                .rotationDeg = glm::vec3{0.0F, 0.0F, glm::degrees(static_cast<float>(p.thetaRad))},
                .scale = glm::vec3{1.0F, 1.0F, 1.0F},
            },
        .hasTransform = true,
        .culling =
            EntityCulling{
                .enabled = true,
                .boundsRadius = std::max(robotL_, robotW_),
            },
    };
    sceneGraph.AddNode(std::move(robotEntity));

    const float headingLen = std::max(robotL_, robotW_) * 0.95F;
    const glm::vec3 start{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ + 0.02F};
    const glm::vec3 end{start.x + headingLen * std::cos(static_cast<float>(p.thetaRad)),
                        start.y + headingLen * std::sin(static_cast<float>(p.thetaRad)),
                        start.z};
    LinePrimitive headingLine{
        .a = start,
        .b = end,
        .color = colHeading_,
        .width = 3.0F,
        .pass = RenderPass::Transparent,
    };
    sceneGraph.AddNode(MakeStaticEntity(
        "robot_heading_live",
        RenderPass::Transparent,
        headingLine,
        {.enabled = false, .boundsRadius = 0.0F}));
  }

  if (snap.activeGoal.has_value()) {
    const Pose2D& p = snap.activeGoal.value();
    BoxPrimitive box{
        .center = glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
        .size = glm::vec3{robotL_, robotW_, robotH_},
        .yawDeg = glm::degrees(static_cast<float>(p.thetaRad)),
        .color = colActive_,
        .pass = RenderPass::Transparent,
    };
    sceneGraph.AddNode(MakeStaticEntity(
        "robot_goal_active",
        RenderPass::Transparent,
        box,
        {.enabled = true, .boundsRadius = std::max(robotL_, robotW_)}));
  }

  if (snap.chosenCollect.has_value()) {
    const Pose2D& p = snap.chosenCollect.value();
    BoxPrimitive box{
        .center = glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
        .size = glm::vec3{robotL_, robotW_, robotH_},
        .yawDeg = glm::degrees(static_cast<float>(p.thetaRad)),
        .color = colChosen_,
        .pass = RenderPass::Transparent,
    };
    sceneGraph.AddNode(MakeStaticEntity(
        "robot_goal_chosen",
        RenderPass::Transparent,
        box,
        {.enabled = true, .boundsRadius = std::max(robotL_, robotW_)}));
  }

  if (snap.finalCollect.has_value()) {
    const Pose2D& p = snap.finalCollect.value();
    BoxPrimitive box{
        .center = glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
        .size = glm::vec3{robotL_, robotW_, robotH_},
        .yawDeg = glm::degrees(static_cast<float>(p.thetaRad)),
        .color = colFinal_,
        .pass = RenderPass::Transparent,
    };
    sceneGraph.AddNode(MakeStaticEntity(
        "robot_goal_final",
        RenderPass::Transparent,
        box,
        {.enabled = true, .boundsRadius = std::max(robotL_, robotW_)}));
  }
}

void Season2026RebuiltModelBuilder::AppendCameraPrimitives(scenegraph::SceneGraphBuilder& sceneGraph, const WorldSnapshot& snap) const {
  if (!snap.pose.has_value() || snap.cameras.empty()) {
    return;
  }

  const Pose2D& rp = snap.pose.value();
  const float rx = static_cast<float>(rp.x);
  const float ry = static_cast<float>(rp.y);
  const float rz = fieldZ_;
  const float yaw = static_cast<float>(rp.thetaRad);

  const float yawCos = std::cos(yaw);
  const float yawSin = std::sin(yaw);

  for (std::size_t cameraIndex = 0; cameraIndex < snap.cameras.size(); ++cameraIndex) {
    const auto& c = snap.cameras[cameraIndex];
    const std::string cameraKey = c.name.empty() ? ("camera_" + std::to_string(cameraIndex)) : c.name;
    const float cx = rx + static_cast<float>(c.x) * yawCos - static_cast<float>(c.y) * yawSin;
    const float cy = ry + static_cast<float>(c.x) * yawSin + static_cast<float>(c.y) * yawCos;
    const float cz = rz + static_cast<float>(c.z);

    const float yawWorld = yaw + glm::radians(static_cast<float>(c.yawDeg));
    const float pitchWorld = glm::radians(static_cast<float>(c.pitchDeg));
    const float rollWorld = glm::radians(static_cast<float>(c.rollDeg));

    const float hfov = glm::radians(std::max(1e-6F, static_cast<float>(c.hfovDeg)));
    const float vfov = glm::radians(std::max(1e-6F, static_cast<float>(c.vfovDeg)));
    const float depth = std::max(0.2F, static_cast<float>(c.maxRange));

    BoxPrimitive cameraBody{
        .center = glm::vec3{cx, cy, cz},
        .size = glm::vec3{0.06F, 0.06F, 0.06F},
        .yawDeg = glm::degrees(yawWorld),
        .color = colCam_,
        .pass = RenderPass::Transparent,
    };
    sceneGraph.AddNode(MakeStaticEntity(
        "camera_body_" + cameraKey,
        RenderPass::Transparent,
        cameraBody,
        {.enabled = true, .boundsRadius = 0.2F}));

    const float ch = std::cos(yawWorld);
    const float sh = std::sin(yawWorld);
    const float cp = std::cos(pitchWorld);
    const float sp = std::sin(pitchWorld);

    const glm::vec3 forward{cp * ch, cp * sh, sp};
    const glm::vec3 right{-sh, ch, 0.0F};
    const glm::vec3 up{-sp * ch, -sp * sh, cp};

    const float tanH = std::tan(hfov * 0.5F);
    const float tanV = std::tan(vfov * 0.5F);

    const glm::vec3 farCenter = glm::vec3{cx, cy, cz} + forward * depth;
    const glm::vec3 wv = right * (depth * tanH);
    const glm::vec3 hv = up * (depth * tanV);

    const std::array<glm::vec3, 4> corners = {
        farCenter + wv + hv,
        farCenter + wv - hv,
        farCenter - wv - hv,
        farCenter - wv + hv,
    };

    std::size_t edgeIndex = 0;
    for (const auto& p : corners) {
      LinePrimitive fovRay{.a = glm::vec3{cx, cy, cz}, .b = p, .color = colCamFov_, .width = 1.0F, .pass = RenderPass::Transparent};
      sceneGraph.AddNode(MakeStaticEntity(
          "camera_fov_ray_" + cameraKey + "_" + std::to_string(edgeIndex++),
          RenderPass::Transparent,
          fovRay,
          {.enabled = false, .boundsRadius = 0.0F}));
    }

    for (size_t i = 0; i < corners.size(); ++i) {
      LinePrimitive fovEdge{
          .a = corners[i],
          .b = corners[(i + 1) % corners.size()],
          .color = colCamFov_,
          .width = 1.0F,
          .pass = RenderPass::Transparent};
      sceneGraph.AddNode(MakeStaticEntity(
          "camera_fov_edge_" + cameraKey + "_" + std::to_string(i),
          RenderPass::Transparent,
          fovEdge,
          {.enabled = false, .boundsRadius = 0.0F}));
    }

    const float cr = std::cos(rollWorld);
    const float sr = std::sin(rollWorld);

    const float r00 = ch * cp;
    const float r01 = ch * sp * sr - sh * cr;
    const float r02 = ch * sp * cr + sh * sr;
    const float r10 = sh * cp;
    const float r11 = sh * sp * sr + ch * cr;
    const float r12 = sh * sp * cr - ch * sr;
    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    const std::size_t maxRays = static_cast<std::size_t>(std::max(0, maxCameraDebugRaysPerCamera_));
    const std::size_t total = snap.fieldVision.size();
    const std::size_t step = (maxRays > 0 && total > maxRays) ? ((total + maxRays - 1) / maxRays) : 1;
    std::size_t sampleIndex = 0;
    for (const auto& o : snap.fieldVision) {
      if (step > 1 && (sampleIndex % step) != 0) {
        ++sampleIndex;
        continue;
      }
      ++sampleIndex;
      const float ox = static_cast<float>(o.x);
      const float oy = static_cast<float>(o.y);
      const float oz = rz + static_cast<float>(o.z);
      const float dx = ox - cx;
      const float dy = oy - cy;
      const float dz = oz - cz;

      const float xCam = r00 * dx + r10 * dy + r20 * dz;
      const float yCam = r01 * dx + r11 * dy + r21 * dz;
      const float zCam = r02 * dx + r12 * dy + r22 * dz;

      glm::vec4 rayColor = colCamRayBad_;
      if (xCam > 1e-6F) {
        const float dyaw = std::atan2(yCam, xCam);
        const float dpitch = std::atan2(zCam, xCam);
        if (std::abs(dyaw) <= hfov * 0.5F && std::abs(dpitch) <= vfov * 0.5F) {
          rayColor = colCamRayOk_;
        }
      }

      LinePrimitive debugRay{
          .a = glm::vec3{cx, cy, cz},
          .b = glm::vec3{ox, oy, oz},
          .color = rayColor,
          .width = 1.0F,
          .pass = RenderPass::Transparent,
      };
      sceneGraph.AddNode(MakeStaticEntity(
          "camera_debug_ray_" + cameraKey + "_" + std::to_string(sampleIndex),
          RenderPass::Transparent,
          debugRay,
          {.enabled = false, .boundsRadius = 0.0F}));
    }
  }
}

void Season2026RebuiltModelBuilder::AppendCadModelPrimitives(
    scenegraph::SceneGraphBuilder& sceneGraph,
    const WorldSnapshot& snap) const {
  if (showFieldCadModel_ && !fieldCadModelPath_.empty()) {
    const float sx = fieldCadFlipX_ ? -fieldCadScaleM_ : fieldCadScaleM_;
    MeshInstancePrimitive mesh{
        .assetPath = fieldCadModelPath_,
        .position = glm::vec3{fieldCadOffsetXM_, fieldCadOffsetYM_, fieldZ_ + fieldCadZOffsetM_},
        .rotationDeg = glm::vec3{0.0F, 0.0F, 0.0F},
        .scale = glm::vec3{sx, fieldCadScaleM_, fieldCadScaleM_},
        .color = glm::vec4{1.0F, 1.0F, 1.0F, 1.0F},
        .useAssetColor = true,
        .centerOnMeshBounds = true,
        .wireframe = false,
        .roughnessOverride = 0.82F,
        .metallicOverride = 0.03F,
        .pass = RenderPass::Opaque,
    };
    const float fieldCullingRadius = meshculling::ComputeRectFootprintBoundsRadius(
        fieldLength_,
        fieldWidth_,
        fieldCadScaleM_,
        3.0F,
        6.0F);
    sceneGraph.AddNode(
        {.id = "field_cad",
         .pass = RenderPass::Opaque,
         .payload = mesh,
         .parentId = "",
         .transform = {},
         .hasTransform = false,
         .culling = {.enabled = true, .boundsRadius = fieldCullingRadius}});
  }

  if (!showRobotCadModel_ || robotCadModelPath_.empty()) {
    return;
  }

  auto appendPoseCad = [&](
                           const std::optional<Pose2D>& pose,
                           const glm::vec4& color,
                           const std::string& id,
                           const std::string& parentId,
                           const bool worldTransformFromPose) {
    if (!pose.has_value()) {
      return;
    }

    const Pose2D& p = pose.value();
    const bool useLocalPose = worldTransformFromPose || !parentId.empty();
    MeshInstancePrimitive mesh{
        .assetPath = robotCadModelPath_,
        .position = useLocalPose
                        ? glm::vec3{0.0F, 0.0F, fieldZ_ + robotCadZOffsetM_}
                        : glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotCadZOffsetM_},
        .rotationDeg = glm::vec3{0.0F, 0.0F, 0.0F},
        .scale = glm::vec3{robotCadScaleM_, robotCadScaleM_, robotCadScaleM_},
        .color = color,
        .wireframe = false,
        .roughnessOverride = 0.55F,
        .metallicOverride = 0.08F,
        .pass = RenderPass::Opaque,
    };
    RenderEntity entity{
        .id = id,
        .pass = RenderPass::Opaque,
        .payload = mesh,
        .parentId = parentId,
        .transform =
            Transform3D{
                .position = glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), 0.0F},
                .rotationDeg = glm::vec3{0.0F, 0.0F, glm::degrees(static_cast<float>(p.thetaRad))},
                .scale = glm::vec3{1.0F, 1.0F, 1.0F},
            },
        .hasTransform = worldTransformFromPose,
        .culling =
            meshculling::ResolveMeshEntityCulling(
                mesh,
                EntityCulling{
                .enabled = true,
                .boundsRadius = std::max(robotL_, robotW_),
                },
                0.20F,
                std::max(0.35F, std::min(robotL_, robotW_) * 0.5F)),
    };
    sceneGraph.AddNode(std::move(entity));
  };

  appendPoseCad(snap.pose, glm::vec4{0.95F, 0.45F, 0.15F, 0.90F}, "robot_cad_live", "robot_pose_live", false);
  appendPoseCad(snap.activeGoal, glm::vec4{0.20F, 0.88F, 0.35F, 0.50F}, "robot_cad_active", "", true);
  appendPoseCad(snap.chosenCollect, glm::vec4{0.20F, 0.40F, 0.95F, 0.50F}, "robot_cad_chosen", "", true);
  appendPoseCad(snap.finalCollect, glm::vec4{0.95F, 0.35F, 0.15F, 0.55F}, "robot_cad_final", "", true);
}

std::string Season2026RebuiltModelBuilder::NormalizeType(const std::string& type) {
  std::string out = type;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

}  // namespace repulsor3d
