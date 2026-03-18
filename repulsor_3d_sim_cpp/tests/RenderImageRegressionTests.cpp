#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

namespace {

constexpr std::uint64_t kFnv64OffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnv64Prime = 1099511628211ULL;

struct Rgba8 {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
};

struct RasterImage {
  int width = 0;
  int height = 0;
  std::vector<Rgba8> pixels;

  Rgba8& At(const int x, const int y) {
    return pixels[static_cast<std::size_t>(y * width + x)];
  }
};

struct WorldBounds2D {
  float minX = -8.0F;
  float maxX = 8.0F;
  float minY = -4.0F;
  float maxY = 4.0F;
};

std::string ReadTextFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return {};
  }
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

bool WriteTextFile(const std::string& path, const std::string& contents) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  out << contents;
  return true;
}

void FnvMixByte(std::uint64_t& hash, const std::uint8_t byte) {
  hash ^= static_cast<std::uint64_t>(byte);
  hash *= kFnv64Prime;
}

std::string ToHexString(const std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << value;
  return out.str();
}

WorldBounds2D ComputeWorldBounds(const repulsor3d::RenderSceneFrame& frame) {
  bool hasAny = false;
  WorldBounds2D bounds;
  bounds.minX = std::numeric_limits<float>::max();
  bounds.maxX = std::numeric_limits<float>::lowest();
  bounds.minY = std::numeric_limits<float>::max();
  bounds.maxY = std::numeric_limits<float>::lowest();

  auto includePoint = [&](const float x, const float y) {
    hasAny = true;
    bounds.minX = std::min(bounds.minX, x);
    bounds.maxX = std::max(bounds.maxX, x);
    bounds.minY = std::min(bounds.minY, y);
    bounds.maxY = std::max(bounds.maxY, y);
  };

  for (const auto& sphere : frame.spheres) {
    includePoint(sphere.center.x - sphere.radius, sphere.center.y - sphere.radius);
    includePoint(sphere.center.x + sphere.radius, sphere.center.y + sphere.radius);
  }
  for (const auto& box : frame.boxes) {
    includePoint(box.center.x - box.size.x * 0.5F, box.center.y - box.size.y * 0.5F);
    includePoint(box.center.x + box.size.x * 0.5F, box.center.y + box.size.y * 0.5F);
  }
  for (const auto& line : frame.lines) {
    includePoint(line.a.x, line.a.y);
    includePoint(line.b.x, line.b.y);
  }
  for (const auto& mesh : frame.meshInstances) {
    includePoint(mesh.position.x, mesh.position.y);
    includePoint(mesh.position.x + mesh.scale.x * 0.5F, mesh.position.y + mesh.scale.y * 0.5F);
    includePoint(mesh.position.x - mesh.scale.x * 0.5F, mesh.position.y - mesh.scale.y * 0.5F);
  }
  for (const auto& entity : frame.entities) {
    std::visit(
        [&](const auto& payload) {
          using T = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<T, repulsor3d::SpherePrimitive>) {
            includePoint(payload.center.x - payload.radius, payload.center.y - payload.radius);
            includePoint(payload.center.x + payload.radius, payload.center.y + payload.radius);
          } else if constexpr (std::is_same_v<T, repulsor3d::BoxPrimitive>) {
            includePoint(payload.center.x - payload.size.x * 0.5F, payload.center.y - payload.size.y * 0.5F);
            includePoint(payload.center.x + payload.size.x * 0.5F, payload.center.y + payload.size.y * 0.5F);
          } else if constexpr (std::is_same_v<T, repulsor3d::LinePrimitive>) {
            includePoint(payload.a.x, payload.a.y);
            includePoint(payload.b.x, payload.b.y);
          } else if constexpr (std::is_same_v<T, repulsor3d::MeshInstancePrimitive>) {
            includePoint(payload.position.x, payload.position.y);
          }
        },
        entity.payload);
  }

  if (!hasAny) {
    return WorldBounds2D{};
  }

  const float paddingX = std::max((bounds.maxX - bounds.minX) * 0.10F, 0.8F);
  const float paddingY = std::max((bounds.maxY - bounds.minY) * 0.10F, 0.8F);
  bounds.minX -= paddingX;
  bounds.maxX += paddingX;
  bounds.minY -= paddingY;
  bounds.maxY += paddingY;
  if (std::abs(bounds.maxX - bounds.minX) < 1e-4F) {
    bounds.minX -= 1.0F;
    bounds.maxX += 1.0F;
  }
  if (std::abs(bounds.maxY - bounds.minY) < 1e-4F) {
    bounds.minY -= 1.0F;
    bounds.maxY += 1.0F;
  }
  return bounds;
}

glm::vec2 WorldToPixel(const WorldBounds2D& bounds, const RasterImage& image, const glm::vec2 world) {
  const float nx = (world.x - bounds.minX) / std::max(bounds.maxX - bounds.minX, 1e-5F);
  const float ny = (world.y - bounds.minY) / std::max(bounds.maxY - bounds.minY, 1e-5F);
  return glm::vec2{
      nx * static_cast<float>(image.width - 1),
      (1.0F - ny) * static_cast<float>(image.height - 1)};
}

void AlphaBlendPixel(Rgba8& dst, const glm::vec4& src) {
  const float a = std::clamp(src.a, 0.0F, 1.0F);
  const float ia = 1.0F - a;
  const float r = src.r * 255.0F;
  const float g = src.g * 255.0F;
  const float b = src.b * 255.0F;
  dst.r = static_cast<std::uint8_t>(std::clamp(r * a + static_cast<float>(dst.r) * ia, 0.0F, 255.0F));
  dst.g = static_cast<std::uint8_t>(std::clamp(g * a + static_cast<float>(dst.g) * ia, 0.0F, 255.0F));
  dst.b = static_cast<std::uint8_t>(std::clamp(b * a + static_cast<float>(dst.b) * ia, 0.0F, 255.0F));
}

void DrawDisc(RasterImage& image, const glm::vec2 centerPx, const float radiusPx, const glm::vec4& color) {
  const int minX = std::max(0, static_cast<int>(std::floor(centerPx.x - radiusPx)));
  const int maxX = std::min(image.width - 1, static_cast<int>(std::ceil(centerPx.x + radiusPx)));
  const int minY = std::max(0, static_cast<int>(std::floor(centerPx.y - radiusPx)));
  const int maxY = std::min(image.height - 1, static_cast<int>(std::ceil(centerPx.y + radiusPx)));
  const float rr = radiusPx * radiusPx;
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const float dx = static_cast<float>(x) - centerPx.x;
      const float dy = static_cast<float>(y) - centerPx.y;
      if (dx * dx + dy * dy <= rr) {
        AlphaBlendPixel(image.At(x, y), color);
      }
    }
  }
}

void DrawSegment(RasterImage& image, glm::vec2 aPx, glm::vec2 bPx, const float widthPx, const glm::vec4& color) {
  const glm::vec2 d = bPx - aPx;
  const float length = glm::length(d);
  const int steps = std::max(1, static_cast<int>(std::ceil(length)));
  for (int i = 0; i <= steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    const glm::vec2 p = aPx + d * t;
    DrawDisc(image, p, std::max(0.5F, widthPx * 0.5F), color);
  }
}

void DrawOrientedBox(
    RasterImage& image,
    const WorldBounds2D& bounds,
    const glm::vec2 centerWorld,
    const glm::vec2 sizeWorld,
    const float yawDeg,
    const glm::vec4& color) {
  const glm::vec2 centerPx = WorldToPixel(bounds, image, centerWorld);
  const glm::vec2 sizePx = glm::abs(WorldToPixel(bounds, image, centerWorld + sizeWorld * 0.5F) - centerPx) * 2.0F;
  const float hx = std::max(sizePx.x * 0.5F, 0.5F);
  const float hy = std::max(sizePx.y * 0.5F, 0.5F);
  const float angle = glm::radians(-yawDeg);
  const float c = std::cos(angle);
  const float s = std::sin(angle);

  const int minX = std::max(0, static_cast<int>(std::floor(centerPx.x - hx - 2.0F)));
  const int maxX = std::min(image.width - 1, static_cast<int>(std::ceil(centerPx.x + hx + 2.0F)));
  const int minY = std::max(0, static_cast<int>(std::floor(centerPx.y - hy - 2.0F)));
  const int maxY = std::min(image.height - 1, static_cast<int>(std::ceil(centerPx.y + hy + 2.0F)));
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const float lx = (static_cast<float>(x) - centerPx.x) * c - (static_cast<float>(y) - centerPx.y) * s;
      const float ly = (static_cast<float>(x) - centerPx.x) * s + (static_cast<float>(y) - centerPx.y) * c;
      if (std::abs(lx) <= hx && std::abs(ly) <= hy) {
        AlphaBlendPixel(image.At(x, y), color);
      }
    }
  }
}

RasterImage RasterizeTopDown(const repulsor3d::RenderSceneFrame& frame, const int width, const int height) {
  RasterImage image;
  image.width = width;
  image.height = height;
  image.pixels.assign(static_cast<std::size_t>(width * height), Rgba8{12, 14, 18, 255});
  const WorldBounds2D bounds = ComputeWorldBounds(frame);

  if (frame.drawGrid) {
    const glm::vec4 gridColor{0.18F, 0.24F, 0.29F, 0.35F};
    for (int i = 1; i <= 12; ++i) {
      const float t = static_cast<float>(i) / 13.0F;
      const float x = bounds.minX + (bounds.maxX - bounds.minX) * t;
      const float y = bounds.minY + (bounds.maxY - bounds.minY) * t;
      DrawSegment(
          image,
          WorldToPixel(bounds, image, glm::vec2{x, bounds.minY}),
          WorldToPixel(bounds, image, glm::vec2{x, bounds.maxY}),
          1.0F,
          gridColor);
      DrawSegment(
          image,
          WorldToPixel(bounds, image, glm::vec2{bounds.minX, y}),
          WorldToPixel(bounds, image, glm::vec2{bounds.maxX, y}),
          1.0F,
          gridColor);
    }
  }

  for (const auto& line : frame.lines) {
    DrawSegment(
        image,
        WorldToPixel(bounds, image, glm::vec2{line.a.x, line.a.y}),
        WorldToPixel(bounds, image, glm::vec2{line.b.x, line.b.y}),
        std::max(1.0F, line.width),
        line.color);
  }
  for (const auto& box : frame.boxes) {
    DrawOrientedBox(
        image,
        bounds,
        glm::vec2{box.center.x, box.center.y},
        glm::vec2{box.size.x, box.size.y},
        box.yawDeg,
        box.color);
  }
  for (const auto& sphere : frame.spheres) {
    const glm::vec2 centerPx = WorldToPixel(bounds, image, glm::vec2{sphere.center.x, sphere.center.y});
    const glm::vec2 edgePx = WorldToPixel(bounds, image, glm::vec2{sphere.center.x + sphere.radius, sphere.center.y});
    DrawDisc(image, centerPx, glm::length(edgePx - centerPx), sphere.color);
  }
  for (const auto& mesh : frame.meshInstances) {
    const glm::vec2 centerPx = WorldToPixel(bounds, image, glm::vec2{mesh.position.x, mesh.position.y});
    const glm::vec2 edgePx = WorldToPixel(
        bounds,
        image,
        glm::vec2{mesh.position.x + std::max(mesh.scale.x, mesh.scale.y) * 0.35F, mesh.position.y});
    const glm::vec4 meshColor = mesh.color.a > 0.0F ? mesh.color : glm::vec4{0.92F, 0.92F, 0.92F, 0.70F};
    DrawDisc(image, centerPx, std::max(2.0F, glm::length(edgePx - centerPx)), meshColor);
  }

  return image;
}

std::string BuildImageHashHex(const RasterImage& image) {
  std::uint64_t hash = kFnv64OffsetBasis;
  for (const auto& px : image.pixels) {
    FnvMixByte(hash, px.r);
    FnvMixByte(hash, px.g);
    FnvMixByte(hash, px.b);
    FnvMixByte(hash, px.a);
  }
  return ToHexString(hash);
}

int Run2026RebuiltImageRegression() {
  repulsor3d::ViewerConfig cfg;
  cfg.sceneProfile = "2026Rebuilt";
  cfg.showFieldCadModel = true;
  cfg.fieldCadModelPath = "field_2026rebuilt.gltf";
  cfg.showRobotCadModel = true;
  cfg.robotCadModelPath = "field_2026rebuilt.gltf";

  repulsor3d::SnapshotBundle bundle;
  bundle.connected = true;
  bundle.pieces = 5;
  bundle.method = "image_regression";
  bundle.snapshot.pose = repulsor3d::Pose2D{1.0, -1.0, 0.15};
  bundle.snapshot.activeGoal = repulsor3d::Pose2D{2.5, 1.8, -0.2};
  bundle.snapshot.finalCollect = repulsor3d::Pose2D{-1.5, 1.3, 0.6};

  for (int i = 0; i < 4; ++i) {
    repulsor3d::FieldVisionObject fuel;
    fuel.oid = "fuel_" + std::to_string(i);
    fuel.type = "fuel";
    fuel.x = -0.6 + static_cast<double>(i) * 0.7;
    fuel.y = -1.0 + static_cast<double>(i % 2) * 1.6;
    fuel.z = 0.05;
    bundle.snapshot.fieldVision.push_back(fuel);
  }

  repulsor3d::RepulsorVisionObstacle obs;
  obs.oid = "obs_0";
  obs.kind = "generic";
  obs.x = 0.7;
  obs.y = -0.2;
  obs.sizeX = 0.9;
  obs.sizeY = 0.8;
  bundle.snapshot.repulsorVision.push_back(obs);

  repulsor3d::SceneToggleState toggles;
  toggles.showFieldImage = false;
  toggles.showTruthFuel = true;
  toggles.showAgeFilteredFuel = true;
  toggles.showCameraDebug = true;

  repulsor3d::Season2026RebuiltModelBuilder builder(cfg);
  const repulsor3d::RenderSceneFrame frame = builder.BuildFrame(bundle, toggles);
  const RasterImage image = RasterizeTopDown(frame, 320, 200);
  const std::string hash = BuildImageHashHex(image);

  std::string goldenPath = "tests/golden/2026rebuilt_frame_raster_hash.txt";
#ifdef REPULSOR_TEST_SOURCE_DIR
  goldenPath = std::string(REPULSOR_TEST_SOURCE_DIR) + "/tests/golden/2026rebuilt_frame_raster_hash.txt";
#endif

  if (const char* updateEnv = std::getenv("REPULSOR_UPDATE_IMAGE_GOLDEN");
      updateEnv != nullptr && std::string(updateEnv) == "1") {
    if (!WriteTextFile(goldenPath, hash + "\n")) {
      std::cerr << "Failed writing golden image hash: " << goldenPath << "\n";
      return 4;
    }
  }

  std::string golden = ReadTextFile(goldenPath);
  while (!golden.empty() && (golden.back() == '\n' || golden.back() == '\r' || golden.back() == ' ' || golden.back() == '\t')) {
    golden.pop_back();
  }
  if (golden.empty()) {
    std::cerr << "Golden image hash missing or empty: " << goldenPath << "\n";
    std::cerr << "Computed hash: " << hash << "\n";
    return 2;
  }
  if (golden != hash) {
    std::cerr << "Image regression mismatch against " << goldenPath << "\n";
    std::cerr << "Expected hash: " << golden << "\n";
    std::cerr << "Actual hash:   " << hash << "\n";
    return 3;
  }
  return 0;
}

}  // namespace

int main() {
  if (const int rc = Run2026RebuiltImageRegression(); rc != 0) {
    return rc;
  }
  std::cout << "Render image regression tests passed\n";
  return 0;
}
