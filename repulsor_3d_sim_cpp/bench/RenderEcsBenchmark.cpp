#include <chrono>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

#include "repulsor3d/render/RenderCommandBuffer.hpp"
#include "repulsor3d/render/SceneFrame.hpp"
#include "repulsor3d/render/ecs/Systems.hpp"

namespace {

repulsor3d::RenderSceneFrame BuildSyntheticScene(const std::size_t entityCount) {
  repulsor3d::RenderSceneFrame frame;
  frame.entities.reserve(entityCount);
  for (std::size_t i = 0; i < entityCount; ++i) {
    const float t = static_cast<float>(i);
    const float x = std::fmod(t * 0.173F, 20.0F) - 10.0F;
    const float y = std::fmod(t * 0.219F, 14.0F) - 7.0F;
    const float z = 0.2F + std::fmod(t * 0.037F, 2.0F);
    frame.entities.push_back(
        {.id = "bench_entity_" + std::to_string(i),
         .pass = repulsor3d::RenderPass::Opaque,
         .payload =
             repulsor3d::BoxPrimitive{
                 .center = glm::vec3{0.0F, 0.0F, 0.0F},
                 .size = glm::vec3{0.8F, 0.8F, 0.3F},
                 .yawDeg = 0.0F,
                 .color = glm::vec4{0.9F, 0.4F, 0.2F, 0.8F},
             },
         .parentId = "",
         .transform =
             repulsor3d::Transform3D{
                 .position = glm::vec3{x, y, z},
                 .rotationDeg = glm::vec3{0.0F, 0.0F, std::fmod(t * 3.0F, 360.0F)},
                 .scale = glm::vec3{1.0F, 1.0F, 1.0F},
             },
         .hasTransform = true,
         .culling = repulsor3d::EntityCulling{.enabled = true, .boundsRadius = 0.9F}});
  }
  return frame;
}

}  // namespace

double GetEnvThreshold(const char* name, const double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

int main() {
  repulsor3d::RenderSceneFrame scene = BuildSyntheticScene(12000);
  const glm::mat4 view = glm::lookAt(glm::vec3{13.0F, 8.0F, 7.0F}, glm::vec3{0.0F, 0.0F, 0.0F}, glm::vec3{0.0F, 0.0F, 1.0F});
  const glm::mat4 proj = glm::perspective(glm::radians(65.0F), 16.0F / 9.0F, 0.1F, 250.0F);
  const glm::mat4 vp = proj * view;

  constexpr int kIterations = 120;
  double cullTotalMs = 0.0;
  double commandTotalMs = 0.0;
  int lastVisible = 0;
  int lastCommands = 0;

  for (int i = 0; i < kIterations; ++i) {
    repulsor3d::RenderSceneFrame working = scene;
    const auto cullStart = std::chrono::steady_clock::now();
    const repulsor3d::EntityCullingStats stats = repulsor3d::ApplyRenderEntityHierarchyAndCulling(working, vp);
    const auto cullEnd = std::chrono::steady_clock::now();
    const auto commandBuffer = repulsor3d::BuildRenderCommandBuffer(working);
    const auto commandEnd = std::chrono::steady_clock::now();

    cullTotalMs +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(cullEnd - cullStart).count();
    commandTotalMs +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(commandEnd - cullEnd).count();
    lastVisible = stats.visible;
    lastCommands = static_cast<int>(commandBuffer.size());
  }

  const double avgCullMs = cullTotalMs / static_cast<double>(kIterations);
  const double avgCommandMs = commandTotalMs / static_cast<double>(kIterations);
  std::cout << "render_bench avgCullMs=" << avgCullMs
            << " avgCommandMs=" << avgCommandMs
            << " visibleEntities=" << lastVisible
            << " commandCount=" << lastCommands << "\n";

  const double maxCullMs = GetEnvThreshold("REPULSOR_BENCH_MAX_CULL_MS", 0.0);
  const double maxCommandMs = GetEnvThreshold("REPULSOR_BENCH_MAX_COMMAND_MS", 0.0);
  const bool enforceCullGate = maxCullMs > 0.0;
  const bool enforceCommandGate = maxCommandMs > 0.0;
  if ((enforceCullGate && avgCullMs > maxCullMs) ||
      (enforceCommandGate && avgCommandMs > maxCommandMs)) {
    std::cerr << "Render benchmark exceeded threshold: "
              << "avgCullMs=" << avgCullMs << " (limit " << maxCullMs << "), "
              << "avgCommandMs=" << avgCommandMs << " (limit " << maxCommandMs << ")\n";
    return 2;
  }
  return 0;
}
