#pragma once

#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace repulsor3d::cad {

struct ProjectedSphere {
  bool valid = false;
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;
  float nearDepth01 = 1.0F;
  float farDepth01 = 1.0F;
  float radiusPx = 0.0F;
};

ProjectedSphere ProjectSphereToViewport(
    const glm::vec3& worldCenter,
    float worldRadius,
    const glm::mat4& viewProjection,
    int viewportWidth,
    int viewportHeight);

class TiledOcclusionBuffer {
 public:
  void Reset(int viewportWidth, int viewportHeight, int tilesX, int tilesY);
  bool IsOccluded(
      const ProjectedSphere& projected,
      float depthMargin,
      float maxCullRadiusPx,
      float minCoverage) const;
  void SubmitOccluder(const ProjectedSphere& projected, float minOccluderRadiusPx);

 private:
  int tilesX_ = 1;
  int tilesY_ = 1;
  int viewportWidth_ = 1;
  int viewportHeight_ = 1;
  float tileWidth_ = 1.0F;
  float tileHeight_ = 1.0F;
  std::vector<float> occluderNearestDepth01_;
  std::vector<float> occluderCoverage_;
};

}  // namespace repulsor3d::cad
