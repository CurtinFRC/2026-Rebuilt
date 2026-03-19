#include "repulsor3d/render/cad/CadVisibility.hpp"

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>

namespace repulsor3d::cad {
namespace {

float ComputeOverlapArea(
    const float aMinX,
    const float aMinY,
    const float aMaxX,
    const float aMaxY,
    const float bMinX,
    const float bMinY,
    const float bMaxX,
    const float bMaxY) {
  const float overlapMinX = std::max(aMinX, bMinX);
  const float overlapMinY = std::max(aMinY, bMinY);
  const float overlapMaxX = std::min(aMaxX, bMaxX);
  const float overlapMaxY = std::min(aMaxY, bMaxY);
  if (overlapMaxX <= overlapMinX || overlapMaxY <= overlapMinY) {
    return 0.0F;
  }
  return (overlapMaxX - overlapMinX) * (overlapMaxY - overlapMinY);
}

}  // namespace

ProjectedSphere ProjectSphereToViewport(
    const glm::vec3& worldCenter,
    const float worldRadius,
    const glm::mat4& viewProjection,
    const int viewportWidth,
    const int viewportHeight) {
  ProjectedSphere out;
  if (worldRadius <= 1e-5F || viewportWidth <= 0 || viewportHeight <= 0) {
    return out;
  }

  const glm::vec4 clipCenter = viewProjection * glm::vec4(worldCenter, 1.0F);
  if (std::abs(clipCenter.w) <= 1e-5F) {
    return out;
  }
  const glm::vec3 ndcCenter{clipCenter.x / clipCenter.w, clipCenter.y / clipCenter.w, clipCenter.z / clipCenter.w};

  const glm::vec4 clipOffsetX = viewProjection * glm::vec4(worldCenter + glm::vec3{worldRadius, 0.0F, 0.0F}, 1.0F);
  const glm::vec4 clipOffsetY = viewProjection * glm::vec4(worldCenter + glm::vec3{0.0F, worldRadius, 0.0F}, 1.0F);
  if (std::abs(clipOffsetX.w) <= 1e-5F || std::abs(clipOffsetY.w) <= 1e-5F) {
    return out;
  }

  const glm::vec2 ndcOffsetX{clipOffsetX.x / clipOffsetX.w, clipOffsetX.y / clipOffsetX.w};
  const glm::vec2 ndcOffsetY{clipOffsetY.x / clipOffsetY.w, clipOffsetY.y / clipOffsetY.w};
  const glm::vec2 ndcCenterXY{ndcCenter.x, ndcCenter.y};
  const float ndcRadius = std::max(
      glm::length(ndcOffsetX - ndcCenterXY),
      glm::length(ndcOffsetY - ndcCenterXY));
  const float radiusPx = ndcRadius * 0.5F * static_cast<float>(std::min(viewportWidth, viewportHeight));
  if (!std::isfinite(radiusPx) || radiusPx <= 0.0F) {
    return out;
  }

  const float centerX = (ndcCenter.x * 0.5F + 0.5F) * static_cast<float>(viewportWidth);
  const float centerY = (1.0F - (ndcCenter.y * 0.5F + 0.5F)) * static_cast<float>(viewportHeight);
  int minX = static_cast<int>(std::floor(centerX - radiusPx));
  int maxX = static_cast<int>(std::ceil(centerX + radiusPx));
  int minY = static_cast<int>(std::floor(centerY - radiusPx));
  int maxY = static_cast<int>(std::ceil(centerY + radiusPx));
  minX = std::clamp(minX, 0, viewportWidth - 1);
  maxX = std::clamp(maxX, 0, viewportWidth - 1);
  minY = std::clamp(minY, 0, viewportHeight - 1);
  maxY = std::clamp(maxY, 0, viewportHeight - 1);
  if (minX > maxX || minY > maxY) {
    return out;
  }

  const float centerDepth01 = std::clamp(ndcCenter.z * 0.5F + 0.5F, 0.0F, 1.0F);
  const float depthSpread = std::clamp(ndcRadius * 0.4F, 0.0F, 1.0F);
  out.valid = true;
  out.minX = minX;
  out.maxX = maxX;
  out.minY = minY;
  out.maxY = maxY;
  out.nearDepth01 = std::clamp(centerDepth01 - depthSpread, 0.0F, 1.0F);
  out.farDepth01 = std::clamp(centerDepth01 + depthSpread, 0.0F, 1.0F);
  out.radiusPx = radiusPx;
  return out;
}

void TiledOcclusionBuffer::Reset(const int viewportWidth, const int viewportHeight, const int tilesX, const int tilesY) {
  viewportWidth_ = std::max(viewportWidth, 1);
  viewportHeight_ = std::max(viewportHeight, 1);
  tilesX_ = std::max(tilesX, 1);
  tilesY_ = std::max(tilesY, 1);
  tileWidth_ = static_cast<float>(viewportWidth_) / static_cast<float>(tilesX_);
  tileHeight_ = static_cast<float>(viewportHeight_) / static_cast<float>(tilesY_);
  const std::size_t tileCount = static_cast<std::size_t>(tilesX_ * tilesY_);
  occluderNearestDepth01_.assign(tileCount, 1.0F);
  occluderCoverage_.assign(tileCount, 0.0F);
}

bool TiledOcclusionBuffer::IsOccluded(
    const ProjectedSphere& projected,
    const float depthMargin,
    const float maxCullRadiusPx,
    const float minCoverage) const {
  if (!projected.valid || occluderNearestDepth01_.empty() || occluderCoverage_.empty()) {
    return false;
  }
  if (projected.radiusPx > maxCullRadiusPx) {
    return false;
  }

  const int tileMinX = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.minX) / tileWidth_)), 0, tilesX_ - 1);
  const int tileMaxX = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.maxX) / tileWidth_)), 0, tilesX_ - 1);
  const int tileMinY = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.minY) / tileHeight_)), 0, tilesY_ - 1);
  const int tileMaxY = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.maxY) / tileHeight_)), 0, tilesY_ - 1);
  const float requiredDepth = projected.nearDepth01 - std::max(depthMargin, 0.0F);
  const float requiredCoverage = std::clamp(minCoverage, 0.0F, 1.0F);

  for (int y = tileMinY; y <= tileMaxY; ++y) {
    for (int x = tileMinX; x <= tileMaxX; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y * tilesX_ + x);
      if (idx >= occluderNearestDepth01_.size() || idx >= occluderCoverage_.size()) {
        continue;
      }
      if (occluderCoverage_[idx] < requiredCoverage) {
        return false;
      }
      if (occluderNearestDepth01_[idx] > requiredDepth) {
        return false;
      }
    }
  }
  return true;
}

void TiledOcclusionBuffer::SubmitOccluder(
    const ProjectedSphere& projected,
    const float minOccluderRadiusPx) {
  if (!projected.valid || occluderNearestDepth01_.empty() || occluderCoverage_.empty()) {
    return;
  }
  if (projected.radiusPx < std::max(0.0F, minOccluderRadiusPx)) {
    return;
  }
  const int tileMinX = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.minX) / tileWidth_)), 0, tilesX_ - 1);
  const int tileMaxX = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.maxX) / tileWidth_)), 0, tilesX_ - 1);
  const int tileMinY = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.minY) / tileHeight_)), 0, tilesY_ - 1);
  const int tileMaxY = std::clamp(static_cast<int>(std::floor(static_cast<float>(projected.maxY) / tileHeight_)), 0, tilesY_ - 1);
  const float projectedMinX = static_cast<float>(projected.minX);
  const float projectedMinY = static_cast<float>(projected.minY);
  const float projectedMaxX = static_cast<float>(projected.maxX + 1);
  const float projectedMaxY = static_cast<float>(projected.maxY + 1);

  for (int y = tileMinY; y <= tileMaxY; ++y) {
    for (int x = tileMinX; x <= tileMaxX; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y * tilesX_ + x);
      if (idx >= occluderNearestDepth01_.size() || idx >= occluderCoverage_.size()) {
        continue;
      }
      const float tileMinPxX = static_cast<float>(x) * tileWidth_;
      const float tileMinPxY = static_cast<float>(y) * tileHeight_;
      const float tileMaxPxX = tileMinPxX + tileWidth_;
      const float tileMaxPxY = tileMinPxY + tileHeight_;
      const float overlapArea = ComputeOverlapArea(
          projectedMinX,
          projectedMinY,
          projectedMaxX,
          projectedMaxY,
          tileMinPxX,
          tileMinPxY,
          tileMaxPxX,
          tileMaxPxY);
      if (overlapArea <= 0.0F) {
        continue;
      }
      const float tileArea = std::max(tileWidth_ * tileHeight_, 1.0F);
      const float coverage = std::clamp(overlapArea / tileArea, 0.0F, 1.0F);
      occluderCoverage_[idx] = std::clamp(occluderCoverage_[idx] + coverage, 0.0F, 1.0F);
      occluderNearestDepth01_[idx] = std::min(occluderNearestDepth01_[idx], projected.nearDepth01);
    }
  }
}

}  // namespace repulsor3d::cad
