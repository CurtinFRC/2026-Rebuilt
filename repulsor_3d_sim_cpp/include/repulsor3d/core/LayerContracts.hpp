#pragma once

#include <string>

namespace repulsor3d {

enum class ArchitectureLayer {
  Core,
  Domain,
  DataSource,
  Render,
  Season,
  App,
  Plugin,
};

inline const char* ArchitectureLayerName(const ArchitectureLayer layer) {
  switch (layer) {
    case ArchitectureLayer::Core:
      return "core";
    case ArchitectureLayer::Domain:
      return "domain";
    case ArchitectureLayer::DataSource:
      return "datasource";
    case ArchitectureLayer::Render:
      return "render";
    case ArchitectureLayer::Season:
      return "season";
    case ArchitectureLayer::App:
      return "app";
    case ArchitectureLayer::Plugin:
      return "plugin";
  }
  return "unknown";
}

inline bool IsLayerDependencyAllowed(const ArchitectureLayer from, const ArchitectureLayer to) {
  if (from == to) {
    return true;
  }

  switch (from) {
    case ArchitectureLayer::Core:
      return false;
    case ArchitectureLayer::Domain:
      return to == ArchitectureLayer::Core;
    case ArchitectureLayer::DataSource:
      return to == ArchitectureLayer::Core || to == ArchitectureLayer::Domain;
    case ArchitectureLayer::Render:
      return to == ArchitectureLayer::Core || to == ArchitectureLayer::Domain;
    case ArchitectureLayer::Season:
      return to == ArchitectureLayer::Core || to == ArchitectureLayer::Domain || to == ArchitectureLayer::Render;
    case ArchitectureLayer::App:
      return true;
    case ArchitectureLayer::Plugin:
      return to != ArchitectureLayer::App;
  }
  return false;
}

inline std::string DescribeLayerDependency(const ArchitectureLayer from, const ArchitectureLayer to) {
  return std::string(ArchitectureLayerName(from)) + " -> " + ArchitectureLayerName(to);
}

}  // namespace repulsor3d
