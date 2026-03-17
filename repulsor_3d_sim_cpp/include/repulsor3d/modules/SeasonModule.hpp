#pragma once

#include <functional>
#include <memory>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"

namespace repulsor3d {

inline constexpr int kSeasonModuleAbiVersion = 1;

class ISeasonModule {
 public:
  virtual ~ISeasonModule() = default;

  virtual std::string Id() const = 0;
  virtual std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& cfg) const = 0;
};

using SeasonModuleFactoryFn = std::function<std::unique_ptr<ISeasonModule>()>;
using CreateSeasonModuleAbiFn = ISeasonModule* (*)();
using DestroySeasonModuleAbiFn = void (*)(ISeasonModule*);
using QuerySeasonModuleAbiVersionFn = int (*)();

void RegisterSeasonModule(const std::string& moduleId, SeasonModuleFactoryFn factoryFn);
std::unique_ptr<ISeasonModule> CreateSeasonModule(const std::string& moduleId);
std::unique_ptr<ISeasonModule> CreateSeasonModuleFromPlugin(const std::string& pluginPath);
std::unique_ptr<ISeasonModule> CreateDefaultSeasonModule(const ViewerConfig& cfg);

}  // namespace repulsor3d
