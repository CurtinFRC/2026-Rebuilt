#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Diagnostics.hpp"
#include "repulsor3d/core/LayerContracts.hpp"
#include "repulsor3d/domain/CoordinateSystemService.hpp"
#include "repulsor3d/plugins/PluginSdk.hpp"
#include "repulsor3d/render/pipeline/RenderFeatureFactoryRegistry.hpp"
#include "repulsor3d/render/pipeline/RenderPipelineConfig.hpp"
#include "repulsor3d/render/resources/ResourceLifetimeManager.hpp"
#include "repulsor3d/season/SeasonDefinitionRegistry.hpp"
#include "repulsor3d/async/TaskScheduler.hpp"

namespace {

class DummyFeature final : public repulsor3d::IRenderFeature {
 public:
  explicit DummyFeature(std::string name) : name_(std::move(name)) {}

  std::string Name() const override { return name_; }
  void Render(const repulsor3d::RenderFeatureContext&, const repulsor3d::RendererDrawApi&) override {}

 private:
  std::string name_;
};

using Layer = repulsor3d::ArchitectureLayer;
namespace fs = std::filesystem;

std::optional<fs::path> FindProjectRoot() {
  fs::path probe = fs::current_path();
  for (int i = 0; i < 8; ++i) {
    const fs::path includePath = probe / "include" / "repulsor3d";
    const fs::path srcPath = probe / "src";
    if (fs::exists(includePath) && fs::exists(srcPath)) {
      return probe;
    }
    if (!probe.has_parent_path()) {
      break;
    }
    probe = probe.parent_path();
  }
  return std::nullopt;
}

std::optional<Layer> LayerFromFilePath(const fs::path& filePath) {
  const std::string p = filePath.generic_string();
  if (p.find("DataSourceFactory.cpp") != std::string::npos ||
      p.find("SeasonModule.cpp") != std::string::npos ||
      p.find("RenderFeaturePlugin.cpp") != std::string::npos ||
      p.find("RenderWorldAdapter.cpp") != std::string::npos) {
    return Layer::App;
  }
  if (p.find("/render/") != std::string::npos || p.find("\\render\\") != std::string::npos ||
      p.find("Renderer.cpp") != std::string::npos || p.find("Renderer.hpp") != std::string::npos) {
    return Layer::Render;
  }
  if (p.find("/domain/") != std::string::npos || p.find("\\domain\\") != std::string::npos) {
    return Layer::Domain;
  }
  if (p.find("/season/") != std::string::npos || p.find("\\season\\") != std::string::npos ||
      p.find("/modules/") != std::string::npos || p.find("\\modules\\") != std::string::npos) {
    return Layer::Season;
  }
  if (p.find("/nt/") != std::string::npos || p.find("\\nt\\") != std::string::npos ||
      p.find("/datasource/") != std::string::npos || p.find("\\datasource\\") != std::string::npos ||
      p.find("DataSource") != std::string::npos || p.find("ReplayDataSource") != std::string::npos) {
    return Layer::DataSource;
  }
  if (p.find("/core/") != std::string::npos || p.find("\\core\\") != std::string::npos) {
    return Layer::Core;
  }
  if (p.find("/plugins/") != std::string::npos || p.find("\\plugins\\") != std::string::npos) {
    return Layer::Plugin;
  }
  if (p.find("/app/") != std::string::npos || p.find("\\app\\") != std::string::npos ||
      p.find("App.cpp") != std::string::npos || p.find("App.hpp") != std::string::npos) {
    return Layer::App;
  }
  return std::nullopt;
}

std::optional<Layer> LayerFromIncludePath(const std::string& includePath) {
  if (includePath.rfind("repulsor3d/render/", 0) == 0 || includePath == "repulsor3d/Renderer.hpp") {
    return Layer::Render;
  }
  if (includePath.rfind("repulsor3d/domain/", 0) == 0) {
    return Layer::Domain;
  }
  if (includePath.rfind("repulsor3d/season/", 0) == 0 || includePath.rfind("repulsor3d/modules/", 0) == 0) {
    return Layer::Season;
  }
  if (includePath.rfind("repulsor3d/nt/", 0) == 0 || includePath.rfind("repulsor3d/datasource/", 0) == 0 ||
      includePath.find("DataSource") != std::string::npos || includePath.find("ReplayDataSource") != std::string::npos) {
    return Layer::DataSource;
  }
  if (includePath.rfind("repulsor3d/core/", 0) == 0 || includePath == "repulsor3d/Config.hpp" ||
      includePath == "repulsor3d/Diagnostics.hpp") {
    return Layer::Core;
  }
  if (includePath.rfind("repulsor3d/plugins/", 0) == 0) {
    return Layer::Plugin;
  }
  if (includePath.rfind("repulsor3d/app/", 0) == 0 || includePath == "repulsor3d/App.hpp") {
    return Layer::App;
  }
  return std::nullopt;
}

int RunLayerIncludeBoundaryTests() {
  const auto rootOpt = FindProjectRoot();
  if (!rootOpt.has_value()) {
    return 12;
  }

  std::vector<std::string> violations;
  const std::array<fs::path, 2> scanRoots = {*rootOpt / "src", *rootOpt / "include" / "repulsor3d"};
  for (const auto& scanRoot : scanRoots) {
    if (!fs::exists(scanRoot)) {
      continue;
    }

    for (auto it = fs::recursive_directory_iterator(scanRoot); it != fs::recursive_directory_iterator(); ++it) {
      if (!it->is_regular_file()) {
        continue;
      }
      const auto ext = it->path().extension().string();
      if (ext != ".cpp" && ext != ".hpp" && ext != ".h") {
        continue;
      }

      const auto fromLayerOpt = LayerFromFilePath(it->path());
      if (!fromLayerOpt.has_value()) {
        continue;
      }

      std::ifstream in(it->path());
      std::string line;
      while (std::getline(in, line)) {
        const std::size_t includeStart = line.find("#include \"");
        if (includeStart == std::string::npos) {
          continue;
        }
        const std::size_t begin = includeStart + 10;
        const std::size_t end = line.find('"', begin);
        if (end == std::string::npos || end <= begin) {
          continue;
        }

        const std::string includePath = line.substr(begin, end - begin);
        if (includePath.rfind("repulsor3d/", 0) != 0) {
          continue;
        }
        const auto toLayerOpt = LayerFromIncludePath(includePath);
        if (!toLayerOpt.has_value()) {
          continue;
        }
        if (!repulsor3d::IsLayerDependencyAllowed(*fromLayerOpt, *toLayerOpt)) {
          std::ostringstream violation;
          violation << it->path().generic_string() << " includes " << includePath
                    << " (" << repulsor3d::DescribeLayerDependency(*fromLayerOpt, *toLayerOpt) << ")";
          violations.push_back(violation.str());
        }
      }
    }
  }

  if (!violations.empty()) {
    std::cerr << "Layer include violations:\n";
    for (std::size_t i = 0; i < std::min<std::size_t>(violations.size(), 20); ++i) {
      std::cerr << "  - " << violations[i] << "\n";
    }
    return 13;
  }
  return 0;
}

int RunLayerContractTests() {
  using repulsor3d::ArchitectureLayer;
  if (!repulsor3d::IsLayerDependencyAllowed(ArchitectureLayer::Render, ArchitectureLayer::Domain)) {
    return 10;
  }
  if (repulsor3d::IsLayerDependencyAllowed(ArchitectureLayer::Core, ArchitectureLayer::App)) {
    return 11;
  }
  if (const int rc = RunLayerIncludeBoundaryTests(); rc != 0) {
    return rc;
  }
  return 0;
}

int RunSeasonDefinitionTests() {
  repulsor3d::ViewerConfig cfg;
  auto season = repulsor3d::CreateDefaultSeasonDefinition(cfg);
  if (season == nullptr) {
    return 20;
  }
  auto builder = season->CreateSceneModelBuilder(cfg);
  if (builder == nullptr) {
    return 21;
  }
  return 0;
}

int RunRenderPipelineTests() {
  const auto defaultCfg = repulsor3d::MakeDefaultRenderPipelineConfig();
  if (defaultCfg.passes.empty()) {
    return 30;
  }
  const auto defaultValidation = repulsor3d::ValidateRenderPipelineConfig(defaultCfg);
  if (!defaultValidation.ok) {
    return 33;
  }

  const std::string path = "test_pipeline.json";
  {
    std::ofstream out(path);
    out << R"({"passes":[{"factory":"dummy","name":"dummy_pass","enabled":true}]})";
  }

  repulsor3d::RenderPipelineConfig loaded;
  std::string error;
  const bool ok = repulsor3d::LoadRenderPipelineConfigFromFile(path, loaded, &error);
  std::remove(path.c_str());
  if (!ok || loaded.passes.size() != 1) {
    return 31;
  }

  repulsor3d::RenderFeatureFactoryRegistry registry;
  registry.Register("dummy", [](const repulsor3d::RenderPipelinePassSpec& spec) {
    return std::make_unique<DummyFeature>(spec.name);
  });
  auto features = repulsor3d::BuildRenderFeaturesFromPipeline(loaded, registry);
  if (features.size() != 1 || features.front()->Name() != "dummy_pass") {
    return 32;
  }

  repulsor3d::RenderPipelineConfig invalid;
  invalid.passes.push_back({.factory = "dummy", .name = "p0", .dependencies = {"missing"}, .renderPass = "", .enabled = true});
  const auto invalidValidation = repulsor3d::ValidateRenderPipelineConfig(invalid);
  if (invalidValidation.ok) {
    return 34;
  }

  return 0;
}

int RunCoordinateServiceTests() {
  repulsor3d::CoordinateSystemService service;
  service.SetTransform(
      repulsor3d::CoordinateFrameId::Incoming,
      repulsor3d::CoordinateFrameId::Field,
      repulsor3d::CoordinateFrameMapper::Config{
          .originXM = 1.0,
          .originYM = 2.0,
          .rotationDeg = 0.0,
          .scaleMPerUnit = 2.0,
          .zScaleMPerUnit = 3.0,
      });

  const auto xy = service.TransformPointXY(repulsor3d::CoordinateFrameId::Incoming, repulsor3d::CoordinateFrameId::Field, 3.0, 4.0);
  if (std::abs(xy.x - 7.0F) > 1e-5F || std::abs(xy.y - 10.0F) > 1e-5F) {
    return 40;
  }
  const double z = service.TransformZ(repulsor3d::CoordinateFrameId::Incoming, repulsor3d::CoordinateFrameId::Field, 2.0);
  if (std::abs(z - 6.0) > 1e-9) {
    return 41;
  }
  return 0;
}

int RunTaskSchedulerTests() {
  repulsor3d::TaskScheduler scheduler;
  std::mutex orderMutex;
  std::vector<int> order;

  std::packaged_task<int()> lowTask([&]() {
    std::scoped_lock lock(orderMutex);
    order.push_back(1);
    return 1;
  });
  auto lowFuture = lowTask.get_future();

  std::packaged_task<int()> highTask([&]() {
    std::scoped_lock lock(orderMutex);
    order.push_back(2);
    return 2;
  });
  auto highFuture = highTask.get_future();

  scheduler.Start(1);
  scheduler.Enqueue(std::move(lowTask), repulsor3d::TaskScheduler::TaskPriority::Low);
  scheduler.Enqueue(std::move(highTask), repulsor3d::TaskScheduler::TaskPriority::High);

  if (lowFuture.wait_for(std::chrono::seconds(2)) != std::future_status::ready ||
      highFuture.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    scheduler.Stop();
    return 50;
  }
  if (order.size() != 2) {
    scheduler.Stop();
    return 51;
  }

  auto cancellationToken = repulsor3d::TaskScheduler::CancellationToken::Create();
  cancellationToken.Cancel();
  std::packaged_task<int()> canceledTask([]() { return 99; });
  auto canceledFuture = canceledTask.get_future();
  scheduler.Enqueue(std::move(canceledTask), repulsor3d::TaskScheduler::TaskPriority::Normal, cancellationToken);

  if (canceledFuture.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    scheduler.Stop();
    return 52;
  }
  bool canceledAsBrokenPromise = false;
  try {
    (void)canceledFuture.get();
  } catch (...) {
    canceledAsBrokenPromise = true;
  }
  if (!canceledAsBrokenPromise) {
    scheduler.Stop();
    return 53;
  }

  scheduler.Stop();
  return 0;
}

int RunResourceManagerTests() {
  repulsor3d::ResourceLifetimeManager manager;
  manager.SetBudget(repulsor3d::ResourceClass::Texture, repulsor3d::ResourceBudget{.maxCount = 1, .maxBytes = 128});
  manager.Register(repulsor3d::ResourceClass::Texture, 96);
  if (manager.IsOverBudget(repulsor3d::ResourceClass::Texture)) {
    return 60;
  }
  manager.Register(repulsor3d::ResourceClass::Texture, 96);
  if (!manager.IsOverBudget(repulsor3d::ResourceClass::Texture)) {
    return 61;
  }

  manager.Register(repulsor3d::ResourceClass::Texture, "tex.old", 80);
  manager.Register(repulsor3d::ResourceClass::Texture, "tex.new", 80);
  manager.Touch(repulsor3d::ResourceClass::Texture, "tex.new");
  const auto evict = manager.CollectEvictionCandidates(repulsor3d::ResourceClass::Texture, 2);
  if (evict.empty() || evict.front().handleId != "tex.old") {
    return 62;
  }
  const bool evicted = manager.EvictOne(repulsor3d::ResourceClass::Texture, [](const auto&) { return true; });
  if (!evicted) {
    return 63;
  }
  if (manager.OverBudgetBytes(repulsor3d::ResourceClass::Texture) == 0) {
    return 64;
  }
  return 0;
}

int RunPluginSdkTests() {
  repulsor3d::PluginSdkInfoV1 sdk;
  if (!repulsor3d::IsPluginSdkCompatible(sdk, repulsor3d::kPluginSdkVersion)) {
    return 70;
  }
  sdk.minHostSdkVersion = repulsor3d::kPluginSdkVersion + 1;
  if (repulsor3d::IsPluginSdkCompatible(sdk, repulsor3d::kPluginSdkVersion)) {
    return 71;
  }
  sdk.minHostSdkVersion = repulsor3d::kPluginSdkVersion;
  sdk.sdkFlavor = nullptr;
  if (repulsor3d::IsPluginSdkCompatible(sdk, repulsor3d::kPluginSdkVersion)) {
    return 72;
  }
  return 0;
}

int RunDiagnosticsMessageTests() {
  repulsor3d::DiagnosticsService diagnostics;
  diagnostics.BeginFrame();
  diagnostics.RecordMessage("hot reload ok");
  diagnostics.EndFrame(16.0);
  const auto& latest = diagnostics.Latest();
  if (latest.messages.empty() || latest.messages.front() != "hot reload ok") {
    return 80;
  }
  return 0;
}

}  // namespace

int main() {
  if (const int rc = RunLayerContractTests(); rc != 0) {
    std::cerr << "Layer contract tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunSeasonDefinitionTests(); rc != 0) {
    std::cerr << "Season definition tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunRenderPipelineTests(); rc != 0) {
    std::cerr << "Render pipeline tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunCoordinateServiceTests(); rc != 0) {
    std::cerr << "Coordinate service tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunTaskSchedulerTests(); rc != 0) {
    std::cerr << "Task scheduler tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunResourceManagerTests(); rc != 0) {
    std::cerr << "Resource manager tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunPluginSdkTests(); rc != 0) {
    std::cerr << "Plugin SDK tests failed: " << rc << "\n";
    return rc;
  }
  if (const int rc = RunDiagnosticsMessageTests(); rc != 0) {
    std::cerr << "Diagnostics message tests failed: " << rc << "\n";
    return rc;
  }

  std::cout << "Abstraction contract tests passed\n";
  return 0;
}
