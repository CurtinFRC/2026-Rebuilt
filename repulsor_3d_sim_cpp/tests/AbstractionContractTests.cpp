#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>

#include "repulsor3d/Config.hpp"
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

int RunLayerContractTests() {
  using repulsor3d::ArchitectureLayer;
  if (!repulsor3d::IsLayerDependencyAllowed(ArchitectureLayer::Render, ArchitectureLayer::Domain)) {
    return 10;
  }
  if (repulsor3d::IsLayerDependencyAllowed(ArchitectureLayer::Core, ArchitectureLayer::App)) {
    return 11;
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
  scheduler.Start(1);

  std::packaged_task<int()> task([]() { return 42; });
  auto future = task.get_future();
  scheduler.Enqueue(std::move(task));

  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    scheduler.Stop();
    return 50;
  }
  if (future.get() != 42) {
    scheduler.Stop();
    return 51;
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

  std::cout << "Abstraction contract tests passed\n";
  return 0;
}
