# Repulsor 3D Sim (C++)

This is a full C++ rewrite of `repulsor_3d_sim` with:
- modern OpenGL (core profile + shaders)
- stronger object-oriented structure
- background snapshot worker
- truth socket receiver
- optional NT4 datasource (when `ntcore` is available)
- scene-model abstraction for season-specific game rendering
- hot-reload support for season module plugins + scene descriptors

## Build

If you want NT4 datasource support, first build native NTCore dependencies from a local `allwpilib` checkout.

### Build NTCore from allwpilib (required for NT4 datasource)

```powershell
git clone https://github.com/wpilibsuite/allwpilib.git C:\Users\paulh\allwpilib
cd C:\Users\paulh\allwpilib
.\gradlew.bat :wpiutil:build :wpinet:build :ntcore:build
```

The sim expects these outputs (or equivalent) under your `allwpilib` tree:
- `ntcore\build\libs\ntcore\shared\windowsx86-64\release\ntcore.lib` + `ntcore.dll`
- `wpinet\build\libs\wpinet\shared\windowsx86-64\release\wpinet.lib` + `wpinet.dll`
- `wpiutil\build\libs\wpiutil\shared\windowsx86-64\release\wpiutil.lib` + `wpiutil.dll`
- headers under:
  - `ntcore\src\main\native\include`
  - `ntcore\src\generated\main\native\include`
  - `wpinet\src\main\native\include`
  - `wpiutil\src\main\native\include`

Then build this project:

```powershell
cd repulsor_3d_sim_cpp
cmake -S . -B build -DREPULSOR_SIM_FETCH_DEPS=ON -DREPULSOR_SIM_USE_NTCORE=ON -DREPULSOR_SIM_NTCORE_ROOT=C:/Users/paulh/allwpilib
cmake --build build --config Release
```

## Run

```powershell
cd repulsor_3d_sim_cpp
.\build\Release\repulsor_3d_sim_cpp.exe
```

`field.png` is copied from `../repulsor_3d_sim/field.png` after build when present.

## Controls

- Left mouse drag: orbit camera
- Mouse wheel: zoom
- `R`: reset camera/follow state
- `C`: toggle camera debug rendering
- `T`: toggle truth fuel rendering
- `A`: toggle age-filtered fuel rendering
- `F`: toggle field texture image
- `Esc`: quit

## Renderer Architecture (Refactored)

The renderer is now split into a backend + season-model pipeline:

- `Renderer` (`src/Renderer.cpp`, `src/render/RendererResources.cpp`, `src/render/RendererText.cpp`):
  - owns GPU resources/shaders/meshes
  - draws generic primitives (`SpherePrimitive`, `BoxPrimitive`, `LinePrimitive`)
  - does not encode game-specific object logic
- `ISimWorld` + `IRenderWorldAdapter`:
  - world state boundary (`include/repulsor3d/sim/SimWorld.hpp`)
  - adapter boundary from world -> render frame (`include/repulsor3d/render/RenderWorldAdapter.hpp`)
  - renderer now consumes adapters, not season-specific builders directly
- `IRenderFeature` pipeline (`include/repulsor3d/render/RenderFeature.hpp`, `src/render/RenderFeature.cpp`):
  - pluggable render passes (default: world, primitives, CAD meshes, overlay)
  - allows adding new visuals (3D fields, custom post-processing, extra UI overlays) by adding feature modules instead of editing `Renderer::Draw`
- `ISeasonModule` plugin boundary (`include/repulsor3d/modules/SeasonModule.hpp`):
  - season profile resolves to a module
  - module provides the world adapter for that season (`2026Rebuilt` built-in)
  - supports dynamic plugin module loading via `SEASON_MODULE_PLUGIN_PATH`
- Data-driven `SceneDescriptor` (`include/repulsor3d/render/SceneDescriptor.hpp`):
  - optional JSON overrides for draw flags and static overlay widgets
  - loaded from `SCENE_DESCRIPTOR_PATH` or `assets/scenes/<sceneProfile>.json`
  - supports static primitives/meshes/entities for season-specific data-driven scene composition
- Multi-pass feature pipeline:
  - default pipeline is now staged: `world -> geometry_opaque -> cad_opaque -> geometry_transparent -> cad_transparent -> overlay`
  - per-primitive `RenderPass` tagging controls pass placement
- ECS-style entity registry:
  - `include/repulsor3d/render/ecs/EntityRegistry.hpp`
  - entity payloads can be converted into render commands without direct primitive vectors
- ECS systems for transform hierarchy + frustum culling:
  - `include/repulsor3d/render/ecs/Systems.hpp`
  - entities support optional parent linkage and local transforms
  - culling bounds are per-entity and run before render command generation
- Overlay layout engine:
  - overlay widgets support `TopLeft`, `TopRight`, `BottomLeft`, `BottomRight` anchors
  - smoothed FPS widget is rendered via the same overlay widget path (top-right)
- Asset pipeline abstraction for CAD:
  - `ICadMeshImporter`, `ICadMeshCooker`, `ICadMeshCache`, `CadMeshAssetPipeline`
  - default multi-format importer supports STL / OBJ / glTF (`.gltf`) / binary glTF (`.glb`)
  - default cache is in-memory + disk-backed (`.repulsor_cache/cad_mesh`)
- Telemetry and diagnostics:
  - per-pass CPU timings
  - GPU timing abstraction via backend timer queries
  - CAD asset telemetry (cache lookup/import/cook/store/total timings)
- Backend abstraction expansion:
  - `IRenderBackend` now abstracts uniforms plus common buffer/texture/vertex-attrib upload paths
  - renderer setup/upload code migrated off direct OpenGL calls where practical
- `CadModelRenderFeature` (`include/repulsor3d/render/CadModelRenderFeature.hpp`, `src/render/CadModelRenderFeature.cpp`):
  - renders generic STL CAD instances from `RenderSceneFrame::meshInstances`
  - supports both static field CAD and dynamic robot/goal CAD supplied by scene builders
- `ISceneModelBuilder` (`include/repulsor3d/render/SceneModelBuilder.hpp`):
  - converts runtime snapshot data into a `RenderSceneFrame`
- `Season2026RebuiltModelBuilder` (`src/render/Season2026RebuiltModelBuilder.cpp`):
  - current `2026Rebuilt` game-specific mapping for fuel/robots/cameras/overlay
  - default field CAD model path: `field_2026rebuilt.gltf` (field image rendering currently disabled in this season builder)
- `SceneModelBuilderFactory` (`src/render/SceneModelBuilderFactory.cpp`):
  - resolves `SIM_SCENE_PROFILE` to a concrete scene model builder
- Generic scaffold:
  - `include/repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp`
  - `include/repulsor3d/render/templates/GenericRenderFeatureTemplate.hpp`

To support next season, add a new `ISceneModelBuilder` implementation and provide it to `Renderer` via `SetSceneModelBuilder(...)` (or constructor injection), without touching OpenGL draw code.

To add new graphics systems, implement `IRenderFeature` and install via `Renderer::SetRenderFeatures(...)`; the scene model and renderer core stay unchanged.

## NT4 Backend

NTCore discovery order:
- first: CMake package (`find_package(ntcore CONFIG)`) under `REPULSOR_SIM_NTCORE_ROOT`
- fallback: raw allwpilib layout (headers + `.lib`/`.dll` from `ntcore/wpinet/wpiutil` build outputs)

If NTCore is found at configure time, the app builds with NT4 input support.
If not found, it falls back to `NullDataSource` (renderer still runs).

## Environment Variables

The C++ app accepts the same key env vars used by the Python sim, including:

- `NT_SERVER`, `NT_CLIENT_NAME`
- `NT_FIELDVISION_PATH`, `NT_REPULSORVISION_PATH`
- `NT_POSE_BASE_PATH`, `NT_POSE_STRUCT_KEY`
- `TRUTH_SOCKET_ENABLED`, `TRUTH_SOCKET_HOST`, `TRUTH_SOCKET_PORT`
- `WINDOW_W`, `WINDOW_H`, `FPS`
- `FIELD_IMAGE_PATH`, `SHOW_FIELD_IMAGE`, `FIELD_IMAGE_ALPHA`, `FIELD_IMAGE_FLIP_X`, `FIELD_IMAGE_FLIP_Y`
- `FOLLOW_ROBOT`, `SHOW_CAMERA_DEBUG`, `SHOW_TRUTH_FUEL`, `SHOW_AGE_FILTERED_FUEL`
- `SHOW_ROBOT_CAD_MODEL`, `ROBOT_CAD_MODEL_PATH`, `ROBOT_CAD_SCALE_M`, `ROBOT_CAD_Z_OFFSET_M`
- `SHOW_FIELD_CAD_MODEL`, `FIELD_CAD_MODEL_PATH`, `FIELD_CAD_SCALE_M`, `FIELD_CAD_FLIP_X`, `FIELD_CAD_Z_OFFSET_M`
- `CAD_LOD_COUNT` (default `2`)
- `CAD_LOD_RATIO` (default `0.62`)
- `CAD_LOD_MIN_VERTICES` (default `700000`)
- `CAD_LOD0_MAX_VERTICES` (default `4500000`)
- `CAD_LOD_KEEP_FULL` (default `true`)
- `CAD_LOD_MAX_DRAW_INDICES` (default `8500000`)
- `CAD_LOD_NORMAL_BINS` (default `20`)
- `CAD_LOD_PRESERVE_FLAT` (default `true`)
- `CAD_LOD0_SCREEN_RADIUS_PX` (default `1200`)
- `CAD_LOD_SCREEN_RADIUS_DECAY` (default `0.5`)
- `CAD_PREPARED_CACHE_DIR` (optional override for cached prepared CAD LOD data; default is under `%LOCALAPPDATA%/repulsor_3d_sim_cpp/cad_prepared_cache` on Windows)
- `CAD_FRUSTUM_CULLING` (default `true`)
- `CAD_KEY_LIGHT_INTENSITY` (default `1.35`)
- `CAD_FILL_LIGHT_INTENSITY` (default `0.20`)
- `CAD_AMBIENT_STRENGTH` (default `0.12`)
- `CAD_DEPTH_CUE_STRENGTH` (default `0.08`)
- `CAD_SPECULAR_STRENGTH` (default `0.42`)
- `CAD_RIM_STRENGTH` (default `0.03`)
- `CAD_ROUGHNESS` (default `0.72`)
- `CAD_METALLIC` (default `0.00`)
- `CAD_EXPOSURE` (default `0.86`)
- `CAD_FOG_DENSITY` (default `0.003`)
- `CAD_SATURATION` (default `1.00`)
- `CAD_GAMMA` (default `2.2`)
- `SIM_SCENE_PROFILE` (defaults to `2026Rebuilt`)
- `SCENE_DESCRIPTOR_PATH` (optional JSON descriptor path)
- `SEASON_MODULE_PLUGIN_PATH` (optional path to season module plugin `.dll`/`.so`)
- `HOT_RELOAD_SCENE_DESCRIPTOR` (default `true`)
- `HOT_RELOAD_SEASON_MODULE` (default `true`)

### CAD LOD Pipeline

CAD meshes now go through an abstract multi-stage path in `CadModelRenderFeature`:
1. triangle-soup indexing + dedup
2. degenerate-triangle cleanup
3. shape-preserving vertex-cluster simplification
4. multi-LOD chain generation
5. screen-space + draw-budget LOD selection at render time

This is season-agnostic and applies to any mesh instance (including `2026Rebuilt` field CAD).

## Plugin SDK + ABI Checks

- Sample plugin SDK template source:
  - `plugin_sdk/GenericSeasonPluginTemplate.cpp`
  - `plugin_sdk/CMakeLists.txt`
- ABI contract now requires plugin export:
  - `repulsor3d_query_season_module_abi_version`
  - `repulsor3d_create_season_module`
  - `repulsor3d_destroy_season_module`
- CTest includes plugin ABI smoke test:
  - `repulsor_3d_sim_cpp_plugin_abi_tests`
- CMake target for CI-style ABI check:

```powershell
cmake --build build --config Release --target repulsor_plugin_abi_check
```

## Clangd / LSP

This folder includes:
- `.clangd`
- `compile_flags.txt`

These provide a fallback compile configuration for clangd so IntelliSense/LSP does not report spurious errors when `compile_commands.json` is unavailable.
