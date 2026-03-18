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

You can also use presets:

```powershell
cmake --preset release
cmake --build --preset build-release
ctest --preset test-release
```

Image regression harness (CPU top-down raster, deterministic golden hash):

```powershell
ctest --test-dir build -C Release --output-on-failure -R repulsor_3d_sim_cpp_render_image_regression_tests
```

To refresh the golden hash after intentional visual-model changes:

```powershell
$env:REPULSOR_UPDATE_IMAGE_GOLDEN="1"
.\build\Release\repulsor_3d_sim_cpp_render_image_regression_tests.exe
Remove-Item Env:REPULSOR_UPDATE_IMAGE_GOLDEN
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
- `D`: toggle debug panel
- `1`: toggle debug counters section
- `2`: toggle debug CPU timings section
- `3`: toggle debug GPU timings section
- `4`: toggle debug assets section
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
- Coordinate-frame mapper:
  - `CoordinateFrameMapper` (`include/repulsor3d/domain/CoordinateFrameMapper.hpp`)
  - domain-layer transform maps incoming coordinates to render-space meters before scene building
  - keeps season builders focused on visuals, not frame-conversion math
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
  - includes structural JSON validation before load (invalid descriptor shapes are rejected)
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
- Scene graph builder abstraction:
  - `include/repulsor3d/render/scenegraph/SceneGraphBuilder.hpp`
  - season builders can compose parent/child render entities through a generic builder API (used by `2026Rebuilt`)
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

## Data Source Modes

Datasource selection is now configurable via `DATA_SOURCE_TYPE`:
- `auto` (default): uses replay if `REPLAY_SNAPSHOT_PATH` is set, otherwise NT (or null if NT not compiled)
- `nt`: force NetworkTables source
- `replay`: force replay source
- `null`: force no live input

Optional plugin override:
- `DATA_SOURCE_PLUGIN_PATH` (if set and valid, plugin source is used before built-ins)
- `RENDER_FEATURE_PLUGIN_PATH` (if set and valid, plugin render features are appended to the default renderer feature stack)

Replay/record env vars:
- `REPLAY_SNAPSHOT_PATH` (NDJSON snapshot log path)
- `REPLAY_LOOP` (`true`/`false`)
- `RECORD_SNAPSHOT_PATH` (write NDJSON snapshots while running)

Each line in replay/record files is one JSON `SnapshotBundle`.

## NT Schema File (Data-Driven + Hot Reload)

You can override default NT stream schemas from a JSON file:
- `NT_SCHEMA_PATH` (schema override file path)
- `HOT_RELOAD_NT_SCHEMA` (default `true`)

Supported top-level keys:
- `fieldVision`
- `repulsor`
- `cameras`

Each group may include:
- `tablePrefix`, `idPrefix`
- `aliveField` object
- `fields` array of field objects (`key`, `topicSuffix`, `type`, defaults)
- `autoCaptureAdditionalScalars`, `extraKeyPrefix`

When hot reload is enabled, changes to this file are picked up at runtime.
Schema files are structurally validated before applying overrides.

## Environment Variables

The C++ app accepts the same key env vars used by the Python sim, including:

- `NT_SERVER`, `NT_CLIENT_NAME`
- `DATA_SOURCE_TYPE` (`auto|nt|replay|null`)
- `DATA_SOURCE_PLUGIN_PATH`
- `REPLAY_SNAPSHOT_PATH`, `REPLAY_LOOP`, `RECORD_SNAPSHOT_PATH`
- `NT_FIELDVISION_PATH`, `NT_REPULSORVISION_PATH`
- `NT_POSE_BASE_PATH`, `NT_POSE_STRUCT_KEY`
- `NT_SCHEMA_PATH`, `HOT_RELOAD_NT_SCHEMA`
- `TRUTH_SOCKET_ENABLED`, `TRUTH_SOCKET_HOST`, `TRUTH_SOCKET_PORT`
- `WINDOW_W`, `WINDOW_H`, `FPS`
- `VSYNC` (default `true`; set `false` to disable swap-interval lock)
- `FIELD_IMAGE_PATH`, `SHOW_FIELD_IMAGE`, `FIELD_IMAGE_ALPHA`, `FIELD_IMAGE_FLIP_X`, `FIELD_IMAGE_FLIP_Y`
- `FOLLOW_ROBOT`, `SHOW_CAMERA_DEBUG`, `SHOW_TRUTH_FUEL`, `SHOW_AGE_FILTERED_FUEL`
- `SHOW_ROBOT_CAD_MODEL`, `ROBOT_CAD_MODEL_PATH`, `ROBOT_CAD_SCALE_M`, `ROBOT_CAD_Z_OFFSET_M`
- `SHOW_FIELD_CAD_MODEL`, `FIELD_CAD_MODEL_PATH`, `FIELD_CAD_SCALE_M`, `FIELD_CAD_FLIP_X`, `FIELD_CAD_Z_OFFSET_M`
- `MAX_RENDER_FUEL` (default `400`; cap rendered field-vision fuel markers)
- `MAX_RENDER_TRUTH_FUEL` (default `400`; cap rendered truth fuel markers)
- `MAX_CAMERA_DEBUG_RAYS_PER_CAMERA` (default `600`; cap per-camera debug rays)
- `CAD_PERF_AUTO_QUALITY` (default `false`; adaptive CAD quality based on frame time)
- `CAD_PERF_TARGET_FRAME_MS` (default `16.7`; adaptive target frame time)
- `CAD_AUTO_MAX_LOD_BIAS` (default `0`; max automatic CAD LOD step-down)
- `CAD_SHADOW_LOD_BIAS` (default `1`; shadow-pass LOD bias to reduce shadow cost)
- `CAD_SHADOW_CAST_MAX_INDICES` (default `0`; when >0, meshes above this index count do not cast dynamic shadows)
- `CAD_FAST_SHADING_MIN_INDICES` (default `0`; optional large-mesh cheaper shading path, disabled by default)
- `CAD_RUNTIME_MAX_DRAW_INDICES` (default `3000000`; runtime cap for selected CAD LOD draw index count for non-field meshes)
- `CAD_RUNTIME_MAX_DRAW_INDICES_FIELD` (default `1800000`; stricter runtime cap for field-like meshes)
- `CAD_TARGET_MAX_LAST_LOD_INDICES` (default `1800000`; cook-time target for the deepest generated CAD LOD)
- `CAD_SHADOW_PROXY_MAX_VERTICES` (default `180000`; max vertices for dedicated shadow-only proxy mesh)
- `SHOW_DEBUG_PANEL` (default `true`; enables diagnostics counters/panel)
- `INCOMING_COORD_FRAME` (default `top_right_negative`; `custom` to use affine params below)
- `INCOMING_COORD_CALIBRATION_PROFILE_PATH` (optional JSON calibration profile file path)
- `INCOMING_COORD_CALIBRATION_PROFILE_NAME` (optional profile key inside calibration file; default `default`)
- `INCOMING_COORD_ORIGIN_X_M` (used when `INCOMING_COORD_FRAME=custom`, default `0.0`)
- `INCOMING_COORD_ORIGIN_Y_M` (used when `INCOMING_COORD_FRAME=custom`, default `0.0`)
- `INCOMING_COORD_ROTATION_DEG` (default `0.0`)
- `INCOMING_COORD_SCALE_M_PER_UNIT` (default `1.0`)
- `INCOMING_COORD_Z_SCALE_M_PER_UNIT` (default `1.0`)
- `CAD_LOD_COUNT` (default `4`)
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
- `CAD_UPLOADS_PER_FRAME` (default `1`; caps CAD GPU uploads per frame to reduce hitching on heavy assets)
- `CAD_CANCEL_PENDING_LOADS` (default `false`; when `true`, marks pending CAD loads for cancellation)
- `CAD_AUTO_RETRY_FAILED_LOADS` (default `false`; retries failed CAD loads when requested again)
- `CAD_FRUSTUM_CULLING` (default `true`)
- `CAD_BROADPHASE_ENABLED` (default `true`; enables CAD broadphase prefilter before frustum tests)
- `CAD_BROADPHASE_CELL_SIZE_M` (default `3.0`; broadphase uniform-grid cell size)
- `CAD_SHADOW_QUALITY` (`low|medium|high|ultra`, default `high`)
- `CAD_SHADOW_CASCADES` (default `1`; scaffold path supports up to `2`)
- `CAD_SHADOW_CASCADE_SPLIT_M` (default `12.0`; split distance for near/far cascade selection)
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

For `INCOMING_COORD_FRAME=top_right_negative`, incoming coordinates map like:
- `(0, 0)` -> field top-right in centered grid coordinates
- `(-FIELD_LENGTH_M, -FIELD_WIDTH_M)` -> field bottom-left
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

### Dynamic NT Channels

`NT_SCHEMA_PATH` now supports generic runtime channels:

```json
{
  "channels": [
    {
      "channel": "robot_debug",
      "tablePrefix": "/Debug/Robot",
      "idPrefix": "entry_",
      "aliveField": {"key":"alive","topicSuffix":"","type":"boolean"},
      "fields": [{"key":"label","topicSuffix":"label","type":"string"}]
    }
  ]
}
```

Those channels are appended into `WorldSnapshot::dynamicEntityGroups[channel]` as key/value records without new C++ topic wiring.

## Season Scaffolding

Generate a new season model-builder scaffold:

```powershell
.\tools\new-season.ps1 -SeasonName Season2027Prototype
```

This creates:
- `include/repulsor3d/render/Season2027PrototypeModelBuilder.hpp`
- `src/render/Season2027PrototypeModelBuilder.cpp`

## Plugin SDK + ABI Checks

- Sample plugin SDK template source:
  - `plugin_sdk/GenericSeasonPluginTemplate.cpp`
  - `plugin_sdk/GenericDataSourcePluginTemplate.cpp`
  - `plugin_sdk/GenericRenderFeaturePluginTemplate.cpp`
  - `plugin_sdk/CMakeLists.txt`
- ABI contract now requires plugin export:
  - `repulsor3d_query_season_module_abi_version`
  - `repulsor3d_create_season_module`
  - `repulsor3d_destroy_season_module`
- Optional but recommended ABI metadata export (used for plugin-kind/version negotiation):
  - `repulsor3d_query_plugin_manifest_v1`
- CTest includes plugin ABI smoke test:
  - `repulsor_3d_sim_cpp_plugin_abi_tests`
  - `repulsor_3d_sim_cpp_datasource_plugin_abi_tests`
- CMake target for CI-style ABI check:

```powershell
cmake --build build --config Release --target repulsor_plugin_abi_check
```

Performance smoke test target:

```powershell
ctest --test-dir build -C Release --output-on-failure -R repulsor_3d_sim_cpp_perf_tests
```

Render benchmark target:

```powershell
.\tools\benchmark.ps1 -Config Release
```

Headless regression target:

```powershell
ctest --test-dir build -C Release --output-on-failure -R repulsor_3d_sim_cpp_render_regression_tests
```

## Clangd / LSP

This folder includes:
- `.clangd`
- `compile_flags.txt`

These provide a fallback compile configuration for clangd so IntelliSense/LSP does not report spurious errors when `compile_commands.json` is unavailable.
