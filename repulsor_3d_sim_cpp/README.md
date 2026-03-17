# Repulsor 3D Sim (C++)

This is a full C++ rewrite of `repulsor_3d_sim` with:
- modern OpenGL (core profile + shaders)
- stronger object-oriented structure
- background snapshot worker
- truth socket receiver
- optional NT4 datasource (when `ntcore` is available)
- scene-model abstraction for season-specific game rendering

## Build

```powershell
cd repulsor_3d_sim_cpp
  cmake -S . -B build `
    -DCMAKE_TOOLCHAIN_FILE=C:/Users/paulh/vcpkg/scripts/buildsystems/vcpkg.cmake `
    -DVCPKG_TARGET_TRIPLET=x64-windows `
    -DREPULSOR_SIM_USE_NTCORE=ON
    -DREPULSOR_SIM_FETCH_DEPS=ON
    
cmake -S . -B build -DREPULSOR_SIM_FETCH_DEPS=ON -DREPULSOR_SIM_USE_NTCORE=ON
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
  - owns OpenGL resources/shaders/meshes
  - draws generic primitives (`SpherePrimitive`, `BoxPrimitive`, `LinePrimitive`)
  - does not encode game-specific object logic
- `IRenderFeature` pipeline (`include/repulsor3d/render/RenderFeature.hpp`, `src/render/RenderFeature.cpp`):
  - pluggable render passes (default: world, primitives, CAD meshes, overlay)
  - allows adding new visuals (3D fields, custom post-processing, extra UI overlays) by adding feature modules instead of editing `Renderer::Draw`
- `CadModelRenderFeature` (`include/repulsor3d/render/CadModelRenderFeature.hpp`, `src/render/CadModelRenderFeature.cpp`):
  - renders generic STL CAD instances from `RenderSceneFrame::meshInstances`
  - supports both static field CAD and dynamic robot/goal CAD supplied by scene builders
- `ISceneModelBuilder` (`include/repulsor3d/render/SceneModelBuilder.hpp`):
  - converts runtime snapshot data into a `RenderSceneFrame`
- `Season2026RebuiltModelBuilder` (`src/render/Season2026RebuiltModelBuilder.cpp`):
  - current `2026Rebuilt` game-specific mapping for fuel/robots/cameras/overlay
- `SceneModelBuilderFactory` (`src/render/SceneModelBuilderFactory.cpp`):
  - resolves `SIM_SCENE_PROFILE` to a concrete scene model builder
- Generic scaffold:
  - `include/repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp`
  - `include/repulsor3d/render/templates/GenericRenderFeatureTemplate.hpp`

To support next season, add a new `ISceneModelBuilder` implementation and provide it to `Renderer` via `SetSceneModelBuilder(...)` (or constructor injection), without touching OpenGL draw code.

To add new graphics systems, implement `IRenderFeature` and install via `Renderer::SetRenderFeatures(...)`; the scene model and renderer core stay unchanged.

## NT4 Backend

If `ntcore` is found at configure time, the app builds with NT4 input support.
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
- `SHOW_FIELD_CAD_MODEL`, `FIELD_CAD_MODEL_PATH`, `FIELD_CAD_SCALE_M`, `FIELD_CAD_Z_OFFSET_M`
- `SIM_SCENE_PROFILE` (defaults to `2026Rebuilt`)

## Clangd / LSP

This folder includes:
- `.clangd`
- `compile_flags.txt`

These provide a fallback compile configuration for clangd so IntelliSense/LSP does not report spurious errors when `compile_commands.json` is unavailable.
