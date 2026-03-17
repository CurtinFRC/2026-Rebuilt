# Repulsor 3D Sim (C++)

This is a full C++ rewrite of `repulsor_3d_sim` with:
- modern OpenGL (core profile + shaders)
- stronger object-oriented structure
- background snapshot worker
- truth socket receiver
- optional NT4 datasource (when `ntcore` is available)

## Build

```powershell
cd repulsor_3d_sim_cpp
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
