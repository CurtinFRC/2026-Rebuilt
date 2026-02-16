## Licensing

This repository is licensed under the MIT License (see `LICENSE.md`) **except** for:

- `src/main/org/curtinfrc/frc2026/util/Repulsor`

Repulsor is under the GNU GENERAL PUBLIC LICENSE, but is **copyright (c) 2026 Paul Hodges**
and includes its own license file at:

- `src/main/org/curtinfrc/frc2026/util/Repulsor/LICENSE.md`

## Subprojects

Repulsor is the autonomous planning and field-intelligence subsystem for this robot code.
See [https://github.com/CurtinFRC/Repulsor/blob/main/src/main/java/org/curtinfrc/frc2026/util/Repulsor/README.md](https://github.com/CurtinFRC/Repulsor/blob/main/src/main/java/org/curtinfrc/frc2026/util/Repulsor/README.md) for architecture and usage notes.

## Offload Compute System

### Module layout

- `:` (root robot project, GradleRIO/WPILib)
- `:offload-api`
- `:offload-processor`
- `:offload-client`
- `:offload-server`
- `:offload-tasks`

### What this adds

- `@Offloadable` annotation + annotation processor generated wrappers (`*_Offloaded`)
- `CompletableFuture` async wrappers with timeout + fallback
- TCP RPC protocol with request/response correlation IDs
- HELLO/PING health handshake with manifest hash
- Runtime client with bounded queue, backpressure, and host failover
- Dynamic server task loading from `/opt/offload/plugins/*.jar` via `ServiceLoader`
- Generated manifest (`offload-manifest.json`) and constants (`OffloadIds`)
- Example offload integration for DragShot planner via DTO boundary in:
  - `src/main/java/org/curtinfrc/frc2026/util/Repulsor/Offload`

### Build tasks

- Build tasks plugin jar:
  - `./gradlew buildOffloadTasksJar`
- Generate manifest into `build/offload/offload-manifest.json`:
  - `./gradlew generateOffloadManifest`
- Deploy to coprocessors and restart service:
  - `./gradlew deployOffloadToCoprocessors -PoffloadHosts=10.xx.yy.11,10.xx.yy.12`

Optional deploy properties:

- `-PoffloadUser=lvuser`
- `-PoffloadPluginDir=/opt/offload/plugins`
- `-PoffloadServiceName=offload-server`

### Run server on coprocessor

1. Copy `offload-tasks.jar` and `offload-manifest.json` into `/opt/offload/plugins/`.
2. Run the server app (`:offload-server`) as a JVM process/service.
3. Environment variables supported:
   - `OFFLOAD_PORT` (default `5808`)
   - `OFFLOAD_PLUGIN_DIR` (default `/opt/offload/plugins`)
   - `OFFLOAD_WORKERS` (default `4`)
   - `OFFLOAD_SERVER_NAME` (default `repulsor-offload`)
   - `OFFLOAD_SERVER_VERSION` (default `1.0.0`)

### Robot runtime configuration

The robot offload runtime is initialized via:

- `org.curtinfrc.frc2026.util.Repulsor.Offload.RepulsorOffloadRuntime`

Host configuration:

- JVM property `repulsor.offload.hosts=host1:5808,host2:5808`
- or env var `REPULSOR_OFFLOAD_HOSTS`

### Writing new offloadable methods

1. Add a DTO request/response pair in:
   - `src/main/java/org/curtinfrc/frc2026/util/Repulsor/Offload`
2. Add a static entrypoint method and annotate with `@Offloadable`.
3. Call the generated wrapper (`*_Offloaded`) from robot call sites.
4. Add an `OffloadFunction` implementation in `:offload-tasks` and register it in:
   - `offload-tasks/src/main/resources/META-INF/services/org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction`
5. Rebuild plugin + manifest and deploy.

### Timeout and async guidance

- Keep sync wrapper timeout small (10-30 ms typical for loop-safe behavior).
- Use generated async wrappers in periodic flows that can tolerate eventual completion.
- Use `OffloadCached<T>` to return last-known-good values while an updated async request is in flight.
- When outbound queue is full, calls fail fast and wrappers fallback locally.

### Testing

Implemented tests include:

- DTO conversion test
- Manifest ID stability test
- Client/server roundtrip test
- Integration test with local server and fallback validation when server is down
