# Repulsor

Repulsor is a field-aware autonomy and utility stack used by the `2026-Rebuilt` robot codebase.
It combines pathing, obstacle avoidance, objective selection, behavior orchestration, tracking, predictive scoring, shooting setpoints, and simulation support in one package.

## Overview

The primary runtime entry point is `Repulsor.java`.
It connects:

- Robot drive IO (`DriveRepulsor`)
- Motion planning (`FieldPlanner`)
- Dynamic obstacle sources (`VisionPlanner`, `RepulsorVision`)
- Behavior scheduling (`BehaviourManager`)
- Match-state reasoning (`Reasoning`)
- Field/objective tracking (`FieldTrackerCore` and `FieldVision`)
- Setpoint and shot selection (`Setpoints`, `Shooting`)

`Repulsor` exposes a command-focused API (`alignTo`, `within`, gate-following helpers) so this module can be dropped directly into WPILib command-based robot code.

## Simulating

Create a Python virtual environment:

```bash
python -m venv .venv
```

Activate it:

```bash
# Windows (PowerShell)
.\.venv\Scripts\Activate.ps1

# Windows (cmd)
.\.venv\Scripts\activate.bat

# macOS / Linux
source .venv/bin/activate
```

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

Run the main simulation (vision + game pieces on-field + basic physics):

```bash
python -m repulsor_sim.main
```

(Optional) Run the live 3D/debug visualiser (robot, cameras, game pieces, etc.):

```bash
python repulsor_3d_sim/run.py
```

Finally, run the WPILib robot simulation from your robot project:

```bash
./gradlew simulateJava
```

## Runtime Flow

At a high level, each cycle:

1. `Repulsor.update()` refreshes field tracking from any configured `FieldVision` feeds.
2. Tracked element state can update next scoring intent (`refreshNextScoreFromFieldTracker`).
3. `VisionPlanner` updates dynamic obstacle snapshots.
4. `BehaviourManager` selects one active behavior using flags from the configured reasoner.
5. Active behavior calls `FieldPlanner.calculate(...)` to produce a `RepulsorSample`.
6. `RepulsorSample` is converted to `ChassisSpeeds` and sent to `DriveRepulsor.runVelocity(...)`.

## Core Modules

- `Repulsor.java`: facade, command builders, goal/phase gate integration.
- `FieldPlanner/`: vector-field planning, wall and obstacle forces, staged goals, fallback rerouting, bypass integration.
- `FieldPlanner/Obstacles`: geometric obstacle primitives and gated attractors.
- `ReactiveBypass/`: short-horizon bypass runtime for local blockage, relatch logic, and pinned-mode handling.
- `ExtraPathing*`: collision tests, clear-path checks, and helper geometry.
- `Setpoints/`: alliance-aware setpoint system and contextual setpoint resolution.
- `Setpoints/Specific/_Rebuilt2026.java`: game-specific setpoints and dynamic hub shot pose solving.
- `Shooting/`: drag-based shot solver, shot library generation, and online refinement.
- `Tracking/`: field model, dynamic object ingestion, objective caches, collect planner runtime.
- `Predictive/`: ranking and scoring of future objective choices from dynamic world state.
- `Vision/` and `VisionPlanner.java`: adapters from vision detections to planner obstacles.
- `Behaviours/`: behavior contracts and concrete modes (`AutoPath`, `Shuttle`, `Defense`, `Test`).
- `Reasoning/`: signal-driven finite-state reasoning for behavior flags.
- `Fields/`: field definitions (`Rebuilt2026`, `Reefscape2025`) including obstacles, heatmaps, and objective layout.
- `Tuning/`: translational and turning tuning strategies.
- `DriverStation/`: NetworkTables-backed control/config layer for Repulsor runtime parameters and commands.
- `Simulation/`, `Metrics/`, `Profiler/`, `Target/`: supporting simulation models, telemetry, profiling, and targeting utilities.

## Creating And Registering Behaviours

Behaviours are now registered from robot code using `Repulsor.addBehaviour(...)` or
`Repulsor.addBehaviours(...)`.
`Repulsor.java` no longer hardcodes a fixed behaviour list.

Typical pattern:

1. Construct `Repulsor`.
2. Build any suppliers/triggers your behaviour needs.
3. Register behaviours.
4. Attach a reasoner with `repulsor.setReasoner(...)`.

Example:

```java
repulsor.addBehaviours(
    new DefenseBehaviour(30, defenseGoalSup, () -> 2.8),
    new ShuttleBehaviour(20, shuttleGoalSup, () -> 3.2),
    new TestBehaviour(),
    new AutoPathBehaviour(
        10,
        repulsor::isInScoringGate,
        repulsor::isInCollectingGate,
        repulsor::getNextScore,
        hpOptionsSup,
        atHPSup,
        hasPieceSup,
        sp -> sp.point().name(),
        () -> 3.5));

repulsor.setReasoner(reasoner);
```

If you need to rebuild the behaviour set at runtime, call `repulsor.clearBehaviours()` and re-add.

## Field And Objective Model

- `FieldDefinition` ties together three concerns: obstacle provider, heatmap provider, and objective layout provider.
- `Rebuilt2026` is the default field definition via `Constants.FIELD`.
- `FieldMapBuilder` constructs alliance-tagged `GameElement` objectives with category tags:
  - `kScore`
  - `kCollect`
  - `kEndgame`

This allows the planner and predictive layers to ask for mode-specific candidates while reusing one field map.

## Configuration And Control

Repulsor exposes runtime tuning and command channels through `NtRepulsorDriverStation`.
Default keys include:

- `clearance_scale`
- `repulsion_scale`
- `force_controller_override`

Default command channels include pose override, pose reset, and goal setpoint command endpoints under `/Repulsor/DriverStation`.

## Integration Notes

Typical wiring pattern:

```java
RepulsorDriverStationBootstrap.useDefaultNt();

Repulsor repulsor =
    new Repulsor(driveRepulsor, robotX, robotY, coralOffset, algaeOffset, hasPieceSupplier)
        .withVision(myVisionSource)
        .withShooterReleaseHeightMetersSupplier(shooterReleaseHeightMetersSupplier);
```

Then:

- Register behaviours through `repulsor.addBehaviour(...)` or `repulsor.addBehaviours(...)`.
- Set the reasoner via `repulsor.setReasoner(...)`.
- Call `repulsor.update()` during periodic execution.
- Use `repulsor.alignTo(...)` when you need direct command-based alignment to a setpoint.
- Use `repulsor.within(...)` triggers for arrival checks.

## Future

There are lots of things to be added


## Licence

Repulsor is licensed under the GNU GPLv3.

Copyright (c) 2026 Paul Hodges

See `LICENCE.md` for the full licence text.

## Acknowledgements

Repulsor's earliest vector-field prototype was inspired by a widely shared community approach used across many FRC codebases.
The current implementation has since been rewritten and expanded substantially.
