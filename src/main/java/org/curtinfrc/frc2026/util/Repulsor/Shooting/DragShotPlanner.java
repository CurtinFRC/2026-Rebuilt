/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

public final class DragShotPlanner {

  public abstract static class GamePiecePhysics {
    public abstract double massKg();

    public abstract double crossSectionAreaM2();

    public abstract double dragCoefficient();

    public double airDensityKgPerM3() {
      return 1.225;
    }
  }

  public static final class Constraints {

    public enum ShotStyle {
      ANY,
      DIRECT,
      ARC
    }

    private final double minLaunchSpeedMetersPerSecond;
    private final double maxLaunchSpeedMetersPerSecond;
    private final double minLaunchAngleDeg;
    private final double maxLaunchAngleDeg;
    private final ShotStyle shotStyle;

    public Constraints(
        double minLaunchSpeedMetersPerSecond,
        double maxLaunchSpeedMetersPerSecond,
        double minLaunchAngleDeg,
        double maxLaunchAngleDeg) {
      this(
          minLaunchSpeedMetersPerSecond,
          maxLaunchSpeedMetersPerSecond,
          minLaunchAngleDeg,
          maxLaunchAngleDeg,
          ShotStyle.ANY);
    }

    public Constraints(
        double minLaunchSpeedMetersPerSecond,
        double maxLaunchSpeedMetersPerSecond,
        double minLaunchAngleDeg,
        double maxLaunchAngleDeg,
        ShotStyle shotStyle) {
      this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
      this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
      this.minLaunchAngleDeg = minLaunchAngleDeg;
      this.maxLaunchAngleDeg = maxLaunchAngleDeg;
      this.shotStyle = shotStyle == null ? ShotStyle.ANY : shotStyle;
    }

    public double minLaunchSpeedMetersPerSecond() {
      return minLaunchSpeedMetersPerSecond;
    }

    public double maxLaunchSpeedMetersPerSecond() {
      return maxLaunchSpeedMetersPerSecond;
    }

    public double minLaunchAngleDeg() {
      return minLaunchAngleDeg;
    }

    public double maxLaunchAngleDeg() {
      return maxLaunchAngleDeg;
    }

    public ShotStyle shotStyle() {
      return shotStyle;
    }
  }

  public static final class ShotSolution {
    private final Translation2d shooterPosition;
    private final Rotation2d shooterYaw;
    private final double launchSpeedMetersPerSecond;
    private final Rotation2d launchAngle;
    private final double timeToPlaneSeconds;
    private final Translation2d impactFieldPosition;
    private final double verticalErrorMeters;

    public ShotSolution(
        Translation2d shooterPosition,
        Rotation2d shooterYaw,
        double launchSpeedMetersPerSecond,
        Rotation2d launchAngle,
        double timeToPlaneSeconds,
        Translation2d impactFieldPosition,
        double verticalErrorMeters) {
      this.shooterPosition = shooterPosition;
      this.shooterYaw = shooterYaw == null ? Rotation2d.kZero : shooterYaw;
      this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
      this.launchAngle = launchAngle == null ? Rotation2d.kZero : launchAngle;
      this.timeToPlaneSeconds = timeToPlaneSeconds;
      this.impactFieldPosition =
          impactFieldPosition == null ? new Translation2d() : impactFieldPosition;
      this.verticalErrorMeters = verticalErrorMeters;
    }

    public Translation2d shooterPosition() {
      return shooterPosition;
    }

    public Rotation2d shooterYaw() {
      return shooterYaw;
    }

    public double launchSpeedMetersPerSecond() {
      return launchSpeedMetersPerSecond;
    }

    public Rotation2d launchAngle() {
      return launchAngle;
    }

    public double timeToPlaneSeconds() {
      return timeToPlaneSeconds;
    }

    public Translation2d impactFieldPosition() {
      return impactFieldPosition;
    }

    public double verticalErrorMeters() {
      return verticalErrorMeters;
    }
  }

  public static final class ShotLibraryEntry {
    private final Translation2d shooterPosition;
    private final double shooterYawRad;
    private final double launchSpeedMetersPerSecond;
    private final double launchAngleRad;
    private final double timeToPlaneSeconds;
    private final double verticalErrorMeters;

    public ShotLibraryEntry(
        Translation2d shooterPosition,
        double shooterYawRad,
        double launchSpeedMetersPerSecond,
        double launchAngleRad,
        double timeToPlaneSeconds,
        double verticalErrorMeters) {
      this.shooterPosition = shooterPosition;
      this.shooterYawRad = shooterYawRad;
      this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
      this.launchAngleRad = launchAngleRad;
      this.timeToPlaneSeconds = timeToPlaneSeconds;
      this.verticalErrorMeters = verticalErrorMeters;
    }

    public Translation2d shooterPosition() {
      return shooterPosition;
    }

    public double shooterYawRad() {
      return shooterYawRad;
    }

    public double launchSpeedMetersPerSecond() {
      return launchSpeedMetersPerSecond;
    }

    public double launchAngleRad() {
      return launchAngleRad;
    }

    public double timeToPlaneSeconds() {
      return timeToPlaneSeconds;
    }

    public double verticalErrorMeters() {
      return verticalErrorMeters;
    }
  }

  public static final class ShotLibrary {
    private final Translation2d targetFieldPosition;
    private final double targetHeightMeters;
    private final double shooterReleaseHeightMeters;
    private final double robotHalfLengthMeters;
    private final double robotHalfWidthMeters;
    private final Constraints constraints;
    private final List<ShotLibraryEntry> entries;
    private final boolean complete;

    public ShotLibrary(
        Translation2d targetFieldPosition,
        double targetHeightMeters,
        double shooterReleaseHeightMeters,
        double robotHalfLengthMeters,
        double robotHalfWidthMeters,
        Constraints constraints,
        List<ShotLibraryEntry> entries,
        boolean complete) {
      this.targetFieldPosition = targetFieldPosition;
      this.targetHeightMeters = targetHeightMeters;
      this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
      this.robotHalfLengthMeters = robotHalfLengthMeters;
      this.robotHalfWidthMeters = robotHalfWidthMeters;
      this.constraints = constraints;
      this.entries = entries;
      this.complete = complete;
    }

    public Translation2d targetFieldPosition() {
      return targetFieldPosition;
    }

    public double targetHeightMeters() {
      return targetHeightMeters;
    }

    public double shooterReleaseHeightMeters() {
      return shooterReleaseHeightMeters;
    }

    public double robotHalfLengthMeters() {
      return robotHalfLengthMeters;
    }

    public double robotHalfWidthMeters() {
      return robotHalfWidthMeters;
    }

    public Constraints constraints() {
      return constraints;
    }

    public List<ShotLibraryEntry> entries() {
      return entries;
    }

    public boolean complete() {
      return complete;
    }
  }

  public static final class ShotLibraryBuilder {
    private final GamePiecePhysics gamePiece;
    private final Translation2d targetFieldPosition;
    private final double targetHeightMeters;
    private final double shooterReleaseHeightMeters;
    private final double robotHalfLengthMeters;
    private final double robotHalfWidthMeters;
    private final Constraints constraints;

    private final double minSpeed;
    private final double maxSpeed;
    private final double minAngleDeg;
    private final double maxAngleDeg;
    private final boolean fixedAngle;

    private final double speedStep;
    private final double angleStepDeg;
    private final double radialStep;
    private final double bearingStepDeg;

    private final ArrayList<ShotLibraryEntry> entries = new ArrayList<>(768);

    private double range;
    private double bearingDeg;
    private boolean done;

    private int publishCounter;

    public ShotLibraryBuilder(
        GamePiecePhysics gamePiece,
        Translation2d targetFieldPosition,
        double targetHeightMeters,
        double shooterReleaseHeightMeters,
        double robotHalfLengthMeters,
        double robotHalfWidthMeters,
        Constraints constraints,
        double speedStep,
        double angleStepDeg,
        double radialStep,
        double bearingStepDeg) {
      this.gamePiece = gamePiece;
      this.targetFieldPosition = targetFieldPosition;
      this.targetHeightMeters = targetHeightMeters;
      this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
      this.robotHalfLengthMeters = robotHalfLengthMeters;
      this.robotHalfWidthMeters = robotHalfWidthMeters;
      this.constraints = constraints;

      this.minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      this.maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      this.minAngleDeg = constraints.minLaunchAngleDeg();
      this.maxAngleDeg = constraints.maxLaunchAngleDeg();
      this.fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

      this.speedStep = speedStep;
      this.angleStepDeg = angleStepDeg;
      this.radialStep = radialStep;
      this.bearingStepDeg = bearingStepDeg;

      this.range = MIN_RANGE_METERS;
      this.bearingDeg = 0.0;
      this.done = false;
      this.publishCounter = 0;

      Profiler.ensureInit();
      if (Profiler.enabled()) {
        Profiler.gaugeSet(
            "DragShotPlanner.profiler_file_hint_hash", Profiler.outputPathOrEmpty().hashCode());
      }
    }

    public boolean done() {
      return done;
    }

    public int size() {
      return entries.size();
    }

    public ShotLibrary snapshot(boolean completeFlag) {
      AutoCloseable _p = Profiler.section("DragShotPlanner.ShotLibraryBuilder.snapshot");
      try {
        return new ShotLibrary(
            targetFieldPosition,
            targetHeightMeters,
            shooterReleaseHeightMeters,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            constraints,
            List.copyOf(entries),
            completeFlag);
      } finally {
        closeQuietly(_p);
      }
    }

    public ShotLibrary maybeStep(long budgetNanos) {
      AutoCloseable _p = Profiler.section("DragShotPlanner.ShotLibraryBuilder.maybeStep");
      try {
        if (done) {
          return null;
        }
        long start = System.nanoTime();
        ShotLibrary publish = null;

        while (!done && (System.nanoTime() - start) < budgetNanos) {
          Translation2d shooterPos =
              targetFieldPosition.minus(
                  new Translation2d(range, Rotation2d.fromDegrees(bearingDeg)));

          boolean ok;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.library");
          try {
            ok =
                isShooterPoseValid(
                    shooterPos,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    null);
          } finally {
            closeQuietly(_p1);
          }

          if (ok) {
            ShotSolution bestAtPos;
            AutoCloseable _p2 =
                Profiler.section("DragShotPlanner.solveBestAtShooterPosition.library");
            try {
              bestAtPos =
                  solveBestAtShooterPosition(
                      gamePiece,
                      shooterPos,
                      targetFieldPosition,
                      targetHeightMeters,
                      shooterReleaseHeightMeters,
                      minSpeed,
                      maxSpeed,
                      minAngleDeg,
                      maxAngleDeg,
                      fixedAngle,
                      ACCEPTABLE_VERTICAL_ERROR_METERS,
                      constraints.shotStyle(),
                      speedStep,
                      angleStepDeg);
            } finally {
              closeQuietly(_p2);
            }

            if (bestAtPos != null) {
              entries.add(
                  new ShotLibraryEntry(
                      bestAtPos.shooterPosition(),
                      bestAtPos.shooterYaw().getRadians(),
                      bestAtPos.launchSpeedMetersPerSecond(),
                      bestAtPos.launchAngle().getRadians(),
                      bestAtPos.timeToPlaneSeconds(),
                      bestAtPos.verticalErrorMeters()));
              Profiler.counterAdd("DragShotPlanner.library.entries_added", 1);
              publishCounter++;
              if (publishCounter >= 8) {
                publishCounter = 0;
                publish = snapshot(false);
              }
            } else {
              Profiler.counterAdd("DragShotPlanner.library.no_solution", 1);
            }
          } else {
            Profiler.counterAdd("DragShotPlanner.library.pose_invalid", 1);
          }

          bearingDeg += bearingStepDeg;
          if (bearingDeg >= 360.0 - 1e-9) {
            bearingDeg = 0.0;
            range += radialStep;
            if (range > MAX_RANGE_METERS + 1e-6) {
              done = true;
              publish = snapshot(true);
              break;
            }
          }
        }

        Profiler.gaugeSet("DragShotPlanner.library.entries_size", entries.size());
        return publish;
      } finally {
        closeQuietly(_p);
      }
    }
  }

  public static final class OnlineSearchState {
    private Translation2d seed;
    private double stepMeters;
    private long lastUpdateNs;

    public OnlineSearchState(Translation2d seed, double stepMeters) {
      this.seed = seed;
      this.stepMeters = stepMeters;
      this.lastUpdateNs = System.nanoTime();
      Profiler.ensureInit();
    }

    public Translation2d seed() {
      return seed;
    }

    public void seed(Translation2d seed) {
      this.seed = seed;
    }

    public double stepMeters() {
      return stepMeters;
    }

    public void stepMeters(double stepMeters) {
      this.stepMeters = stepMeters;
    }

    public long lastUpdateNs() {
      return lastUpdateNs;
    }

    public void touch() {
      this.lastUpdateNs = System.nanoTime();
    }
  }

  private static final class Candidate {
    final Translation2d shooterPosition;
    final double shooterYawRad;
    final double speed;
    final double angleRad;
    final double timeToPlane;
    final double verticalError;
    final double robotDistanceSq;

    Candidate(
        Translation2d shooterPosition,
        double shooterYawRad,
        double speed,
        double angleRad,
        double timeToPlane,
        double verticalError,
        double robotDistanceSq) {
      this.shooterPosition = shooterPosition;
      this.shooterYawRad = shooterYawRad;
      this.speed = speed;
      this.angleRad = angleRad;
      this.timeToPlane = timeToPlane;
      this.verticalError = verticalError;
      this.robotDistanceSq = robotDistanceSq;
    }
  }

  public static final class GamePiecePhysicsConfig {
    public String name;
    public double mass_kg;
    public double drag_coefficient;
    public double cross_section_area_m2;
    public Double air_density_kg_per_m3;
    public Metadata metadata;

    public static final class Metadata {
      public String source_mesh;
      public String mesh_units;
      public double scale;
      public double volume_m3;
      public List<Double> bounds_extents_m;
      public String flight_axis;
      public double density_kg_per_m3;
      public Double mass_kg_override;
      public String area_method;
      public String auto_shape_hint;
      public double auto_drag_coefficient_hint;
      public String material;
      public double estimated_terminal_velocity_mps;
      public AltCrossSectionEstimates alt_cross_section_estimates_m2;
      public List<String> warnings;
    }

    public static final class AltCrossSectionEstimates {
      public double projected;
      public double bbox_ellipse;
      public double bbox_rect;
    }
  }

  private static final class YamlGamePiecePhysics extends GamePiecePhysics {
    private final String name;
    private final double massKg;
    private final double crossSectionAreaM2;
    private final double dragCoefficient;
    private final double airDensityKgPerM3;

    YamlGamePiecePhysics(
        String name,
        double massKg,
        double crossSectionAreaM2,
        double dragCoefficient,
        double airDensityKgPerM3) {
      this.name = name;
      this.massKg = massKg;
      this.crossSectionAreaM2 = crossSectionAreaM2;
      this.dragCoefficient = dragCoefficient;
      this.airDensityKgPerM3 = airDensityKgPerM3;
    }

    @SuppressWarnings("unused")
    public String name() {
      return name;
    }

    @Override
    public double massKg() {
      return massKg;
    }

    @Override
    public double crossSectionAreaM2() {
      return crossSectionAreaM2;
    }

    @Override
    public double dragCoefficient() {
      return dragCoefficient;
    }

    @Override
    public double airDensityKgPerM3() {
      return airDensityKgPerM3;
    }
  }

  private static final class SimParams {
    final double kOverM;

    SimParams(double kOverM) {
      this.kOverM = kOverM;
    }
  }

  private static final class SimOut {
    boolean hitPlane;
    double timeAtPlaneSeconds;
    double verticalErrorMeters;

    void set(boolean hitPlane, double timeAtPlaneSeconds, double verticalErrorMeters) {
      this.hitPlane = hitPlane;
      this.timeAtPlaneSeconds = timeAtPlaneSeconds;
      this.verticalErrorMeters = verticalErrorMeters;
    }
  }

  private static final ThreadLocal<SimOut> SIM_OUT_TL =
      ThreadLocal.withInitial(
          () -> {
            SimOut o = new SimOut();
            o.set(false, 0.0, Double.POSITIVE_INFINITY);
            return o;
          });

  private static final double EPS = 1e-6;
  private static final double MIN_RANGE_METERS = 0.5;
  private static final double MAX_RANGE_METERS = 7.0;
  private static final double MAX_ROBOT_TRAVEL_METERS = 7.0;
  private static final double MAX_ROBOT_TRAVEL_METERS_SQ =
      MAX_ROBOT_TRAVEL_METERS * MAX_ROBOT_TRAVEL_METERS;
  private static final double ACCEPTABLE_VERTICAL_ERROR_METERS = 0.06;

  private static final ConcurrentHashMap<String, GamePiecePhysics> GAME_PIECE_CACHE =
      new ConcurrentHashMap<>();

  private static final ConcurrentHashMap<GamePiecePhysics, SimParams> SIM_PARAMS_CACHE =
      new ConcurrentHashMap<>();

  private static final int SOLVE_Q_MM = 20;
  private static final int SOLVE_Q_DEG_TENTH = 5;

  private static final int SOLVE_CACHE_BITS = 14;
  private static final int SOLVE_CACHE_SIZE = 1 << SOLVE_CACHE_BITS;
  private static final long[] SOLVE_CACHE_KEYS = new long[SOLVE_CACHE_SIZE];
  private static final ShotSolution[] SOLVE_CACHE_VALS = new ShotSolution[SOLVE_CACHE_SIZE];
  private static final int SOLVE_CACHE_LOCK_BITS = 6;
  private static final int SOLVE_CACHE_LOCKS = 1 << SOLVE_CACHE_LOCK_BITS;
  private static final Object[] SOLVE_CACHE_GUARDS = new Object[SOLVE_CACHE_LOCKS];

  static {
    for (int i = 0; i < SOLVE_CACHE_GUARDS.length; i++) SOLVE_CACHE_GUARDS[i] = new Object();
  }

  private static long mix64(long z) {
    z ^= (z >>> 33);
    z *= 0xff51afd7ed558ccdL;
    z ^= (z >>> 33);
    z *= 0xc4ceb9fe1a85ec53L;
    z ^= (z >>> 33);
    return z;
  }

  private static ShotSolution solveCacheGet(long key) {
    int idx = (int) mix64(key) & (SOLVE_CACHE_SIZE - 1);
    if (SOLVE_CACHE_KEYS[idx] == key) {
      return SOLVE_CACHE_VALS[idx];
    }
    return null;
  }

  private static void solveCachePut(long key, ShotSolution val) {
    int idx = (int) mix64(key) & (SOLVE_CACHE_SIZE - 1);
    Object g = SOLVE_CACHE_GUARDS[idx & (SOLVE_CACHE_LOCKS - 1)];
    synchronized (g) {
      SOLVE_CACHE_KEYS[idx] = key;
      SOLVE_CACHE_VALS[idx] = val;
    }
  }

  private static long solveKey(
      Translation2d shooterPos,
      Translation2d target,
      double targetH,
      double releaseH,
      double minSpeed,
      double maxSpeed,
      double minAngDeg,
      double maxAngDeg,
      Constraints.ShotStyle style) {

    int sx = (int) Math.round(shooterPos.getX() * 1000.0 / SOLVE_Q_MM);
    int sy = (int) Math.round(shooterPos.getY() * 1000.0 / SOLVE_Q_MM);
    int tx = (int) Math.round(target.getX() * 1000.0 / SOLVE_Q_MM);
    int ty = (int) Math.round(target.getY() * 1000.0 / SOLVE_Q_MM);

    int th = (int) Math.round(targetH * 1000.0 / 20.0);
    int rh = (int) Math.round(releaseH * 1000.0 / 20.0);

    int ms = (int) Math.round(minSpeed * 10.0);
    int xs = (int) Math.round(maxSpeed * 10.0);

    int a0 = (int) Math.round(minAngDeg * 10.0 / SOLVE_Q_DEG_TENTH);
    int a1 = (int) Math.round(maxAngDeg * 10.0 / SOLVE_Q_DEG_TENTH);

    int st = style == null ? 0 : style.ordinal();

    long k = 1469598103934665603L;
    k = (k ^ sx) * 1099511628211L;
    k = (k ^ sy) * 1099511628211L;
    k = (k ^ tx) * 1099511628211L;
    k = (k ^ ty) * 1099511628211L;
    k = (k ^ th) * 1099511628211L;
    k = (k ^ rh) * 1099511628211L;
    k = (k ^ ms) * 1099511628211L;
    k = (k ^ xs) * 1099511628211L;
    k = (k ^ a0) * 1099511628211L;
    k = (k ^ a1) * 1099511628211L;
    k = (k ^ st) * 1099511628211L;
    return k;
  }

  private static volatile List<FieldPlanner.Obstacle> STATIC_OBSTACLES;

  private static final double[][] ONLINE_OFFS =
      new double[][] {
        {0.0, 0.0},
        {1.0, 0.0},
        {-1.0, 0.0},
        {0.0, 1.0},
        {0.0, -1.0},
        {0.7071067811865476, 0.7071067811865476},
        {0.7071067811865476, -0.7071067811865476},
        {-0.7071067811865476, 0.7071067811865476},
        {-0.7071067811865476, -0.7071067811865476}
      };

  private DragShotPlanner() {}

  private static void closeQuietly(AutoCloseable c) {
    if (c == null) return;
    try {
      c.close();
    } catch (Exception ignored) {
    }
  }

  private static SimParams simParams(GamePiecePhysics gp) {
    return SIM_PARAMS_CACHE.computeIfAbsent(
        gp,
        k -> {
          double m = gp.massKg();
          double A = gp.crossSectionAreaM2();
          double Cd = gp.dragCoefficient();
          double rho = gp.airDensityKgPerM3();
          double kOverM = 0.5 * rho * Cd * A / m;
          return new SimParams(kOverM);
        });
  }

  private static List<FieldPlanner.Obstacle> staticObstacles() {
    AutoCloseable _p = Profiler.section("DragShotPlanner.staticObstacles");
    try {
      List<FieldPlanner.Obstacle> v = STATIC_OBSTACLES;
      if (v != null) {
        return v;
      }
      ArrayList<FieldPlanner.Obstacle> out = new ArrayList<>(64);
      try {
        out.addAll(Constants.FIELD.walls());
      } catch (Throwable ignored) {
      }
      try {
        out.addAll(Constants.FIELD.fieldObstacles());
      } catch (Throwable ignored) {
      }
      STATIC_OBSTACLES = List.copyOf(out);
      Profiler.gaugeSet("DragShotPlanner.staticObstacles.size", STATIC_OBSTACLES.size());
      return STATIC_OBSTACLES;
    } finally {
      closeQuietly(_p);
    }
  }

  public static GamePiecePhysics loadGamePieceFromDeployYaml(String id) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.loadGamePieceFromDeployYaml");
    try {
      if (id == null || id.isEmpty()) {
        throw new IllegalArgumentException("id must be non-empty");
      }
      return GAME_PIECE_CACHE.computeIfAbsent(
          id, DragShotPlanner::loadGamePieceFromDeployYamlInternal);
    } finally {
      closeQuietly(_p);
    }
  }

  private static GamePiecePhysics loadGamePieceFromDeployYamlInternal(String id) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.loadGamePieceFromDeployYamlInternal");
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      Path gamePiecesDir = deployDir.resolve("gamepieces");
      String fileName = id.endsWith(".yml") || id.endsWith(".yaml") ? id : id + ".yaml";
      Path path = gamePiecesDir.resolve(fileName);
      if (!Files.exists(path)) {
        DriverStation.reportError("Game piece YAML not found: " + path.toString(), false);
        throw new IllegalStateException("Missing game piece YAML: " + path.toString());
      }
      Yaml yaml = new Yaml(new Constructor(GamePiecePhysicsConfig.class, new LoaderOptions()));
      try (InputStream in = Files.newInputStream(path)) {
        GamePiecePhysicsConfig cfg = yaml.load(in);
        if (cfg == null) {
          DriverStation.reportError("Game piece YAML empty: " + path.toString(), false);
          throw new IllegalStateException("Game piece YAML empty: " + path.toString());
        }
        double mass =
            (cfg.metadata != null
                    && cfg.metadata.mass_kg_override != null
                    && cfg.metadata.mass_kg_override > 0.0)
                ? cfg.metadata.mass_kg_override
                : cfg.mass_kg;
        double area = cfg.cross_section_area_m2;
        double cd = cfg.drag_coefficient;
        double air =
            cfg.air_density_kg_per_m3 != null && cfg.air_density_kg_per_m3 > 0.0
                ? cfg.air_density_kg_per_m3
                : 1.225;

        if (mass <= 0.0 || area <= 0.0 || cd <= 0.0 || air <= 0.0) {
          DriverStation.reportError("Invalid game piece YAML values: " + path.toString(), false);
          throw new IllegalStateException("Invalid game piece YAML: " + path.toString());
        }
        String name = cfg.name != null && !cfg.name.isEmpty() ? cfg.name : id;
        return new YamlGamePiecePhysics(name, mass, area, cd, air);
      } catch (IOException ex) {
        DriverStation.reportError("Failed to read game piece YAML: " + path.toString(), false);
        throw new IllegalStateException("Failed to read game piece YAML: " + path.toString(), ex);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Failed to parse game piece YAML: " + path.toString(), false);
        throw ex;
      }
    } finally {
      closeQuietly(_p);
    }
  }

  public static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.isShooterPoseValid");
    try {
      return isShooterPoseValidInternal(
          shooterPos,
          targetFieldPosition,
          robotHalfLengthMeters,
          robotHalfWidthMeters,
          dynamicObstacles);
    } finally {
      closeQuietly(_p);
    }
  }

  public static Optional<ShotSolution> findBestShotFromLibrary(
      ShotLibrary library,
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Constraints constraints) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotFromLibrary");
    try {
      Profiler.ensureInit();

      if (library == null || library.entries().isEmpty()) {
        Profiler.counterAdd("DragShotPlanner.library.empty", 1);
        return Optional.empty();
      }

      Profiler.gaugeSet("DragShotPlanner.library.entries", library.entries().size());

      Translation2d robotCurrentPosition = robotPose.getTranslation();
      Candidate best = null;

      boolean needDynamicCheck = dynamicObstacles != null && !dynamicObstacles.isEmpty();
      if (needDynamicCheck) {
        Profiler.gaugeSet("DragShotPlanner.dynamicObstacles.count", dynamicObstacles.size());
      }

      double rx = robotCurrentPosition.getX();
      double ry = robotCurrentPosition.getY();

      int iter = 0;
      int rejectedTravel = 0;
      int rejectedDyn = 0;
      int accepted = 0;

      for (ShotLibraryEntry e : library.entries()) {
        iter++;
        Translation2d shooterPos = e.shooterPosition();

        double dx = rx - shooterPos.getX();
        double dy = ry - shooterPos.getY();
        double robotDistanceSq = dx * dx + dy * dy;
        if (robotDistanceSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
          rejectedTravel++;
          continue;
        }

        if (needDynamicCheck) {
          boolean okDyn;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.dynamic");
          try {
            okDyn =
                isShooterPoseValidInternal(
                    shooterPos,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles);
          } finally {
            closeQuietly(_p1);
          }
          if (!okDyn) {
            rejectedDyn++;
            continue;
          }
        }

        Candidate next =
            new Candidate(
                shooterPos,
                e.shooterYawRad(),
                e.launchSpeedMetersPerSecond(),
                e.launchAngleRad(),
                e.timeToPlaneSeconds(),
                Math.abs(e.verticalErrorMeters()),
                robotDistanceSq);

        if (isBetterCandidate(best, next, constraints.shotStyle())) {
          best = next;
        }
        accepted++;
      }

      Profiler.counterAdd("DragShotPlanner.library.loop_iter", iter);
      Profiler.counterAdd("DragShotPlanner.library.rejected_travel", rejectedTravel);
      Profiler.counterAdd("DragShotPlanner.library.rejected_dynamic", rejectedDyn);
      Profiler.counterAdd("DragShotPlanner.library.accepted", accepted);

      if (best == null) {
        Profiler.counterAdd("DragShotPlanner.library.no_best", 1);
        return Optional.empty();
      }

      ShotSolution refined;
      AutoCloseable _p2 = Profiler.section("DragShotPlanner.refineShotAtPosition.library");
      try {
        refined =
            refineShotAtPosition(
                gamePiece,
                best.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                constraints.minLaunchSpeedMetersPerSecond(),
                constraints.maxLaunchSpeedMetersPerSecond(),
                constraints.minLaunchAngleDeg(),
                constraints.maxLaunchAngleDeg(),
                Math.abs(constraints.maxLaunchAngleDeg() - constraints.minLaunchAngleDeg()) < 1e-6,
                ACCEPTABLE_VERTICAL_ERROR_METERS,
                best.speed,
                best.angleRad);
      } finally {
        closeQuietly(_p2);
      }

      if (refined != null) {
        return Optional.of(refined);
      }

      return Optional.of(
          new ShotSolution(
              best.shooterPosition,
              Rotation2d.fromRadians(best.shooterYawRad),
              best.speed,
              Rotation2d.fromRadians(best.angleRad),
              best.timeToPlane,
              targetFieldPosition,
              best.verticalError));
    } finally {
      closeQuietly(_p);
    }
  }

  public static Optional<ShotSolution> findBestShotAuto(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Constraints constraints) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotAuto");
    try {
      Profiler.ensureInit();

      double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      double minAngleDeg = constraints.minLaunchAngleDeg();
      double maxAngleDeg = constraints.maxLaunchAngleDeg();

      if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
        Profiler.counterAdd("DragShotPlanner.auto.bad_speed_range", 1);
        return Optional.empty();
      }
      if (!(Math.abs(maxAngleDeg - minAngleDeg) < 1e-6) && maxAngleDeg <= minAngleDeg) {
        Profiler.counterAdd("DragShotPlanner.auto.bad_angle_range", 1);
        return Optional.empty();
      }

      Translation2d robotCurrentPosition = robotPose.getTranslation();
      boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

      Candidate coarse;
      AutoCloseable _p1 = Profiler.section("DragShotPlanner.coarseSearch");
      try {
        coarse =
            coarseSearch(
                gamePiece,
                targetFieldPosition,
                targetHeightMeters,
                robotCurrentPosition,
                shooterReleaseHeightMeters,
                robotHalfLengthMeters,
                robotHalfWidthMeters,
                dynamicObstacles,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                constraints.shotStyle());
      } finally {
        closeQuietly(_p1);
      }

      if (coarse == null) {
        Profiler.counterAdd("DragShotPlanner.auto.coarse_none", 1);
        return Optional.empty();
      }

      ShotSolution refined;
      AutoCloseable _p2 = Profiler.section("DragShotPlanner.refineShotAtPosition.auto");
      try {
        refined =
            refineShotAtPosition(
                gamePiece,
                coarse.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                ACCEPTABLE_VERTICAL_ERROR_METERS,
                coarse.speed,
                coarse.angleRad);
      } finally {
        closeQuietly(_p2);
      }

      if (refined == null) {
        Profiler.counterAdd("DragShotPlanner.auto.refine_none", 1);
        return Optional.empty();
      }

      return Optional.of(refined);
    } finally {
      closeQuietly(_p);
    }
  }

  public static Optional<ShotSolution> findBestShotOnlineRefine(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Constraints constraints,
      OnlineSearchState state,
      long budgetNanos) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotOnlineRefine");
    try {
      Profiler.ensureInit();

      if (gamePiece == null
          || targetFieldPosition == null
          || robotPose == null
          || constraints == null
          || state == null) {
        Profiler.counterAdd("DragShotPlanner.online.bad_inputs", 1);
        return Optional.empty();
      }

      double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      double minAngleDeg = constraints.minLaunchAngleDeg();
      double maxAngleDeg = constraints.maxLaunchAngleDeg();
      boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

      if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
        Profiler.counterAdd("DragShotPlanner.online.bad_speed_range", 1);
        return Optional.empty();
      }
      if (!fixedAngle && maxAngleDeg <= minAngleDeg) {
        Profiler.counterAdd("DragShotPlanner.online.bad_angle_range", 1);
        return Optional.empty();
      }

      Translation2d robotPos = robotPose.getTranslation();
      double robotX = robotPos.getX();
      double robotY = robotPos.getY();

      Translation2d seed = state.seed();
      if (seed == null) {
        seed = projectToValidRing(robotPos, targetFieldPosition);
        state.seed(seed);
      }

      double speedStep = Math.max(0.22, (maxSpeed - minSpeed) / 70.0);
      double angleStep = fixedAngle ? 1.0 : Math.max(0.40, (maxAngleDeg - minAngleDeg) / 70.0);

      Candidate best = null;

      ShotSolution seedSol;
      AutoCloseable _p1 = Profiler.section("DragShotPlanner.solveBestAtShooterPosition.seed");
      try {
        seedSol =
            solveBestAtShooterPosition(
                gamePiece,
                seed,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                ACCEPTABLE_VERTICAL_ERROR_METERS,
                constraints.shotStyle(),
                speedStep,
                angleStep);
      } finally {
        closeQuietly(_p1);
      }

      if (seedSol != null) {
        boolean ok;
        AutoCloseable _p2 = Profiler.section("DragShotPlanner.isShooterPoseValid.seed");
        try {
          ok =
              isShooterPoseValidInternal(
                  seed,
                  targetFieldPosition,
                  robotHalfLengthMeters,
                  robotHalfWidthMeters,
                  dynamicObstacles);
        } finally {
          closeQuietly(_p2);
        }
        if (ok) {
          double dx = robotX - seedSol.shooterPosition().getX();
          double dy = robotY - seedSol.shooterPosition().getY();
          double distSq = dx * dx + dy * dy;
          if (distSq <= MAX_ROBOT_TRAVEL_METERS_SQ + 1e-6) {
            best =
                new Candidate(
                    seedSol.shooterPosition(),
                    seedSol.shooterYaw().getRadians(),
                    seedSol.launchSpeedMetersPerSecond(),
                    seedSol.launchAngle().getRadians(),
                    seedSol.timeToPlaneSeconds(),
                    Math.abs(seedSol.verticalErrorMeters()),
                    distSq);
          }
        }
      }

      long start = System.nanoTime();
      double step = Math.max(0.11, Math.min(0.95, state.stepMeters()));
      Translation2d current = seed;

      int outer = 0;
      int inner = 0;
      int rejectedTravel = 0;
      int rejectedPose = 0;
      int rejectedNoSol = 0;
      int improvedCount = 0;

      while ((System.nanoTime() - start) < budgetNanos) {
        outer++;
        boolean improved = false;

        double cx = current.getX();
        double cy = current.getY();

        for (double[] o : ONLINE_OFFS) {
          inner++;
          Translation2d p = new Translation2d(cx + o[0] * step, cy + o[1] * step);
          Translation2d clipped = clipToRingAndField(p, targetFieldPosition);

          double dxr = robotX - clipped.getX();
          double dyr = robotY - clipped.getY();
          double distSq = dxr * dxr + dyr * dyr;
          if (distSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
            rejectedTravel++;
            continue;
          }

          boolean ok;
          AutoCloseable _p3 = Profiler.section("DragShotPlanner.isShooterPoseValid.online");
          try {
            ok =
                isShooterPoseValidInternal(
                    clipped,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles);
          } finally {
            closeQuietly(_p3);
          }
          if (!ok) {
            rejectedPose++;
            continue;
          }

          ShotSolution sol;
          AutoCloseable _p4 = Profiler.section("DragShotPlanner.solveBestAtShooterPosition.online");
          try {
            sol =
                solveBestAtShooterPosition(
                    gamePiece,
                    clipped,
                    targetFieldPosition,
                    targetHeightMeters,
                    shooterReleaseHeightMeters,
                    minSpeed,
                    maxSpeed,
                    minAngleDeg,
                    maxAngleDeg,
                    fixedAngle,
                    ACCEPTABLE_VERTICAL_ERROR_METERS,
                    constraints.shotStyle(),
                    speedStep,
                    angleStep);
          } finally {
            closeQuietly(_p4);
          }

          if (sol == null) {
            rejectedNoSol++;
            continue;
          }

          double dx2 = robotX - sol.shooterPosition().getX();
          double dy2 = robotY - sol.shooterPosition().getY();
          double distSq2 = dx2 * dx2 + dy2 * dy2;

          Candidate cand =
              new Candidate(
                  sol.shooterPosition(),
                  sol.shooterYaw().getRadians(),
                  sol.launchSpeedMetersPerSecond(),
                  sol.launchAngle().getRadians(),
                  sol.timeToPlaneSeconds(),
                  Math.abs(sol.verticalErrorMeters()),
                  distSq2);

          if (isBetterCandidate(best, cand, constraints.shotStyle())) {
            best = cand;
            current = sol.shooterPosition();
            improved = true;
            improvedCount++;
          }
        }

        if (!improved) {
          step *= 0.58;
          if (step < 0.075) {
            break;
          }
        }
      }

      Profiler.counterAdd("DragShotPlanner.online.outer", outer);
      Profiler.counterAdd("DragShotPlanner.online.inner", inner);
      Profiler.counterAdd("DragShotPlanner.online.rejected_travel", rejectedTravel);
      Profiler.counterAdd("DragShotPlanner.online.rejected_pose", rejectedPose);
      Profiler.counterAdd("DragShotPlanner.online.rejected_no_solution", rejectedNoSol);
      Profiler.counterAdd("DragShotPlanner.online.improved", improvedCount);
      Profiler.gaugeSet("DragShotPlanner.online.final_step_mm", (long) (step * 1000.0));

      if (best == null) {
        Profiler.counterAdd("DragShotPlanner.online.no_best", 1);
        return Optional.empty();
      }

      state.seed(best.shooterPosition);
      state.stepMeters(Math.max(0.10, Math.min(0.80, step)));
      state.touch();

      ShotSolution refined;
      AutoCloseable _p5 = Profiler.section("DragShotPlanner.refineShotAtPosition.online");
      try {
        refined =
            refineShotAtPosition(
                gamePiece,
                best.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                ACCEPTABLE_VERTICAL_ERROR_METERS,
                best.speed,
                best.angleRad);
      } finally {
        closeQuietly(_p5);
      }

      if (refined != null) {
        return Optional.of(refined);
      }

      return Optional.of(
          new ShotSolution(
              best.shooterPosition,
              Rotation2d.fromRadians(best.shooterYawRad),
              best.speed,
              Rotation2d.fromRadians(best.angleRad),
              best.timeToPlane,
              targetFieldPosition,
              best.verticalError));
    } finally {
      closeQuietly(_p);
    }
  }

  private static Translation2d projectToValidRing(Translation2d point, Translation2d target) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.projectToValidRing");
    try {
      Translation2d delta = point.minus(target);
      double r = delta.getNorm();
      if (r < 1e-6) {
        return target.minus(new Translation2d(3.0, Rotation2d.kZero));
      }
      double clamped = Math.max(MIN_RANGE_METERS, Math.min(MAX_RANGE_METERS, r));
      Translation2d unit = delta.div(r);
      return target.plus(unit.times(clamped));
    } finally {
      closeQuietly(_p);
    }
  }

  private static Translation2d clipToRingAndField(Translation2d p, Translation2d target) {
    AutoCloseable _p0 = Profiler.section("DragShotPlanner.clipToRingAndField");
    try {
      Translation2d ring = projectToValidRing(p, target);
      double x = ring.getX();
      double y = ring.getY();
      if (!Double.isFinite(x) || !Double.isFinite(y)) {
        return target.minus(new Translation2d(3.0, Rotation2d.kZero));
      }
      return new Translation2d(x, y);
    } finally {
      closeQuietly(_p0);
    }
  }

  private static boolean isBetterCandidate(
      Candidate best, Candidate next, Constraints.ShotStyle style) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.isBetterCandidate");
    try {
      if (best == null) {
        return true;
      }

      if (next.robotDistanceSq < best.robotDistanceSq - 1e-9) {
        return true;
      }
      if (best.robotDistanceSq < next.robotDistanceSq - 1e-9) {
        return false;
      }

      if (style == Constraints.ShotStyle.DIRECT || style == Constraints.ShotStyle.ARC) {
        double angleBest = Math.abs(best.angleRad);
        double angleNext = Math.abs(next.angleRad);
        double angleEps = 1e-3;
        if (style == Constraints.ShotStyle.DIRECT) {
          if (angleNext < angleBest - angleEps) {
            return true;
          }
          if (angleBest < angleNext - angleEps) {
            return false;
          }
        } else {
          if (angleNext > angleBest + angleEps) {
            return true;
          }
          if (angleBest > angleNext + angleEps) {
            return false;
          }
        }
      }

      if (next.speed < best.speed - EPS) {
        return true;
      }
      if (best.speed < next.speed - EPS) {
        return false;
      }

      return next.verticalError < best.verticalError - EPS;
    } finally {
      closeQuietly(_p);
    }
  }

  private static Candidate coarseSearch(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Translation2d robotCurrentPosition,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      double minSpeed,
      double maxSpeed,
      double minAngleDeg,
      double maxAngleDeg,
      boolean fixedAngle,
      Constraints.ShotStyle shotStyle) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.coarseSearch.body");
    try {
      double speedRange = maxSpeed - minSpeed;
      double angleRange = maxAngleDeg - minAngleDeg;

      double speedStepCoarse = Math.max(0.6, speedRange / 9.0);
      double angleStepCoarse = fixedAngle ? 1.0 : Math.max(2.4, angleRange / 9.0);
      double radialStepCoarse = 0.55;
      double bearingStepDegCoarse = 22.0;
      double coarseTolerance = ACCEPTABLE_VERTICAL_ERROR_METERS * 3.0;

      Candidate bestCoarse = null;

      double rx = robotCurrentPosition.getX();
      double ry = robotCurrentPosition.getY();

      int ranges = 0;
      int bearings = 0;
      int posesChecked = 0;
      int posesRejected = 0;
      int sims = 0;
      int simsHit = 0;
      int simsAccepted = 0;

      SimOut sim = SIM_OUT_TL.get();

      for (double range = MIN_RANGE_METERS;
          range <= MAX_RANGE_METERS + 1e-6;
          range += radialStepCoarse) {
        ranges++;
        for (double bearingDeg = 0.0; bearingDeg < 360.0; bearingDeg += bearingStepDegCoarse) {
          bearings++;
          Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
          Translation2d shooterPos = targetFieldPosition.minus(new Translation2d(range, bearing));

          double dx = rx - shooterPos.getX();
          double dy = ry - shooterPos.getY();
          double robotDistanceSq = dx * dx + dy * dy;
          if (robotDistanceSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
            continue;
          }

          posesChecked++;
          boolean ok;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.coarse");
          try {
            ok =
                isShooterPoseValidInternal(
                    shooterPos,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles);
          } finally {
            closeQuietly(_p1);
          }
          if (!ok) {
            posesRejected++;
            continue;
          }

          double horizontalDistance = shooterPos.getDistance(targetFieldPosition);
          if (horizontalDistance < 1e-3) {
            continue;
          }

          Rotation2d shooterYaw = targetFieldPosition.minus(shooterPos).getAngle();

          double angleStartDeg;
          double angleEndDeg;
          double angleStepDeg;

          if (fixedAngle) {
            angleStartDeg = minAngleDeg;
            angleEndDeg = maxAngleDeg;
            angleStepDeg = 1.0;
          } else {
            angleStartDeg = minAngleDeg;
            angleEndDeg = maxAngleDeg;
            angleStepDeg = angleStepCoarse;
          }

          for (double angleDeg = angleStartDeg;
              angleDeg <= angleEndDeg + 1e-6;
              angleDeg += angleStepDeg) {

            double angleRad = Math.toRadians(angleDeg);
            double cos = Math.cos(angleRad);
            if (cos <= 0.0) {
              continue;
            }
            double sin = Math.sin(angleRad);

            for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += speedStepCoarse) {
              sims++;
              AutoCloseable _p2 = Profiler.section("DragShotPlanner.simulateToTargetPlane.coarse");
              try {
                simulateToTargetPlaneInto(
                    sim,
                    gamePiece,
                    speed * cos,
                    speed * sin,
                    shooterReleaseHeightMeters,
                    horizontalDistance,
                    targetHeightMeters);
              } finally {
                closeQuietly(_p2);
              }

              if (!sim.hitPlane) {
                continue;
              }
              simsHit++;

              double error = Math.abs(sim.verticalErrorMeters);
              if (error > coarseTolerance) {
                continue;
              }
              simsAccepted++;

              Candidate next =
                  new Candidate(
                      shooterPos,
                      shooterYaw.getRadians(),
                      speed,
                      angleRad,
                      sim.timeAtPlaneSeconds,
                      error,
                      robotDistanceSq);

              if (isBetterCandidate(bestCoarse, next, shotStyle)) {
                bestCoarse = next;
              }
            }
          }
        }
      }

      Profiler.counterAdd("DragShotPlanner.coarse.ranges", ranges);
      Profiler.counterAdd("DragShotPlanner.coarse.bearings", bearings);
      Profiler.counterAdd("DragShotPlanner.coarse.poses_checked", posesChecked);
      Profiler.counterAdd("DragShotPlanner.coarse.poses_rejected", posesRejected);
      Profiler.counterAdd("DragShotPlanner.coarse.sims", sims);
      Profiler.counterAdd("DragShotPlanner.coarse.sims_hitplane", simsHit);
      Profiler.counterAdd("DragShotPlanner.coarse.sims_accepted", simsAccepted);

      return bestCoarse;
    } finally {
      closeQuietly(_p);
    }
  }

  private static ShotSolution solveBestAtShooterPosition(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      double minSpeed,
      double maxSpeed,
      double minAngleDeg,
      double maxAngleDeg,
      boolean fixedAngle,
      double acceptableVerticalErrorMeters,
      Constraints.ShotStyle shotStyle,
      double speedStep,
      double angleStepDeg) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.solveBestAtShooterPosition.body");
    try {
      long ck =
          solveKey(
              shooterFieldPosition,
              targetFieldPosition,
              targetHeightMeters,
              shooterReleaseHeightMeters,
              minSpeed,
              maxSpeed,
              minAngleDeg,
              maxAngleDeg,
              shotStyle);

      ShotSolution cached = solveCacheGet(ck);
      if (cached != null) {
        return cached;
      }

      double sx = shooterFieldPosition.getX();
      double sy = shooterFieldPosition.getY();
      double tx = targetFieldPosition.getX();
      double ty = targetFieldPosition.getY();

      double dxT = tx - sx;
      double dyT = ty - sy;
      double horizontalDistance = Math.sqrt(dxT * dxT + dyT * dyT);
      if (horizontalDistance < 1e-3) {
        return null;
      }

      double invD = 1.0 / horizontalDistance;
      double dirUx = dxT * invD;
      double dirUy = dyT * invD;

      Rotation2d shooterYaw = Rotation2d.fromRadians(Math.atan2(dyT, dxT));

      double angleStep = fixedAngle ? 1.0 : Math.max(0.18, angleStepDeg);

      double bestErrorAbs = Double.POSITIVE_INFINITY;
      double bestAngleRad = 0.0;
      double bestSpeed = 0.0;
      double bestTime = 0.0;
      double bestSignedErr = 0.0;
      boolean found = false;

      int sims = 0;
      int simsHit = 0;
      int simsWithin = 0;

      SimOut sim = SIM_OUT_TL.get();

      double speedRange = maxSpeed - minSpeed;
      double coarseStep = Math.max(Math.max(0.9, speedStep * 3.0), speedRange / 10.0);

      for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg + 1e-6; angleDeg += angleStep) {
        double angleRad = Math.toRadians(angleDeg);
        double cos = Math.cos(angleRad);
        if (cos <= 0.0) continue;
        double sin = Math.sin(angleRad);

        double localBestSpeed = 0.0;
        double localBestErrAbs = Double.POSITIVE_INFINITY;
        double localBestTime = 0.0;
        double localBestSigned = 0.0;
        boolean localFound = false;

        for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += coarseStep) {
          sims++;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.simulateToTargetPlane.solveAtPos");
          try {
            simulateToTargetPlaneInto(
                sim,
                gamePiece,
                speed * cos,
                speed * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            closeQuietly(_p1);
          }

          if (!sim.hitPlane) continue;
          simsHit++;

          double errAbs = Math.abs(sim.verticalErrorMeters);
          if (errAbs <= acceptableVerticalErrorMeters) simsWithin++;

          if (!localFound || errAbs < localBestErrAbs - 1e-9) {
            localFound = true;
            localBestSpeed = speed;
            localBestErrAbs = errAbs;
            localBestTime = sim.timeAtPlaneSeconds;
            localBestSigned = sim.verticalErrorMeters;
            if (localBestErrAbs <= acceptableVerticalErrorMeters * 0.35) break;
          }
        }

        if (!localFound) continue;

        double lo = localBestSpeed - coarseStep;
        double hi = localBestSpeed + coarseStep;
        if (lo < minSpeed) lo = minSpeed;
        if (hi > maxSpeed) hi = maxSpeed;

        double refineBestSpeed = localBestSpeed;
        double refineBestErrAbs = localBestErrAbs;
        double refineBestTime = localBestTime;
        double refineBestSigned = localBestSigned;

        for (int it = 0; it < 9; it++) {
          double m1 = lo + (hi - lo) * (1.0 / 3.0);
          double m2 = hi - (hi - lo) * (1.0 / 3.0);

          double e1Abs;
          double t1;
          double s1;
          sims++;
          AutoCloseable _p2 = Profiler.section("DragShotPlanner.simulateToTargetPlane.solveAtPos");
          try {
            simulateToTargetPlaneInto(
                sim,
                gamePiece,
                m1 * cos,
                m1 * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            closeQuietly(_p2);
          }
          if (sim.hitPlane) {
            simsHit++;
            e1Abs = Math.abs(sim.verticalErrorMeters);
            t1 = sim.timeAtPlaneSeconds;
            s1 = sim.verticalErrorMeters;
            if (e1Abs <= acceptableVerticalErrorMeters) simsWithin++;
          } else {
            e1Abs = Double.POSITIVE_INFINITY;
            t1 = 0.0;
            s1 = Double.POSITIVE_INFINITY;
          }

          double e2Abs;
          double t2;
          double s2;
          sims++;
          AutoCloseable _p3 = Profiler.section("DragShotPlanner.simulateToTargetPlane.solveAtPos");
          try {
            simulateToTargetPlaneInto(
                sim,
                gamePiece,
                m2 * cos,
                m2 * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            closeQuietly(_p3);
          }
          if (sim.hitPlane) {
            simsHit++;
            e2Abs = Math.abs(sim.verticalErrorMeters);
            t2 = sim.timeAtPlaneSeconds;
            s2 = sim.verticalErrorMeters;
            if (e2Abs <= acceptableVerticalErrorMeters) simsWithin++;
          } else {
            e2Abs = Double.POSITIVE_INFINITY;
            t2 = 0.0;
            s2 = Double.POSITIVE_INFINITY;
          }

          if (e1Abs < refineBestErrAbs) {
            refineBestErrAbs = e1Abs;
            refineBestSpeed = m1;
            refineBestTime = t1;
            refineBestSigned = s1;
          }
          if (e2Abs < refineBestErrAbs) {
            refineBestErrAbs = e2Abs;
            refineBestSpeed = m2;
            refineBestTime = t2;
            refineBestSigned = s2;
          }

          if (e1Abs <= e2Abs) {
            hi = m2;
          } else {
            lo = m1;
          }

          if (refineBestErrAbs <= acceptableVerticalErrorMeters * 0.18) break;
        }

        boolean take;
        if (!found) {
          take = true;
        } else {
          double errEps = 1e-6;
          if (refineBestErrAbs < bestErrorAbs - errEps) {
            take = true;
          } else if (Math.abs(refineBestErrAbs - bestErrorAbs) <= errEps) {
            if (shotStyle == Constraints.ShotStyle.DIRECT
                || shotStyle == Constraints.ShotStyle.ARC) {
              double aBest = Math.abs(bestAngleRad);
              double aNext = Math.abs(angleRad);
              double aEps = 1e-3;
              if (shotStyle == Constraints.ShotStyle.DIRECT) {
                if (aNext < aBest - aEps) take = true;
                else if (aBest < aNext - aEps) take = false;
                else take = refineBestSpeed < bestSpeed - EPS;
              } else {
                if (aNext > aBest + aEps) take = true;
                else if (aBest > aNext + aEps) take = false;
                else take = refineBestSpeed < bestSpeed - EPS;
              }
            } else {
              take = refineBestSpeed < bestSpeed - EPS;
            }
          } else {
            take = false;
          }
        }

        if (take) {
          found = true;
          bestErrorAbs = refineBestErrAbs;
          bestAngleRad = angleRad;
          bestSpeed = refineBestSpeed;
          bestTime = refineBestTime;
          bestSignedErr = refineBestSigned;
        }
      }

      Profiler.counterAdd("DragShotPlanner.solveAtPos.sims", sims);
      Profiler.counterAdd("DragShotPlanner.solveAtPos.sims_hitplane", simsHit);
      Profiler.counterAdd("DragShotPlanner.solveAtPos.sims_within", simsWithin);

      if (!found) {
        return null;
      }

      Translation2d impactPos =
          new Translation2d(sx + dirUx * horizontalDistance, sy + dirUy * horizontalDistance);

      ShotSolution outSolution =
          new ShotSolution(
              shooterFieldPosition,
              shooterYaw,
              bestSpeed,
              Rotation2d.fromRadians(bestAngleRad),
              bestTime,
              impactPos,
              bestSignedErr);

      solveCachePut(ck, outSolution);
      return outSolution;
    } finally {
      closeQuietly(_p);
    }
  }

  private static ShotSolution refineShotAtPosition(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      double minSpeed,
      double maxSpeed,
      double minAngleDeg,
      double maxAngleDeg,
      boolean fixedAngle,
      double acceptableVerticalErrorMeters,
      double coarseSpeed,
      double coarseAngleRad) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.refineShotAtPosition.body");
    try {
      double sx = shooterFieldPosition.getX();
      double sy = shooterFieldPosition.getY();
      double tx = targetFieldPosition.getX();
      double ty = targetFieldPosition.getY();

      double dxT = tx - sx;
      double dyT = ty - sy;
      double horizontalDistance = Math.sqrt(dxT * dxT + dyT * dyT);
      if (horizontalDistance < 1e-3) {
        return null;
      }

      double invD = 1.0 / horizontalDistance;
      double dirUx = dxT * invD;
      double dirUy = dyT * invD;

      Rotation2d shooterYaw = Rotation2d.fromRadians(Math.atan2(dyT, dxT));

      double speedWindow = Math.max(1.5, (maxSpeed - minSpeed) / 8.0);
      double speedMin = Math.max(minSpeed, coarseSpeed - speedWindow);
      double speedMax = Math.min(maxSpeed, coarseSpeed + speedWindow);
      double speedStepFine = Math.max(0.11, (speedMax - speedMin) / 18.0);

      double angleStartDeg;
      double angleEndDeg;
      double angleStepFineDeg;
      double coarseAngleDeg = Math.toDegrees(coarseAngleRad);

      if (fixedAngle) {
        angleStartDeg = minAngleDeg;
        angleEndDeg = maxAngleDeg;
        angleStepFineDeg = 1.0;
      } else {
        double angleWindow = Math.max(3.5, (maxAngleDeg - minAngleDeg) / 9.0);
        angleStartDeg = Math.max(minAngleDeg, coarseAngleDeg - angleWindow);
        angleEndDeg = Math.min(maxAngleDeg, coarseAngleDeg + angleWindow);
        angleStepFineDeg = Math.max(0.22, (angleEndDeg - angleStartDeg) / 22.0);
      }

      ShotSolution best = null;
      double bestError = Double.POSITIVE_INFINITY;

      int sims = 0;
      int simsHit = 0;
      int simsWithin = 0;

      SimOut sim = SIM_OUT_TL.get();

      for (double angleDeg = angleStartDeg;
          angleDeg <= angleEndDeg + 1e-6;
          angleDeg += angleStepFineDeg) {

        double angleRad = Math.toRadians(angleDeg);
        double cos = Math.cos(angleRad);
        if (cos <= 0.0) {
          continue;
        }
        double sin = Math.sin(angleRad);

        for (double speed = speedMin; speed <= speedMax + 1e-6; speed += speedStepFine) {
          sims++;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.simulateToTargetPlane.refine");
          try {
            simulateToTargetPlaneInto(
                sim,
                gamePiece,
                speed * cos,
                speed * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            closeQuietly(_p1);
          }

          if (!sim.hitPlane) {
            continue;
          }
          simsHit++;

          double error = Math.abs(sim.verticalErrorMeters);
          if (error > acceptableVerticalErrorMeters) {
            continue;
          }
          simsWithin++;

          if (best == null
              || error < bestError - EPS
              || (Math.abs(error - bestError) <= EPS
                  && speed < best.launchSpeedMetersPerSecond() - EPS)) {
            Translation2d impactPos =
                new Translation2d(sx + dirUx * horizontalDistance, sy + dirUy * horizontalDistance);
            bestError = error;
            best =
                new ShotSolution(
                    shooterFieldPosition,
                    shooterYaw,
                    speed,
                    Rotation2d.fromRadians(angleRad),
                    sim.timeAtPlaneSeconds,
                    impactPos,
                    sim.verticalErrorMeters);
          }
        }
      }

      Profiler.counterAdd("DragShotPlanner.refine.sims", sims);
      Profiler.counterAdd("DragShotPlanner.refine.sims_hitplane", simsHit);
      Profiler.counterAdd("DragShotPlanner.refine.sims_within", simsWithin);

      return best;
    } finally {
      closeQuietly(_p);
    }
  }

  private static void simulateToTargetPlaneInto(
      SimOut out,
      GamePiecePhysics gamePiece,
      double vx0,
      double vy0,
      double shooterReleaseHeightMeters,
      double targetHorizontalDistanceMeters,
      double targetHeightMeters) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.simulateToTargetPlane.body");
    try {
      final double g = 9.81;

      double avx0 = vx0 >= 0.0 ? vx0 : -vx0;
      if (avx0 < 1e-6) {
        out.set(false, 0.0, Double.POSITIVE_INFINITY);
        return;
      }

      double minHorizontalSpeed = avx0 < 0.6 ? 0.6 : avx0;
      double timeNoDrag = targetHorizontalDistanceMeters / minHorizontalSpeed;
      double maxTimeSeconds = timeNoDrag * 1.45;
      if (maxTimeSeconds < 0.45) maxTimeSeconds = 0.45;
      if (maxTimeSeconds > 4.2) maxTimeSeconds = 4.2;

      double kOverM = simParams(gamePiece).kOverM;

      double x = 0.0;
      double y = shooterReleaseHeightMeters;
      double vx = vx0;
      double vy = vy0;
      double t = 0.0;

      double xPrev = 0.0;
      double yPrev = y;
      double tPrev = 0.0;

      double dxStep = targetHorizontalDistanceMeters / 70.0;
      if (dxStep < 0.05) dxStep = 0.05;
      if (dxStep > 0.14) dxStep = 0.14;

      int steps = 0;

      while (t < maxTimeSeconds && y >= -0.25 && x <= targetHorizontalDistanceMeters + 0.9) {
        steps++;

        xPrev = x;
        yPrev = y;
        tPrev = t;

        double vv = vx * vx + vy * vy;
        if (vv < 1e-12) {
          out.set(false, 0.0, Double.POSITIVE_INFINITY);
          return;
        }

        double v = Math.sqrt(vv);
        double ax = -kOverM * v * vx;
        double ay = -g - kOverM * v * vy;

        double avx = vx >= 0.0 ? vx : -vx;
        double dt = dxStep / (avx < 0.35 ? 0.35 : avx);

        vx += ax * dt;
        vy += ay * dt;
        x += vx * dt;
        y += vy * dt;
        t += dt;

        if (xPrev <= targetHorizontalDistanceMeters && x >= targetHorizontalDistanceMeters) {
          double denom = (x - xPrev);
          double frac =
              (Math.abs(denom) > 1e-12) ? (targetHorizontalDistanceMeters - xPrev) / denom : 1.0;
          if (frac < 0.0) frac = 0.0;
          if (frac > 1.0) frac = 1.0;

          double yCross = yPrev + frac * (y - yPrev);
          double tCross = tPrev + frac * (t - tPrev);
          double verticalError = yCross - targetHeightMeters;

          Profiler.counterAdd("DragShotPlanner.sim.steps", steps);
          out.set(true, tCross, verticalError);
          return;
        }
      }

      Profiler.counterAdd("DragShotPlanner.sim.steps", steps);
      out.set(false, 0.0, Double.POSITIVE_INFINITY);
    } finally {
      closeQuietly(_p);
    }
  }

  private static boolean isShooterPoseValidInternal(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.isShooterPoseValidInternal.body");
    try {
      double x = shooterPos.getX();
      double y = shooterPos.getY();
      if (!Double.isFinite(x) || !Double.isFinite(y)) {
        return false;
      }

      final double SHOOT_X_END_BAND_M = 13.49;
      double minBand = SHOOT_X_END_BAND_M;
      double maxBand = Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M;
      if (x < minBand && x > maxBand) {
        return false;
      }

      Rotation2d yaw = targetFieldPosition.minus(shooterPos).getAngle();
      Translation2d[] rect =
          FieldPlanner.robotRect(shooterPos, yaw, robotHalfLengthMeters, robotHalfWidthMeters);

      for (FieldPlanner.Obstacle sObs : staticObstacles()) {
        if (sObs.intersectsRectangle(rect)) {
          return false;
        }
      }

      if (dynamicObstacles != null && !dynamicObstacles.isEmpty()) {
        for (FieldPlanner.Obstacle d : dynamicObstacles) {
          if (d.intersectsRectangle(rect)) {
            return false;
          }
        }
      }

      if (x < 0.0 || x > Constants.FIELD_LENGTH) {
        return false;
      }
      if (y < 0.0 || y > Constants.FIELD_WIDTH) {
        return false;
      }

      return true;
    } finally {
      closeQuietly(_p);
    }
  }
}
