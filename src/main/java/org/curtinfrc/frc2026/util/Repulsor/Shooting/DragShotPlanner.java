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
    }

    public boolean done() {
      return done;
    }

    public int size() {
      return entries.size();
    }

    public ShotLibrary snapshot(boolean completeFlag) {
      return new ShotLibrary(
          targetFieldPosition,
          targetHeightMeters,
          shooterReleaseHeightMeters,
          robotHalfLengthMeters,
          robotHalfWidthMeters,
          constraints,
          List.copyOf(entries),
          completeFlag);
    }

    public ShotLibrary maybeStep(long budgetNanos) {
      if (done) {
        return null;
      }
      long start = System.nanoTime();
      ShotLibrary publish = null;

      while (!done && (System.nanoTime() - start) < budgetNanos) {
        Translation2d shooterPos =
            targetFieldPosition.minus(new Translation2d(range, Rotation2d.fromDegrees(bearingDeg)));

        if (isShooterPoseValid(
            shooterPos, targetFieldPosition, robotHalfLengthMeters, robotHalfWidthMeters, null)) {

          ShotSolution bestAtPos =
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

          if (bestAtPos != null) {
            entries.add(
                new ShotLibraryEntry(
                    bestAtPos.shooterPosition(),
                    bestAtPos.shooterYaw().getRadians(),
                    bestAtPos.launchSpeedMetersPerSecond(),
                    bestAtPos.launchAngle().getRadians(),
                    bestAtPos.timeToPlaneSeconds(),
                    bestAtPos.verticalErrorMeters()));
            publishCounter++;
            if (publishCounter >= 8) {
              publishCounter = 0;
              publish = snapshot(false);
            }
          }
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

      return publish;
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

  private static final class SimulationResult {
    final boolean hitPlane;
    final double timeAtPlaneSeconds;
    final double verticalErrorMeters;

    SimulationResult(boolean hitPlane, double timeAtPlaneSeconds, double verticalErrorMeters) {
      this.hitPlane = hitPlane;
      this.timeAtPlaneSeconds = timeAtPlaneSeconds;
      this.verticalErrorMeters = verticalErrorMeters;
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

  private static final double EPS = 1e-6;
  private static final double MIN_RANGE_METERS = 0.5;
  private static final double MAX_RANGE_METERS = 7.0;
  private static final double MAX_ROBOT_TRAVEL_METERS = 7.0;
  private static final double MAX_ROBOT_TRAVEL_METERS_SQ =
      MAX_ROBOT_TRAVEL_METERS * MAX_ROBOT_TRAVEL_METERS;
  private static final double ACCEPTABLE_VERTICAL_ERROR_METERS = 0.06;

  private static final ConcurrentHashMap<String, GamePiecePhysics> GAME_PIECE_CACHE =
      new ConcurrentHashMap<>();

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

  private static List<FieldPlanner.Obstacle> staticObstacles() {
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
    return STATIC_OBSTACLES;
  }

  public static GamePiecePhysics loadGamePieceFromDeployYaml(String id) {
    if (id == null || id.isEmpty()) {
      throw new IllegalArgumentException("id must be non-empty");
    }
    return GAME_PIECE_CACHE.computeIfAbsent(
        id, DragShotPlanner::loadGamePieceFromDeployYamlInternal);
  }

  private static GamePiecePhysics loadGamePieceFromDeployYamlInternal(String id) {
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
  }

  public static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {
    return isShooterPoseValidInternal(
        shooterPos,
        targetFieldPosition,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        dynamicObstacles);
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

    if (library == null || library.entries().isEmpty()) {
      return Optional.empty();
    }

    Translation2d robotCurrentPosition = robotPose.getTranslation();
    Candidate best = null;

    boolean needDynamicCheck = dynamicObstacles != null && !dynamicObstacles.isEmpty();

    double rx = robotCurrentPosition.getX();
    double ry = robotCurrentPosition.getY();

    for (ShotLibraryEntry e : library.entries()) {
      Translation2d shooterPos = e.shooterPosition();

      double dx = rx - shooterPos.getX();
      double dy = ry - shooterPos.getY();
      double robotDistanceSq = dx * dx + dy * dy;
      if (robotDistanceSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
        continue;
      }

      if (needDynamicCheck
          && !isShooterPoseValidInternal(
              shooterPos,
              targetFieldPosition,
              robotHalfLengthMeters,
              robotHalfWidthMeters,
              dynamicObstacles)) {
        continue;
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
    }

    if (best == null) {
      return Optional.empty();
    }

    ShotSolution refined =
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

    double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
    double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
    double minAngleDeg = constraints.minLaunchAngleDeg();
    double maxAngleDeg = constraints.maxLaunchAngleDeg();

    if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
      return Optional.empty();
    }
    if (!(Math.abs(maxAngleDeg - minAngleDeg) < 1e-6) && maxAngleDeg <= minAngleDeg) {
      return Optional.empty();
    }

    Translation2d robotCurrentPosition = robotPose.getTranslation();
    boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

    Candidate coarse =
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

    if (coarse == null) {
      return Optional.empty();
    }

    ShotSolution refined =
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

    if (refined == null) {
      return Optional.empty();
    }

    return Optional.of(refined);
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

    if (gamePiece == null
        || targetFieldPosition == null
        || robotPose == null
        || constraints == null
        || state == null) {
      return Optional.empty();
    }

    double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
    double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
    double minAngleDeg = constraints.minLaunchAngleDeg();
    double maxAngleDeg = constraints.maxLaunchAngleDeg();
    boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

    if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
      return Optional.empty();
    }
    if (!fixedAngle && maxAngleDeg <= minAngleDeg) {
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

    ShotSolution seedSol =
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

    if (seedSol != null
        && isShooterPoseValidInternal(
            seed,
            targetFieldPosition,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            dynamicObstacles)) {
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

    long start = System.nanoTime();
    double step = Math.max(0.11, Math.min(0.95, state.stepMeters()));
    Translation2d current = seed;

    while ((System.nanoTime() - start) < budgetNanos) {
      boolean improved = false;

      double cx = current.getX();
      double cy = current.getY();

      for (double[] o : ONLINE_OFFS) {
        Translation2d p = new Translation2d(cx + o[0] * step, cy + o[1] * step);
        Translation2d clipped = clipToRingAndField(p, targetFieldPosition);

        double dxr = robotX - clipped.getX();
        double dyr = robotY - clipped.getY();
        double distSq = dxr * dxr + dyr * dyr;
        if (distSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
          continue;
        }

        if (!isShooterPoseValidInternal(
            clipped,
            targetFieldPosition,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            dynamicObstacles)) {
          continue;
        }

        ShotSolution sol =
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

        if (sol == null) {
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
        }
      }

      if (!improved) {
        step *= 0.58;
        if (step < 0.075) {
          break;
        }
      }
    }

    if (best == null) {
      return Optional.empty();
    }

    state.seed(best.shooterPosition);
    state.stepMeters(Math.max(0.10, Math.min(0.80, step)));
    state.touch();

    ShotSolution refined =
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
  }

  private static Translation2d projectToValidRing(Translation2d point, Translation2d target) {
    Translation2d delta = point.minus(target);
    double r = delta.getNorm();
    if (r < 1e-6) {
      return target.minus(new Translation2d(3.0, Rotation2d.kZero));
    }
    double clamped = Math.max(MIN_RANGE_METERS, Math.min(MAX_RANGE_METERS, r));
    Translation2d unit = delta.div(r);
    return target.plus(unit.times(clamped));
  }

  private static Translation2d clipToRingAndField(Translation2d p, Translation2d target) {
    Translation2d ring = projectToValidRing(p, target);
    double x = ring.getX();
    double y = ring.getY();
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      return target.minus(new Translation2d(3.0, Rotation2d.kZero));
    }
    return new Translation2d(x, y);
  }

  private static boolean isBetterCandidate(
      Candidate best, Candidate next, Constraints.ShotStyle style) {
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

    for (double range = MIN_RANGE_METERS;
        range <= MAX_RANGE_METERS + 1e-6;
        range += radialStepCoarse) {
      for (double bearingDeg = 0.0; bearingDeg < 360.0; bearingDeg += bearingStepDegCoarse) {
        Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
        Translation2d shooterPos = targetFieldPosition.minus(new Translation2d(range, bearing));

        double dx = rx - shooterPos.getX();
        double dy = ry - shooterPos.getY();
        double robotDistanceSq = dx * dx + dy * dy;
        if (robotDistanceSq > MAX_ROBOT_TRAVEL_METERS_SQ) {
          continue;
        }

        if (!isShooterPoseValidInternal(
            shooterPos,
            targetFieldPosition,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            dynamicObstacles)) {
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

          for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += speedStepCoarse) {
            SimulationResult sim =
                simulateToTargetPlane(
                    gamePiece,
                    speed,
                    angleRad,
                    shooterReleaseHeightMeters,
                    horizontalDistance,
                    targetHeightMeters);

            if (!sim.hitPlane) {
              continue;
            }

            double error = Math.abs(sim.verticalErrorMeters);
            if (error > coarseTolerance) {
              continue;
            }

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

    return bestCoarse;
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

    double horizontalDistance = shooterFieldPosition.getDistance(targetFieldPosition);
    if (horizontalDistance < 1e-3) {
      return null;
    }

    double invD = 1.0 / horizontalDistance;
    Translation2d directionUnit =
        new Translation2d(
            (targetFieldPosition.getX() - shooterFieldPosition.getX()) * invD,
            (targetFieldPosition.getY() - shooterFieldPosition.getY()) * invD);

    Rotation2d shooterYaw = targetFieldPosition.minus(shooterFieldPosition).getAngle();

    double angleStep = fixedAngle ? 1.0 : Math.max(0.12, angleStepDeg);

    double bestErrorAbs = Double.POSITIVE_INFINITY;
    double bestAngleRad = 0.0;
    double bestSpeed = 0.0;
    double bestTime = 0.0;
    double bestSignedErr = 0.0;
    boolean found = false;

    for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg + 1e-6; angleDeg += angleStep) {
      double angleRad = Math.toRadians(angleDeg);
      double cos = Math.cos(angleRad);
      if (cos <= 0.0) {
        continue;
      }

      for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += speedStep) {
        SimulationResult sim =
            simulateToTargetPlane(
                gamePiece,
                speed,
                angleRad,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);

        if (!sim.hitPlane) {
          continue;
        }

        double errAbs = Math.abs(sim.verticalErrorMeters);
        if (errAbs > acceptableVerticalErrorMeters) {
          continue;
        }

        boolean take;
        if (!found) {
          take = true;
        } else {
          double errEps = 1e-6;
          if (errAbs < bestErrorAbs - errEps) {
            take = true;
          } else if (Math.abs(errAbs - bestErrorAbs) <= errEps) {
            if (shotStyle == Constraints.ShotStyle.DIRECT
                || shotStyle == Constraints.ShotStyle.ARC) {
              double aBest = Math.abs(bestAngleRad);
              double aNext = Math.abs(angleRad);
              double aEps = 1e-3;
              if (shotStyle == Constraints.ShotStyle.DIRECT) {
                if (aNext < aBest - aEps) {
                  take = true;
                } else if (aBest < aNext - aEps) {
                  take = false;
                } else {
                  take = speed < bestSpeed - EPS;
                }
              } else {
                if (aNext > aBest + aEps) {
                  take = true;
                } else if (aBest > aNext + aEps) {
                  take = false;
                } else {
                  take = speed < bestSpeed - EPS;
                }
              }
            } else {
              take = speed < bestSpeed - EPS;
            }
          } else {
            take = false;
          }
        }

        if (take) {
          found = true;
          bestErrorAbs = errAbs;
          bestAngleRad = angleRad;
          bestSpeed = speed;
          bestTime = sim.timeAtPlaneSeconds;
          bestSignedErr = sim.verticalErrorMeters;
        }
      }
    }

    if (!found) {
      return null;
    }

    Translation2d impactPos = shooterFieldPosition.plus(directionUnit.times(horizontalDistance));

    return new ShotSolution(
        shooterFieldPosition,
        shooterYaw,
        bestSpeed,
        Rotation2d.fromRadians(bestAngleRad),
        bestTime,
        impactPos,
        bestSignedErr);
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

    double horizontalDistance = shooterFieldPosition.getDistance(targetFieldPosition);
    if (horizontalDistance < 1e-3) {
      return null;
    }

    double invD = 1.0 / horizontalDistance;
    Translation2d directionUnit =
        new Translation2d(
            (targetFieldPosition.getX() - shooterFieldPosition.getX()) * invD,
            (targetFieldPosition.getY() - shooterFieldPosition.getY()) * invD);

    Rotation2d shooterYaw = targetFieldPosition.minus(shooterFieldPosition).getAngle();

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

    for (double angleDeg = angleStartDeg;
        angleDeg <= angleEndDeg + 1e-6;
        angleDeg += angleStepFineDeg) {

      double angleRad = Math.toRadians(angleDeg);
      double cos = Math.cos(angleRad);
      if (cos <= 0.0) {
        continue;
      }

      for (double speed = speedMin; speed <= speedMax + 1e-6; speed += speedStepFine) {

        SimulationResult sim =
            simulateToTargetPlane(
                gamePiece,
                speed,
                angleRad,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);

        if (!sim.hitPlane) {
          continue;
        }

        double error = Math.abs(sim.verticalErrorMeters);
        if (error > acceptableVerticalErrorMeters) {
          continue;
        }

        if (best == null
            || error < bestError - EPS
            || (Math.abs(error - bestError) <= EPS
                && speed < best.launchSpeedMetersPerSecond() - EPS)) {
          Translation2d impactPos =
              shooterFieldPosition.plus(directionUnit.times(horizontalDistance));
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

    return best;
  }

  private static SimulationResult simulateToTargetPlane(
      GamePiecePhysics gamePiece,
      double launchSpeedMetersPerSecond,
      double launchAngleRad,
      double shooterReleaseHeightMeters,
      double targetHorizontalDistanceMeters,
      double targetHeightMeters) {

    double g = 9.81;

    double cos = Math.cos(launchAngleRad);
    double horizontalSpeed = launchSpeedMetersPerSecond * cos;
    if (horizontalSpeed < 1e-6) {
      return new SimulationResult(false, 0.0, Double.POSITIVE_INFINITY);
    }

    double minHorizontalSpeed = Math.max(0.5, horizontalSpeed);
    double timeNoDrag = targetHorizontalDistanceMeters / minHorizontalSpeed;
    double maxTimeSeconds = Math.min(5.0, Math.max(0.5, timeNoDrag * 1.55));

    double m = gamePiece.massKg();
    double A = gamePiece.crossSectionAreaM2();
    double Cd = gamePiece.dragCoefficient();
    double rho = gamePiece.airDensityKgPerM3();
    double kOverM = 0.5 * rho * Cd * A / m;

    double x = 0.0;
    double y = shooterReleaseHeightMeters;
    double vx = launchSpeedMetersPerSecond * cos;
    double vy = launchSpeedMetersPerSecond * Math.sin(launchAngleRad);
    double t = 0.0;

    double xPrev = x;
    double yPrev = y;
    double tPrev = t;

    double dxStep = 0.025;

    while (t < maxTimeSeconds && y >= -0.25 && x <= targetHorizontalDistanceMeters + 1.0) {
      xPrev = x;
      yPrev = y;
      tPrev = t;

      double v = Math.hypot(vx, vy);
      double ax = -kOverM * v * vx;
      double ay = -g - kOverM * v * vy;

      double dt = dxStep / Math.max(0.25, Math.abs(vx));

      vx += ax * dt;
      vy += ay * dt;
      x += vx * dt;
      y += vy * dt;
      t += dt;

      if (xPrev <= targetHorizontalDistanceMeters && x >= targetHorizontalDistanceMeters) {
        double denom = (x - xPrev);
        double frac =
            Math.abs(denom) > 1e-12 ? (targetHorizontalDistanceMeters - xPrev) / denom : 1.0;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        double yCross = yPrev + frac * (y - yPrev);
        double tCross = tPrev + frac * (t - tPrev);
        double verticalError = yCross - targetHeightMeters;
        return new SimulationResult(true, tCross, verticalError);
      }
    }

    return new SimulationResult(false, 0.0, Double.POSITIVE_INFINITY);
  }

  private static boolean isShooterPoseValidInternal(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {

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

    for (FieldPlanner.Obstacle s : staticObstacles()) {
      if (s.intersectsRectangle(rect)) {
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

    if (shooterPos.getX() < 0.0 || shooterPos.getX() > Constants.FIELD_LENGTH) {
      return false;
    }
    if (shooterPos.getY() < 0.0 || shooterPos.getY() > Constants.FIELD_WIDTH) {
      return false;
    }

    return true;
  }
}
