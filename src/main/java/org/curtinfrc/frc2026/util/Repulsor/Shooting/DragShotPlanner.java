package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
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
    private final double launchSpeedMetersPerSecond;
    private final Rotation2d launchAngle;
    private final double timeToPlaneSeconds;
    private final Translation2d impactFieldPosition;
    private final double verticalErrorMeters;

    public ShotSolution(
        Translation2d shooterPosition,
        double launchSpeedMetersPerSecond,
        Rotation2d launchAngle,
        double timeToPlaneSeconds,
        Translation2d impactFieldPosition,
        double verticalErrorMeters) {
      this.shooterPosition = shooterPosition;
      this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
      this.launchAngle = launchAngle;
      this.timeToPlaneSeconds = timeToPlaneSeconds;
      this.impactFieldPosition = impactFieldPosition;
      this.verticalErrorMeters = verticalErrorMeters;
    }

    public Translation2d shooterPosition() {
      return shooterPosition;
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
    final double speed;
    final double angleRad;
    final double timeToPlane;
    final double verticalError;
    final double robotDistance;

    Candidate(
        Translation2d shooterPosition,
        double speed,
        double angleRad,
        double timeToPlane,
        double verticalError,
        double robotDistance) {
      this.shooterPosition = shooterPosition;
      this.speed = speed;
      this.angleRad = angleRad;
      this.timeToPlane = timeToPlane;
      this.verticalError = verticalError;
      this.robotDistance = robotDistance;
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
  private static final double ACCEPTABLE_VERTICAL_ERROR_METERS = 0.06;

  private static final ConcurrentHashMap<String, GamePiecePhysics> GAME_PIECE_CACHE =
      new ConcurrentHashMap<>();

  private DragShotPlanner() {}

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

  public static Optional<ShotSolution> findBestShotAuto(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Pose2d robotPose,
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
    if (maxAngleDeg <= minAngleDeg) {
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

  private static boolean isBetterCandidate(
      Candidate best, Candidate next, Constraints.ShotStyle style) {
    if (best == null) {
      return true;
    }

    if (next.robotDistance < best.robotDistance - EPS) {
      return true;
    }
    if (best.robotDistance < next.robotDistance - EPS) {
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

    double speedStepCoarse = Math.max(0.5, speedRange / 10.0);
    double angleStepCoarse = fixedAngle ? 1.0 : Math.max(2.0, angleRange / 10.0);
    double radialStepCoarse = 0.5;
    double bearingStepDegCoarse = 20.0;
    double coarseTolerance = ACCEPTABLE_VERTICAL_ERROR_METERS * 3.0;

    double minRange = MIN_RANGE_METERS;
    double maxRange = MAX_RANGE_METERS;

    Candidate bestCoarse = null;

    for (double range = minRange; range <= maxRange + 1e-6; range += radialStepCoarse) {
      if (range < MIN_RANGE_METERS) {
        continue;
      }

      for (double bearingDeg = 0.0; bearingDeg < 360.0; bearingDeg += bearingStepDegCoarse) {
        Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
        Translation2d offsetFromTarget = new Translation2d(range, bearing);
        Translation2d shooterPos = targetFieldPosition.minus(offsetFromTarget);

        double robotDistance = robotCurrentPosition.getDistance(shooterPos);
        if (robotDistance > MAX_ROBOT_TRAVEL_METERS) {
          continue;
        }

        if (!isShooterPoseValid(
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
                    shooterPos, speed, angleRad, sim.timeAtPlaneSeconds, error, robotDistance);
            if (isBetterCandidate(bestCoarse, next, shotStyle)) {
              bestCoarse = next;
            }
          }
        }
      }
    }

    return bestCoarse;
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

    Translation2d directionUnit =
        new Translation2d(
            (targetFieldPosition.getX() - shooterFieldPosition.getX()) / horizontalDistance,
            (targetFieldPosition.getY() - shooterFieldPosition.getY()) / horizontalDistance);

    double speedWindow = Math.max(1.5, (maxSpeed - minSpeed) / 8.0);
    double speedMin = Math.max(minSpeed, coarseSpeed - speedWindow);
    double speedMax = Math.min(maxSpeed, coarseSpeed + speedWindow);
    double speedStepFine = Math.max(0.1, (speedMax - speedMin) / 20.0);

    double angleStartDeg;
    double angleEndDeg;
    double angleStepFineDeg;
    double coarseAngleDeg = Math.toDegrees(coarseAngleRad);

    if (fixedAngle) {
      angleStartDeg = minAngleDeg;
      angleEndDeg = maxAngleDeg;
      angleStepFineDeg = 1.0;
    } else {
      double angleWindow = Math.max(4.0, (maxAngleDeg - minAngleDeg) / 8.0);
      angleStartDeg = Math.max(minAngleDeg, coarseAngleDeg - angleWindow);
      angleEndDeg = Math.min(maxAngleDeg, coarseAngleDeg + angleWindow);
      angleStepFineDeg = Math.max(0.25, (angleEndDeg - angleStartDeg) / 24.0);
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
    double baseDt = 0.002;

    double cos = Math.cos(launchAngleRad);
    double horizontalSpeed = launchSpeedMetersPerSecond * Math.abs(cos);
    if (horizontalSpeed < 1e-6) {
      return new SimulationResult(false, 0.0, Double.POSITIVE_INFINITY);
    }

    double minHorizontalSpeed = Math.max(0.5, horizontalSpeed);
    double timeNoDrag = targetHorizontalDistanceMeters / minHorizontalSpeed;
    double maxTimeSeconds = Math.min(5.0, Math.max(0.5, timeNoDrag * 1.5));

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

    while (t < maxTimeSeconds && y >= 0.0 && x <= targetHorizontalDistanceMeters + 1.0) {
      xPrev = x;
      yPrev = y;
      tPrev = t;

      double v = Math.hypot(vx, vy);
      double ax = -kOverM * v * vx;
      double ay = -g - kOverM * v * vy;

      vx += ax * baseDt;
      vy += ay * baseDt;
      x += vx * baseDt;
      y += vy * baseDt;
      t += baseDt;

      if (xPrev <= targetHorizontalDistanceMeters && x >= targetHorizontalDistanceMeters) {
        double frac = (targetHorizontalDistanceMeters - xPrev) / (x - xPrev);
        double yCross = yPrev + frac * (y - yPrev);
        double tCross = tPrev + frac * baseDt;
        double verticalError = yCross - targetHeightMeters;
        return new SimulationResult(true, tCross, verticalError);
      }
    }

    return new SimulationResult(false, 0.0, Double.POSITIVE_INFINITY);
  }

  private static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {

    Rotation2d yaw = targetFieldPosition.minus(shooterPos).getAngle();
    Translation2d[] rect =
        FieldPlanner.robotRect(shooterPos, yaw, robotHalfLengthMeters, robotHalfWidthMeters);

    for (FieldPlanner.Obstacle w : Constants.FIELD.walls()) {
      if (w.intersectsRectangle(rect)) {
        return false;
      }
    }

    for (FieldPlanner.Obstacle f : Constants.FIELD.fieldObstacles()) {
      if (f.intersectsRectangle(rect)) {
        return false;
      }
    }

    if (dynamicObstacles != null) {
      for (FieldPlanner.Obstacle d : dynamicObstacles) {
        if (d.intersectsRectangle(rect)) {
          return false;
        }
      }
    }

    return true;
  }
}
