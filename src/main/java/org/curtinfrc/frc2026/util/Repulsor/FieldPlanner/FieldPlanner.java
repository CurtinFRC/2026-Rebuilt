/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerForceModel;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerGeometry;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers.FieldPlannerGoalManager;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.HeadingGate;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.TurnTuning;

public class FieldPlanner {
  private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
  private static final double FORCE_THROUGH_WALL_DIST = 0.7;
  public static final double GOAL_STRENGTH = 2.2;

  private static final class ClearMemo {
    Boolean toGoalDyn;
    Boolean toGoalNoDyn;

    boolean toGoalDyn(
        Translation2d a, Translation2d b, List<? extends Obstacle> dyn, double rx, double ry) {
      if (toGoalDyn != null) return toGoalDyn.booleanValue();
      toGoalDyn = ExtraPathing.isClearPath("Repulsor/IsClear", a, b, dyn, rx, ry, true);
      return toGoalDyn.booleanValue();
    }

    boolean toGoalNoDyn(Translation2d a, Translation2d b, double rx, double ry) {
      if (toGoalNoDyn != null) return toGoalNoDyn.booleanValue();
      toGoalNoDyn =
          ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/NoDyn", a, b, Collections.emptyList(), rx, ry, false);
      return toGoalNoDyn.booleanValue();
    }
  }

  public interface ObstacleProvider {
    List<Obstacle> fieldObstacles();

    List<Obstacle> walls();
  }

  public static final class DefaultObstacleProvider implements ObstacleProvider {
    @Override
    public List<Obstacle> fieldObstacles() {
      return List.of();
    }

    @Override
    public List<Obstacle> walls() {
      return List.of();
    }
  }

  private Optional<RepulsorSetpoint> lastChosenSetpoint = Optional.empty();

  private final TurnTuning turnTuning;
  private final DriveTuning driveTuning;
  public final ReactiveBypass bypass = new ReactiveBypass();
  private final HeadingGate headingGate = new HeadingGate();

  private final ObstacleProvider obstacleProvider;
  private final List<Obstacle> fieldObstacles;
  private final List<Obstacle> walls;
  private final List<GatedAttractorObstacle> gatedAttractors = new ArrayList<>();

  private final FieldPlannerForceModel forceModel;
  private final FieldPlannerGoalManager goalManager;

  private Optional<Distance> currentErr = Optional.empty();
  private Optional<PlannerFallback> fallback = Optional.empty();

  public boolean suppressIsClearPath = false;
  private int stuckStepCount = 0;
  private static final int MAX_STUCK_STEPS = 40;

  public ObstacleProvider getObstacleProvider() {
    return obstacleProvider;
  }

  public FieldPlanner() {
    this(new DefaultTurnTuning(), new DefaultDriveTuning(), Constants.FIELD);
  }

  public FieldPlanner(ObstacleProvider obstacleProvider, DriveTuning driveTuning) {
    this(new DefaultTurnTuning(), driveTuning, obstacleProvider);
  }

  public FieldPlanner(TurnTuning turnTuning, DriveTuning driveTuning) {
    this(turnTuning, driveTuning, new DefaultObstacleProvider());
  }

  public FieldPlanner(
      TurnTuning turnTuning, DriveTuning driveTuning, ObstacleProvider obstacleProvider) {
    this.turnTuning = turnTuning;
    this.driveTuning = driveTuning;
    this.obstacleProvider =
        obstacleProvider == null ? new DefaultObstacleProvider() : obstacleProvider;
    this.fieldObstacles = new ArrayList<>(this.obstacleProvider.fieldObstacles());
    this.walls = new ArrayList<>(this.obstacleProvider.walls());

    for (Obstacle obs : this.fieldObstacles) {
      if (obs instanceof GatedAttractorObstacle gated) {
        if (gated.waypoint) {
          gatedAttractors.add(gated);
        }
      }
    }

    this.forceModel = new FieldPlannerForceModel(fieldObstacles, walls);
    this.goalManager = new FieldPlannerGoalManager(gatedAttractors);

    String prefix = System.getenv("REACTIVE_BYPASS_ID");
    String logName;
    if (prefix != null && !prefix.isEmpty()) {
      logName = prefix + "ReactiveBypassLog.csv";
    } else {
      logName = "ReactiveBypassLog.csv";
    }
    // bypass.enableLogging(logName);
  }

  public static boolean segmentIntersectsPolygonOuter(
      Translation2d a, Translation2d b, Translation2d[] poly) {
    return FieldPlannerGeometry.segmentIntersectsPolygonOuter(a, b, poly);
  }

  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double rx, double ry) {
    return TurnTuning.robotRect(center, yaw, rx, ry);
  }

  private boolean rectIntersectsDynamic(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  private boolean rectIntersectsAny(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle w : walls) if (w.intersectsRectangle(rect)) return true;
    for (Obstacle f : fieldObstacles) if (f.intersectsRectangle(rect)) return true;
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  public List<Obstacle> getObstacles() {
    return fieldObstacles;
  }

  public Translation2d getGoal() {
    return goalManager.getGoalTranslation();
  }

  public FieldPlanner withFallback(PlannerFallback _fallback) {
    fallback = Optional.of(_fallback);
    return this;
  }

  public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
    forceModel.updateArrows(goalManager.getGoalTranslation(), dynamicObstacles);
  }

  public ArrayList<Pose2d> getArrows() {
    return forceModel.getArrows();
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    return forceModel.getGoalForce(curLocation, goal);
  }

  Force getWallForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getWallForce(curLocation, target);
  }

  Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    return forceModel.getObstacleForce(curLocation, target, extra);
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getObstacleForce(curLocation, target);
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    return forceModel.getForce(curLocation, target);
  }

  public void setRequestedGoal(Pose2d requested) {
    goalManager.setRequestedGoal(requested);
    lastChosenSetpoint = Optional.empty();
  }

  void setActiveGoal(Pose2d active) {
    goalManager.setActiveGoal(active);
  }

  public Optional<Distance> getErr() {
    return currentErr;
  }

  public void clearCommitted() {}

  public RepulsorSample calculateAndClear(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      CategorySpec cat,
      double shooterReleaseHeightMeters) {
    return calculate(
        pose,
        dynamicObstacles,
        robot_x,
        robot_y,
        coral_offset,
        algae_offset,
        cat,
        false,
        shooterReleaseHeightMeters);
  }

  public RepulsorSample calculate(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      CategorySpec cat,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {

    Translation2d curTrans = pose.getTranslation();
    double distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());

    var dsBase = RepulsorDriverStation.getInstance();
    if (dsBase instanceof NtRepulsorDriverStation ds) {
      ds.forcedGoalPose("main").ifPresent(this::setRequestedGoal);
    }

    boolean slowDown = goalManager.updateStagedGoal(curTrans, dynamicObstacles);
    distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());

    ClearMemo memo = new ClearMemo();

    boolean forceThrough = bypass.isPinnedMode();
    List<? extends Obstacle> effectiveDynamics =
        forceThrough ? Collections.emptyList() : dynamicObstacles;

    if (!forceThrough && !suppressFallback) {
      boolean blockedWithDynamics =
          !ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/WithDyn",
              curTrans,
              goalManager.getGoalTranslation(),
              dynamicObstacles,
              robot_x,
              robot_y,
              false);

      boolean blockedWithoutDynamics =
          !memo.toGoalNoDyn(curTrans, goalManager.getGoalTranslation(), robot_x, robot_y);

      double dxWall = Math.min(curTrans.getX(), Constants.FIELD_LENGTH - curTrans.getX());
      double dyWall = Math.min(curTrans.getY(), Constants.FIELD_WIDTH - curTrans.getY());
      double dWall = Math.min(dxWall, dyWall);
      boolean nearWall = dWall < FORCE_THROUGH_WALL_DIST;
      boolean nearGoal = distToGoal <= FORCE_THROUGH_GOAL_DIST;

      if (blockedWithDynamics && !blockedWithoutDynamics && nearGoal && nearWall) {
        forceThrough = true;
        effectiveDynamics = Collections.emptyList();
      }
    }

    if (!suppressFallback) {
      if (!forceThrough
          && ExtraPathing.robotIntersects(curTrans, robot_x, robot_y, dynamicObstacles)) {
        currentErr = Optional.of(Meters.of(curTrans.getDistance(goalManager.getGoalTranslation())));
        return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
      }

      boolean pathBlocked = false;
      if (!suppressIsClearPath) {
        pathBlocked =
            !memo.toGoalDyn(
                curTrans, goalManager.getGoalTranslation(), effectiveDynamics, robot_x, robot_y);
      }

      if (pathBlocked && !suppressFallback) {
        Alliance preferred =
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? Alliance.kBlue  
                : Alliance.kRed;

        var cands =
            FieldTrackerCore.getInstance().getPredictedSetpoints(preferred, curTrans, 3.5, cat, 8);

        SetpointContext spCtx =
            new SetpointContext(
                Optional.of(pose),
                Math.max(0.0, robot_x) * 2.0,
                Math.max(0.0, robot_y) * 2.0,
                coral_offset,
                algae_offset,
                shooterReleaseHeightMeters,
                effectiveDynamics);

        for (RepulsorSetpoint sp : cands) {
          Pose2d altGoal = sp.get(spCtx);

          if (altGoal.getTranslation().getDistance(goalManager.getGoalTranslation()) < 1e-3)
            continue;

          boolean clear =
              ExtraPathing.isClearPath(
                  "Repulsor/IsClear/Reroute",
                  curTrans,
                  altGoal.getTranslation(),
                  effectiveDynamics,
                  robot_x,
                  robot_y,
                  true);

          if (clear) {
            setActiveGoal(altGoal);
            lastChosenSetpoint = Optional.of(sp);
            pathBlocked = false;
            break;
          }
        }

        if (pathBlocked) {
          return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
        }
      }
    }

    final List<? extends Obstacle> effectiveDynamicsFinal = effectiveDynamics;

    updateArrows(effectiveDynamicsFinal);

    var err = curTrans.minus(goalManager.getGoalTranslation());
    currentErr = Optional.of(Meters.of(err.getNorm()));

    if (err.getNorm() < 0.04) {
      return new RepulsorSample(
          curTrans, 0, 0, Radians.of(goalManager.getGoalPose().getRotation().getRadians()));
    }

    if (fallback.isPresent() && fallback.get().within(err)) {
      var speeds = fallback.get().calculate(curTrans, goalManager.getGoalTranslation());
      return new RepulsorSample(
          goalManager.getGoalTranslation(), speeds, Radians.of(pose.getRotation().getRadians()));
    }

    var obstacleForceToGoal =
        getObstacleForce(curTrans, goalManager.getGoalTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, goalManager.getGoalTranslation()));
    var netForceToGoal =
        getGoalForce(curTrans, goalManager.getGoalTranslation()).plus(obstacleForceToGoal);
    Rotation2d headingToGoal = netForceToGoal.getAngle();

    var maybeBypass =
        bypass.update(
            pose,
            goalManager.getGoalPose(),
            headingToGoal,
            driveTuning.dtSeconds(),
            robot_x,
            robot_y,
            effectiveDynamicsFinal,
            rect -> rectIntersectsDynamic(rect, effectiveDynamicsFinal),
            tag ->
                ExtraPathing.isClearPath(
                    "Repulsor/Bypass/Rejoin",
                    curTrans,
                    goalManager.getGoalTranslation(),
                    effectiveDynamicsFinal,
                    robot_x,
                    robot_y,
                    true));

    Pose2d effectiveGoal = maybeBypass.orElse(goalManager.getGoalPose());

    var obstacleForce =
        getObstacleForce(curTrans, effectiveGoal.getTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, effectiveGoal.getTranslation()));
    var netForce = getGoalForce(curTrans, effectiveGoal.getTranslation()).plus(obstacleForce);
    var dist = curTrans.getDistance(effectiveGoal.getTranslation());

    double stepSize_m =
        driveTuning.stepSizeMeters(
            dist, obstacleForce.getNorm(), (cat == CategorySpec.kScore), slowDown);
    var step = new Translation2d(stepSize_m, netForce.getAngle());

    if (step.getNorm() < 1e-3) {
      stuckStepCount++;
    } else {
      stuckStepCount = 0;
    }

    if (stuckStepCount >= MAX_STUCK_STEPS) {
      System.out.println("[Repulsor] Stuck! Aborting after " + stuckStepCount + " tiny steps.");
      return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
    }

    Rotation2d desiredHeadingRaw =
        (cat == CategorySpec.kCollect) ? effectiveGoal.getRotation() : netForce.getAngle();
    Rotation2d desiredHeading =
        headingGate.filter(pose.getRotation(), desiredHeadingRaw, driveTuning.dtSeconds());

    var turn =
        turnTuning.plan(
            pose,
            effectiveGoal,
            desiredHeading,
            step,
            (cat == CategorySpec.kScore),
            robot_x,
            robot_y,
            rect -> rectIntersectsAny(rect, effectiveDynamicsFinal));

    step = step.times(turn.speedScale);

    if (!RobotBase.isReal()) {
      Pose2d arrowPose = new Pose2d(curTrans, netForce.getAngle());
    }

    return new RepulsorSample(
        effectiveGoal.getTranslation(),
        step.getX() / driveTuning.dtSeconds(),
        step.getY() / driveTuning.dtSeconds(),
        Radians.of(turn.yaw.getRadians()));
  }

  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    return FieldPlannerGeometry.isPointInPolygon(point, polygon);
  }

  public static double dot(Translation2d a, Translation2d b) {
    return FieldPlannerGeometry.dot(a, b);
  }

  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    return FieldPlannerGeometry.distanceFromPointToSegment(p, a, b);
  }

  public Optional<RepulsorSetpoint> pollChosenSetpoint() {
    var out = lastChosenSetpoint;
    lastChosenSetpoint = Optional.empty();
    return out;
  }
}
