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

package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.HeadingGate;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.TurnTuning;

public class FieldPlanner {
  private static final double EPS = 1e-9;
  private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
  private static final double FORCE_THROUGH_WALL_DIST = 0.7;
  private static final double CORNER_CHAMFER = 0;
  public static final double GOAL_STRENGTH = 2.2;
  private static final double STAGED_CENTER_BAND_M = 3.648981;
  private static final double STAGED_RESTAGE_DIST_M = 1.5;
  private static final double STAGED_SAME_GOAL_POS_M = 0.05;
  private static final double STAGED_SAME_GOAL_ROT_DEG = 5.0;
  private GatedAttractorObstacle stagedGate = null;

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

    for (int i = 0; i < ARROWS_SIZE; i++) arrows.add(new Pose2d());
    String prefix = System.getenv("REACTIVE_BYPASS_ID");
    String logName;
    if (prefix != null && !prefix.isEmpty()) {
      logName = prefix + "ReactiveBypassLog.csv";
    } else {
      logName = "ReactiveBypassLog.csv";
    }
    // bypass.enableLogging(logName);
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  private static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  static boolean segmentIntersectsPolygonOuter(
      Translation2d a, Translation2d b, Translation2d[] poly) {
    if (isPointInPolygon(a, poly) || isPointInPolygon(b, poly)) return true;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      if (segmentsIntersectOuter(a, b, c, d)) return true;
    }
    return false;
  }

  static double orientOuter(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  static boolean onSegOuter(Translation2d a, Translation2d b, Translation2d p) {
    return p.getX() >= Math.min(a.getX(), b.getX()) - 1e-9
        && p.getX() <= Math.max(a.getX(), b.getX()) + 1e-9
        && p.getY() >= Math.min(a.getY(), b.getY()) - 1e-9
        && p.getY() <= Math.max(a.getY(), b.getY()) + 1e-9;
  }

  static boolean segmentsIntersectOuter(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double o1 = orientOuter(a, b, c);
    double o2 = orientOuter(a, b, d);
    double o3 = orientOuter(c, d, a);
    double o4 = orientOuter(c, d, b);

    if ((o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)) return true;

    if (Math.abs(o1) < 1e-9 && onSegOuter(a, b, c)) return true;
    if (Math.abs(o2) < 1e-9 && onSegOuter(a, b, d)) return true;
    if (Math.abs(o3) < 1e-9 && onSegOuter(c, d, a)) return true;
    if (Math.abs(o4) < 1e-9 && onSegOuter(c, d, b)) return true;

    return false;
  }

  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double rx, double ry) {
    return TurnTuning.robotRect(center, yaw, rx, ry);
  }

  private Function<Translation2d[], Boolean> intersectsDynamicFn(
      List<? extends Obstacle> dynamics) {
    return rect -> rectIntersectsDynamic(rect, dynamics);
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

  private static Rotation2d angleOr(Translation2d v, Rotation2d fallback) {
    return v.getNorm() > EPS ? v.getAngle() : fallback;
  }

  private static Translation2d plus(Translation2d a, Translation2d b) {
    return new Translation2d(a.getX() + b.getX(), a.getY() + b.getY());
  }

  private boolean isInsideChamferedField(Translation2d p) {
    double x = p.getX();
    double y = p.getY();
    double L = Constants.FIELD_LENGTH;
    double W = Constants.FIELD_WIDTH;

    if (x < 0.0 || x > L || y < 0.0 || y > W) return false;

    if (x <= CORNER_CHAMFER && y <= CORNER_CHAMFER && x + y < CORNER_CHAMFER) return false;

    if (x >= L - CORNER_CHAMFER && y <= CORNER_CHAMFER && (L - x) + y < CORNER_CHAMFER)
      return false;

    if (x <= CORNER_CHAMFER && y >= W - CORNER_CHAMFER && x + (W - y) < CORNER_CHAMFER)
      return false;

    if (x >= L - CORNER_CHAMFER && y >= W - CORNER_CHAMFER && (L - x) + (W - y) < CORNER_CHAMFER)
      return false;

    return true;
  }

  private Pose2d goal = Pose2d.kZero;
  private Pose2d requestedGoal = Pose2d.kZero;
  private Translation2d stagedAttractor = null;
  private Translation2d lastStagedPoint = null;
  private boolean stagedComplete = false;

  private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

  private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  private Optional<Distance> currentErr = Optional.empty();
  private Optional<PlannerFallback> fallback = Optional.empty();

  public boolean suppressIsClearPath = false;
  private int stuckStepCount = 0;
  private static final int MAX_STUCK_STEPS = 40;

  private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  public List<Obstacle> getObstacles() {
    return fieldObstacles;
  }

  public Translation2d getGoal() {
    return goal.getTranslation();
  }

  public FieldPlanner withFallback(PlannerFallback _fallback) {
    fallback = Optional.of(_fallback);
    return this;
  }

  public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
    if (RobotBase.isReal()) {
      return;
    }

    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(
                x * Constants.FIELD_LENGTH / ARROWS_X, y * Constants.FIELD_WIDTH / ARROWS_Y);

        int idx = x * (ARROWS_Y + 1) + y;

        if (!isInsideChamferedField(translation)) {
          arrows.set(idx, arrowBackstage);
          continue;
        }

        var force = Force.kZero;
        force = force.plus(getObstacleForce(translation, goal.getTranslation(), dynamicObstacles));
        force = force.plus(getWallForce(translation, goal.getTranslation()));
        force = force.plus(getGoalForce(translation, goal.getTranslation()));

        if (force.getNorm() < 1e-6) {
          arrows.set(idx, arrowBackstage);
        } else {
          var rotation = force.getAngle();
          arrows.set(idx, new Pose2d(translation, rotation));
        }
      }
    }
    // Logger.recordOutput("Repulsor/Arrows", arrows.toArray(new Pose2d[0]));
  }

  public ArrayList<Pose2d> getArrows() {
    return arrows;
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) return new Force();
    var direction = displacement.getAngle();
    var mag =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : walls) force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    var force = Force.kZero;
    var dsBase = RepulsorDriverStation.getInstance();
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    for (Obstacle obs : extra)
      force =
          force.plus(
              obs.getForceAtPosition(curLocation, target)
                  .times(
                      dsBase instanceof NtRepulsorDriverStation ds
                          ? ds.getConfigDouble("repulsion_scale")
                          : 1.0));
    return force;
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target)
        .plus(getObstacleForce(curLocation, target))
        .plus(getWallForce(curLocation, target))
        .times(Math.min(1.0, curLocation.getDistance(target)));
  }

  public void setRequestedGoal(Pose2d requested) {
    boolean same = isPoseNear(this.requestedGoal, requested);
    this.requestedGoal = requested;

    if (!same) {
      this.stagedAttractor = null;
      this.lastStagedPoint = null;
      this.stagedComplete = false;
    }

    lastChosenSetpoint = Optional.empty();
    // Logger.recordOutput("Repulsor/RequestedGoal", requested);
  }

  private void setActiveGoal(Pose2d active) {
    this.goal = active;
    // Logger.recordOutput("Repulsor/Goal/Active", active);
  }

  // public void setGoal(Pose2d goal) {
  //   boolean sameGoal = isPoseNear(this.requestedGoal, goal);
  //   this.goal = goal;
  //   this.requestedGoal = goal;
  //   if (!sameGoal) {
  //     this.stagedAttractor = null;
  //     this.lastStagedPoint = null;
  //     this.stagedComplete = false;
  //   }
  //   lastChosenSetpoint = Optional.empty();
  //   // Logger.recordOutput("Repulsor/Setpoint", goal);
  // }

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
    double distToGoal = curTrans.getDistance(goal.getTranslation());

    var dsBase = RepulsorDriverStation.getInstance();
    if (dsBase instanceof NtRepulsorDriverStation ds) {
      ds.forcedGoalPose("main").ifPresent(this::setRequestedGoal);
    }

    boolean slowDown = updateStagedGoal(curTrans);
    distToGoal = curTrans.getDistance(goal.getTranslation());

    ClearMemo memo = new ClearMemo();

    boolean forceThrough = bypass.isPinnedMode();
    List<? extends Obstacle> effectiveDynamics =
        forceThrough ? Collections.emptyList() : dynamicObstacles;

    if (!forceThrough && !suppressFallback) {
      boolean blockedWithDynamics =
          !ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/WithDyn",
              curTrans,
              goal.getTranslation(),
              dynamicObstacles,
              robot_x,
              robot_y,
              false);

      boolean blockedWithoutDynamics =
          !memo.toGoalNoDyn(curTrans, goal.getTranslation(), robot_x, robot_y);

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
        // Logger.recordOutput("Repulsor/Encapsulated", true);
        currentErr = Optional.of(Meters.of(curTrans.getDistance(goal.getTranslation())));
        return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
      } else {
        // Logger.recordOutput("Repulsor/Encapsulated", false);
      }

      boolean pathBlocked = false;
      if (!suppressIsClearPath) {
        pathBlocked =
            !memo.toGoalDyn(curTrans, goal.getTranslation(), effectiveDynamics, robot_x, robot_y);
      }

      if (pathBlocked && !suppressFallback) {
        Alliance preferred =
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? Alliance.kBlue
                : Alliance.kRed;

        var cands =
            FieldTracker.getInstance().getPredictedSetpoints(preferred, curTrans, 3.5, cat, 8);

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

          if (altGoal.getTranslation().getDistance(goal.getTranslation()) < 1e-3) continue;

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
            // Logger.recordOutput("Repulsor/Reroute/Chosen", altGoal);
            pathBlocked = false;
            break;
          }
        }

        if (pathBlocked) {
          // Logger.recordOutput("Repulsor/Reroute/Chosen", "None_StayStill");
          return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
        }
      }
    }

    final List<? extends Obstacle> effectiveDynamicsFinal = effectiveDynamics;

    updateArrows(effectiveDynamicsFinal);

    var err = curTrans.minus(goal.getTranslation());
    currentErr = Optional.of(Meters.of(err.getNorm()));

    if (err.getNorm() < 0.04) {
      return new RepulsorSample(curTrans, 0, 0, Radians.of(goal.getRotation().getRadians()));
    }

    if (fallback.isPresent() && fallback.get().within(err)) {
      // Logger.recordOutput("Repulsor/Fallback/Engaged", true);
      var speeds = fallback.get().calculate(curTrans, goal.getTranslation());
      return new RepulsorSample(
          goal.getTranslation(), speeds, Radians.of(pose.getRotation().getRadians()));
    }
    // Logger.recordOutput("Repulsor/Fallback/Engaged", false);

    var obstacleForceToGoal =
        getObstacleForce(curTrans, goal.getTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, goal.getTranslation()));
    var netForceToGoal = getGoalForce(curTrans, goal.getTranslation()).plus(obstacleForceToGoal);
    Rotation2d headingToGoal = netForceToGoal.getAngle();

    var maybeBypass =
        bypass.update(
            pose,
            goal,
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
                    goal.getTranslation(),
                    effectiveDynamicsFinal,
                    robot_x,
                    robot_y,
                    true));

    Pose2d effectiveGoal = maybeBypass.orElse(goal);
    // Logger.recordOutput("Repulsor/Goal/Effective", effectiveGoal);

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
      // Logger.recordOutput("Repulsor/ArrowFinal", arrowPose);
    }

    return new RepulsorSample(
        effectiveGoal.getTranslation(),
        step.getX() / driveTuning.dtSeconds(),
        step.getY() / driveTuning.dtSeconds(),
        Radians.of(turn.yaw.getRadians()));
  }

  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    int crossings = 0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d a = polygon[i];
      Translation2d b = polygon[(i + 1) % polygon.length];
      boolean cond1 = (a.getY() > point.getY()) != (b.getY() > point.getY());
      double slope =
          (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX();
      if (cond1 && point.getX() < slope) crossings++;
    }
    return (crossings % 2 == 1);
  }

  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }

  public Optional<RepulsorSetpoint> pollChosenSetpoint() {
    var out = lastChosenSetpoint;
    lastChosenSetpoint = Optional.empty();
    return out;
  }

  private static final double STAGED_REACH_ENTER_M = 0.35;
  private static final double STAGED_REACH_EXIT_M = 0.55;
  private static final int STAGED_REACH_TICKS = 3;
  private int stagedReachTicks = 0;

  private static final int STAGED_MAX_TICKS = 40;
  private int stagedModeTicks = 0;
  private boolean stagedGatePassed = false;

  private static final double STAGED_LANE_WEIGHT = 2.0;
  private static final double STAGED_PREF_GATE_PENALTY = 2.0;

  private Translation2d stagedLatchedPull = null;

  private static final double STAGED_GATE_PAD_M = 0.25;
  private static final double STAGED_PASSED_X_HYST_M = 0.35;
  private boolean stagedUsingBypass = false;

  private static Translation2d polyCentroid(Translation2d[] poly) {
    if (poly == null || poly.length == 0) return null;
    double sx = 0.0, sy = 0.0;
    for (Translation2d p : poly) {
      sx += p.getX();
      sy += p.getY();
    }
    return new Translation2d(sx / poly.length, sy / poly.length);
  }

  private static Translation2d[] expandPoly(Translation2d[] poly, double pad) {
    if (poly == null || poly.length == 0) return poly;
    if (pad <= 1e-9) return poly;

    Translation2d c = polyCentroid(poly);
    if (c == null) return poly;

    Translation2d[] out = new Translation2d[poly.length];
    for (int i = 0; i < poly.length; i++) {
      Translation2d v = poly[i].minus(c);
      double n = v.getNorm();
      if (n < 1e-9) out[i] = poly[i];
      else out[i] = c.plus(v.div(n).times(n + pad));
    }
    return out;
  }

  private static double firstIntersectionT(Translation2d a, Translation2d b, Translation2d[] poly) {
    if (poly == null || poly.length < 3) return Double.POSITIVE_INFINITY;

    if (FieldPlanner.isPointInPolygon(a, poly)) return 0.0;

    double bestT = Double.POSITIVE_INFINITY;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      Double t = segmentIntersectionParam(a, b, c, d);
      if (t != null && t >= 0.0 && t <= 1.0 && t < bestT) bestT = t;
    }
    return bestT;
  }

  private static Double segmentIntersectionParam(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {

    double ax = a.getX(), ay = a.getY();
    double bx = b.getX(), by = b.getY();
    double cx = c.getX(), cy = c.getY();
    double dx = d.getX(), dy = d.getY();

    double rpx = bx - ax;
    double rpy = by - ay;
    double spx = dx - cx;
    double spy = dy - cy;

    double rxs = rpx * spy - rpy * spx;
    double qpx = cx - ax;
    double qpy = cy - ay;

    double qpxr = qpx * rpy - qpy * rpx;

    double eps = 1e-9;
    if (Math.abs(rxs) < eps) {
      return null;
    }

    double t = (qpx * spy - qpy * spx) / rxs;
    double u = qpxr / rxs;

    if (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps) {
      if (t < 0.0) t = 0.0;
      if (t > 1.0) t = 1.0;
      return t;
    }

    return null;
  }

  private static boolean gateIsBehind(
      Translation2d pos, Translation2d goal, GatedAttractorObstacle gate) {
    if (pos == null || goal == null || gate == null || gate.center == null) return false;
    Translation2d toGoal = goal.minus(pos);
    double n = toGoal.getNorm();
    if (n <= 1e-6) return false;
    Translation2d dir = toGoal.div(n);
    Translation2d toGate = gate.center.minus(pos);
    double proj = toGate.getX() * dir.getX() + toGate.getY() * dir.getY();
    return proj < -STAGED_PASSED_X_HYST_M;
  }

  private GatedAttractorObstacle firstOccludingGateAlongSegment(
      Translation2d pos, Translation2d target) {
    if (gatedAttractors.isEmpty() || pos == null || target == null) return null;

    GatedAttractorObstacle best = null;
    double bestT = Double.POSITIVE_INFINITY;
    double bestLane = Double.POSITIVE_INFINITY;
    double laneY = 0.5 * (pos.getY() + target.getY());
    final double tieEps = 1e-6;

    for (GatedAttractorObstacle gate : gatedAttractors) {
      if (gate == null || gate.gatePoly == null) continue;
      if (gateIsBehind(pos, target, gate)) continue;

      Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);

      if (!segmentIntersectsPolygonOuter(pos, target, poly)) continue;

      double t = firstIntersectionT(pos, target, poly);
      if (t < bestT - tieEps) {
        bestT = t;
        bestLane = Math.abs(gate.center.getY() - laneY);
        best = gate;
      } else if (Math.abs(t - bestT) <= tieEps) {
        double lane = Math.abs(gate.center.getY() - laneY);
        if (lane < bestLane) {
          bestLane = lane;
          best = gate;
        }
      }
    }

    return best;
  }

  private static int sideSignXBand(double x, double band) {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  private static boolean isPoseNear(Pose2d a, Pose2d b) {
    if (a == null || b == null) return false;
    if (a.getTranslation().getDistance(b.getTranslation()) > STAGED_SAME_GOAL_POS_M) return false;
    double rotDeg =
        Math.abs(
            MathUtil.angleModulus(a.getRotation().getRadians() - b.getRotation().getRadians()));
    return rotDeg <= Math.toRadians(STAGED_SAME_GOAL_ROT_DEG);
  }

  private boolean shouldStageThroughAttractor(Translation2d pos, Translation2d target) {
    int goalSide = sideSignXBand(target.getX(), STAGED_CENTER_BAND_M);
    int robotSide = sideSignXBand(pos.getX(), STAGED_CENTER_BAND_M);
    if (goalSide == 0 && robotSide != 0) return true;
    if (goalSide != 0 && robotSide == 0) return true;
    return goalSide != 0 && robotSide != 0 && goalSide != robotSide;
  }

  private Translation2d stagingPullPoint(
      GatedAttractorObstacle gate, Translation2d pos, Translation2d target) {
    if (gate == null) return null;
    Translation2d center = gate.center;
    if (center == null) return null;

    if (gate.gatePoly == null || gate.bypassPoint == null) return center;
    if (pos == null || target == null) return center;

    if (gate == stagedGate && stagedLatchedPull != null) return stagedLatchedPull;

    Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);
    boolean hit = segmentIntersectsPolygonOuter(pos, target, poly);

    Translation2d inside = gate.bypassPoint;
    Translation2d outside = new Translation2d(2.0 * center.getX() - inside.getX(), inside.getY());

    boolean targetOnRight = target.getX() > center.getX();
    boolean insideOnRight = inside.getX() > center.getX();

    Translation2d pick =
        hit
            ? (targetOnRight
                ? (insideOnRight ? inside : outside)
                : (insideOnRight ? outside : inside))
            : center;

    if (gate == stagedGate) stagedLatchedPull = pick;
    return pick;
  }

  private GatedAttractorObstacle chooseBestGateByScore(
      Translation2d pos, Translation2d target, GatedAttractorObstacle preferredGate) {

    GatedAttractorObstacle best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    double laneY = 0.5 * (pos.getY() + target.getY());

    for (GatedAttractorObstacle gate : gatedAttractors) {
      if (gate == null) continue;
      if (gateIsBehind(pos, target, gate)) continue;

      Translation2d pullTo = stagingPullPoint(gate, pos, target);
      if (pullTo == null) continue;

      double prefPenalty =
          (preferredGate != null && gate != preferredGate) ? STAGED_PREF_GATE_PENALTY : 0.0;
      double lanePenalty = STAGED_LANE_WEIGHT * Math.abs(gate.center.getY() - laneY);

      double score =
          pos.getDistance(pullTo) + pullTo.getDistance(target) + prefPenalty + lanePenalty;

      if (score < bestScore) {
        bestScore = score;
        best = gate;
      }
    }

    return best;
  }

  private boolean updateStagedGoal(Translation2d curPos) {
    if (gatedAttractors.isEmpty()) {
      goal = requestedGoal;
      stagedAttractor = null;
      stagedGate = null;
      stagedUsingBypass = false;
      stagedGatePassed = false;
      stagedLatchedPull = null;
      lastStagedPoint = null;
      stagedComplete = false;
      stagedReachTicks = 0;
      stagedModeTicks = 0;
      return true;
    }

    Translation2d reqT = requestedGoal.getTranslation();
    GatedAttractorObstacle firstBlock = firstOccludingGateAlongSegment(curPos, reqT);

    boolean shouldStage = shouldStageThroughAttractor(curPos, reqT) || (firstBlock != null);
    if (stagedComplete && lastStagedPoint != null) {
      double d = curPos.getDistance(lastStagedPoint);
      if (d < STAGED_RESTAGE_DIST_M) {
        shouldStage = false;
      } else {
        stagedComplete = false;
        lastStagedPoint = null;
      }
    }

    if (shouldStage && stagedAttractor == null) {
      GatedAttractorObstacle gateToUse =
          (firstBlock != null) ? firstBlock : chooseBestGateByScore(curPos, reqT, null);

      if (gateToUse != null) {
        stagedGate = gateToUse;
        stagedLatchedPull = null;

        stagedUsingBypass =
            (stagedGate.gatePoly != null && stagedGate.bypassPoint != null)
                && segmentIntersectsPolygonOuter(
                    curPos, reqT, expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M));

        Translation2d pick = stagingPullPoint(stagedGate, curPos, reqT);

        if (pick != null && curPos.getDistance(pick) > STAGED_REACH_EXIT_M) {
          stagedAttractor = pick;
          lastStagedPoint = pick;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGatePassed = false;
          stagedComplete = false;

          Pose2d staged = new Pose2d(pick, requestedGoal.getRotation());
          setActiveGoal(staged);
          // Logger.recordOutput("Repulsor/StagedGoal", staged);
          return false;
        } else {
          stagedAttractor = null;
          stagedGate = null;
          stagedUsingBypass = false;
          stagedGatePassed = false;
          stagedLatchedPull = null;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
        }
      }
    }

    if (stagedAttractor != null) {
      stagedModeTicks++;

      Translation2d liveTarget =
          (stagedGate != null) ? stagingPullPoint(stagedGate, curPos, reqT) : stagedAttractor;
      if (liveTarget == null) liveTarget = stagedAttractor;

      double d = curPos.getDistance(liveTarget);

      if (d <= STAGED_REACH_ENTER_M) stagedReachTicks++;
      else if (d >= STAGED_REACH_EXIT_M) stagedReachTicks = 0;

      boolean reached = stagedReachTicks >= STAGED_REACH_TICKS;

      if (stagedGate != null) {
        stagedGatePassed = stagedGatePassed || gateIsBehind(curPos, reqT, stagedGate);
      }

      boolean gateCleared = stagedGatePassed;

      if (stagedModeTicks >= STAGED_MAX_TICKS) {
        GatedAttractorObstacle nowFirst = firstOccludingGateAlongSegment(curPos, reqT);
        if (nowFirst == null || stagedGatePassed) {
          reached = true;
          gateCleared = true;
        } else if (stagedGate != null && nowFirst != stagedGate) {
          stagedGate = nowFirst;
          stagedLatchedPull = null;

          stagedUsingBypass =
              (stagedGate.gatePoly != null && stagedGate.bypassPoint != null)
                  && segmentIntersectsPolygonOuter(
                      curPos, reqT, expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M));

          Translation2d repick = stagingPullPoint(stagedGate, curPos, reqT);
          if (repick != null) {
            stagedAttractor = repick;
            lastStagedPoint = repick;
            stagedReachTicks = 0;
            stagedModeTicks = 0;
            stagedGatePassed = false;
            goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
            // Logger.recordOutput("Repulsor/StagedGoal", goal);
            return false;
          }
        }
      }

      if (reached && gateCleared) {
        lastStagedPoint = liveTarget != null ? liveTarget : stagedAttractor;
        stagedAttractor = null;
        stagedGate = null;
        stagedUsingBypass = false;
        stagedGatePassed = false;
        stagedLatchedPull = null;
        lastStagedPoint = null;
        stagedReachTicks = 0;
        stagedModeTicks = 0;
        stagedComplete = true;

        goal = requestedGoal;
        // Logger.recordOutput("Repulsor/StagedGoal", goal);
        return true;
      }

      if (liveTarget.getDistance(stagedAttractor) > 0.02) {
        stagedAttractor = liveTarget;
        goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
        // Logger.recordOutput("Repulsor/StagedGoal", goal);
      } else {
        goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
        // Logger.recordOutput("Repulsor/StagedGoal", goal);
      }

      return false;
    }

    goal = requestedGoal;
    return true;
  }
}
