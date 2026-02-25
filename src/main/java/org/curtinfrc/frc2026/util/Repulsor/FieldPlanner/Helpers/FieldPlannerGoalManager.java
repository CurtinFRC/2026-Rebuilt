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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.littletonrobotics.junction.Logger;

public final class FieldPlannerGoalManager {
  private static final double STAGED_CENTER_BAND_M = 3.648981;
  private static final double STAGED_RESTAGE_DIST_M = 1.5;
  private static final double STAGED_SAME_GOAL_POS_M = 0.05;
  private static final double STAGED_SAME_GOAL_ROT_DEG = 5.0;

  private static final double STAGED_REACH_ENTER_M = 0.35;
  private static final double STAGED_REACH_EXIT_M = 0.55;
  private static final double STAGED_GATE_CLEAR_EXIT_M = 0.40;
  private static final int STAGED_REACH_TICKS = 3;
  private static final int STAGED_GATE_CLEAR_TICKS = 2;

  private static final int STAGED_MAX_TICKS = 40;

  private static final double STAGED_LANE_WEIGHT = 2.0;
  private static final double STAGED_PREF_GATE_PENALTY = 2.0;
  private static final double STAGED_LANE_LOCK_WEIGHT = 2.5;
  private static final double STAGED_LANE_LOCK_MAX_DELTA_M = 3.0;

  private static final double STAGED_GATE_PAD_M = 0.25;
  private static final double STAGED_PASSED_X_HYST_M = 0.35;
  private static final double STAGED_PASSED_TARGET_PROJ_M = 0.16;
  private static final double STAGED_GOAL_SIDE_PROJ_M = 0.05;
  private static final double STAGED_LEAD_THROUGH_SCALE = 0.28;
  private static final double STAGED_LEAD_THROUGH_MIN_M = 0.45;
  private static final double STAGED_LEAD_THROUGH_MAX_M = 1.05;
  private static final double STAGED_DEEP_CENTER_BAND_M = 1.40;
  private static final double STAGED_CENTER_RETURN_STAGE_TRIGGER_M = 3.0;
  private static final double STAGED_CENTER_RETURN_INTERSECTION_TRIGGER_M = 4.2;
  private static final double STAGED_CENTER_RETURN_EXIT_MIN_M = 0.70;
  private static final double STAGED_CENTER_RETURN_EXIT_MAX_M = 2.40;
  private static final double STAGED_CENTER_RETURN_GATE_MIN_OFFSET_M = 2.0;
  private static final double STAGED_FIELD_EDGE_MARGIN_M = 0.35;

  private Pose2d goal = Pose2d.kZero;
  private Pose2d requestedGoal = Pose2d.kZero;
  private Translation2d stagedAttractor = null;
  private Translation2d lastStagedPoint = null;
  private boolean stagedComplete = false;
  private GatedAttractorObstacle stagedGate = null;
  private int stagedReachTicks = 0;
  private int stagedModeTicks = 0;
  private boolean stagedGatePassed = false;
  private int stagedGateClearTicks = 0;
  private Translation2d stagedLatchedPull = null;
  private boolean stagedUsingBypass = false;
  private Double stagedLaneY = null;
  private boolean stagedCenterReturn = false;
  private boolean stagedExitPhase = false;
  private Translation2d stagedExitPoint = null;

  private final List<GatedAttractorObstacle> gatedAttractors;

  public FieldPlannerGoalManager(List<GatedAttractorObstacle> gatedAttractors) {
    this.gatedAttractors = gatedAttractors;
  }

  public Pose2d getGoalPose() {
    return goal;
  }

  public Translation2d getGoalTranslation() {
    return goal.getTranslation();
  }

  public void setRequestedGoal(Pose2d requested) {
    boolean same = isPoseNear(this.requestedGoal, requested);
    this.requestedGoal = requested;

    if (!same) {
      this.stagedAttractor = null;
      this.lastStagedPoint = null;
      this.stagedComplete = false;
      this.stagedLaneY = null;
      this.stagedCenterReturn = false;
      this.stagedExitPhase = false;
      this.stagedExitPoint = null;
    }
  }

  public void setActiveGoal(Pose2d active) {
    this.goal = active;
    Logger.recordOutput("ActiveGoal", this.goal);
  }

  public boolean updateStagedGoal(Translation2d curPos, List<? extends Obstacle> obstacles) {
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
      stagedGateClearTicks = 0;
      stagedLaneY = null;
      stagedCenterReturn = false;
      stagedExitPhase = false;
      stagedExitPoint = null;
      return true;
    }

    Translation2d reqT = requestedGoal.getTranslation();
    GatedAttractorObstacle firstBlock = firstOccludingGateAlongSegment(curPos, reqT);

    boolean stageForOccludingGate =
        firstBlock != null && !shouldDeferCenterReturnStage(curPos, reqT, firstBlock);
    boolean stageByBand = shouldStageThroughAttractor(curPos, reqT);
    boolean shouldStage = stageByBand || stageForOccludingGate;
    boolean allowImmediateCenterExitRestage = stageByBand && isCenterReturnTransition(curPos, reqT);
    if (stagedComplete && lastStagedPoint != null) {
      double d = curPos.getDistance(lastStagedPoint);
      if (d < STAGED_RESTAGE_DIST_M && !stageForOccludingGate && !allowImmediateCenterExitRestage) {
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
        if (stagedGate.center != null) stagedLaneY = stagedGate.center.getY();
        stagedCenterReturn =
            isCenterReturnTransition(curPos, reqT) && isCorridorSideGate(stagedGate);
        stagedExitPhase = false;
        stagedExitPoint =
            stagedCenterReturn ? computeCenterReturnExitPoint(stagedGate, reqT) : null;

        stagedUsingBypass =
            (stagedGate.gatePoly != null && stagedGate.bypassPoint != null)
                && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                    curPos, reqT, expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M));

        Translation2d pick = stagingPullPoint(stagedGate, curPos, reqT);

        boolean mustStageForOccludingGate = stageForOccludingGate;
        if (pick != null
            && (mustStageForOccludingGate
                || shouldStage
                || curPos.getDistance(pick) > STAGED_REACH_EXIT_M)) {
          stagedAttractor = pick;
          lastStagedPoint = pick;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGatePassed = false;
          stagedGateClearTicks = 0;
          stagedComplete = false;

          Pose2d staged = new Pose2d(pick, requestedGoal.getRotation());
          setActiveGoal(staged);
          return false;
        } else {
          stagedAttractor = null;
          stagedGate = null;
          stagedUsingBypass = false;
          stagedGatePassed = false;
          stagedLatchedPull = null;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGateClearTicks = 0;
          stagedCenterReturn = false;
          stagedExitPhase = false;
          stagedExitPoint = null;
        }
      }
    }

    if (stagedAttractor != null) {
      stagedModeTicks++;

      Translation2d liveTarget =
          (stagedGate != null) ? stagingPullPoint(stagedGate, curPos, reqT) : stagedAttractor;
      if (liveTarget == null) liveTarget = stagedAttractor;
      boolean passedLiveTargetTowardGoal =
          hasPassedPointTowardGoal(curPos, liveTarget, reqT, STAGED_PASSED_TARGET_PROJ_M);

      double d = curPos.getDistance(liveTarget);

      if (d <= STAGED_REACH_ENTER_M) stagedReachTicks++;
      else if (d >= STAGED_REACH_EXIT_M) stagedReachTicks = 0;

      boolean reached = stagedReachTicks >= STAGED_REACH_TICKS;

      if (stagedGate != null) {
        boolean gateOccludingNow =
            stagedGate.gatePoly != null
                && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                    curPos, reqT, expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M));
        stagedGatePassed =
            stagedGatePassed
                || gateIsBehind(curPos, reqT, stagedGate)
                || gateOnGoalSide(curPos, reqT, stagedGate)
                || !gateOccludingNow
                || passedLiveTargetTowardGoal;
      }

      boolean gateCleared = (stagedGate == null) || stagedGatePassed;
      if (gateCleared) stagedGateClearTicks++;
      else stagedGateClearTicks = 0;

      if (!reached
          && stagedGate != null
          && stagedGateClearTicks >= STAGED_GATE_CLEAR_TICKS
          && (passedLiveTargetTowardGoal || d <= STAGED_GATE_CLEAR_EXIT_M)) {
        reached = true;
      }
      if (!reached && stagedGate == null && passedLiveTargetTowardGoal) reached = true;

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
                  && FieldPlannerGeometry.segmentIntersectsPolygonOuter(
                      curPos, reqT, expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M));

          Translation2d repick = stagingPullPoint(stagedGate, curPos, reqT);
          if (repick != null) {
            stagedAttractor = repick;
            lastStagedPoint = repick;
            stagedReachTicks = 0;
            stagedModeTicks = 0;
            stagedGatePassed = false;
            stagedGateClearTicks = 0;
            goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
            return false;
          }
        }
      }

      if (reached && gateCleared) {
        if (stagedCenterReturn && !stagedExitPhase && stagedExitPoint != null) {
          stagedExitPhase = true;
          stagedAttractor = stagedExitPoint;
          lastStagedPoint = stagedExitPoint;
          stagedGate = null;
          stagedUsingBypass = false;
          stagedGatePassed = true;
          stagedLatchedPull = null;
          stagedReachTicks = 0;
          stagedModeTicks = 0;
          stagedGateClearTicks = 0;
          goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
          return false;
        }

        lastStagedPoint = liveTarget != null ? liveTarget : stagedAttractor;
        stagedAttractor = null;
        stagedGate = null;
        stagedUsingBypass = false;
        stagedGatePassed = false;
        stagedLatchedPull = null;
        stagedReachTicks = 0;
        stagedModeTicks = 0;
        stagedGateClearTicks = 0;
        stagedComplete = true;
        stagedCenterReturn = false;
        stagedExitPhase = false;
        stagedExitPoint = null;

        goal = requestedGoal;
        return true;
      }

      if (liveTarget.getDistance(stagedAttractor) > 0.02) {
        stagedAttractor = liveTarget;
        goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
      } else {
        goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
      }

      return false;
    }

    goal = requestedGoal;
    return true;
  }

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

    if (FieldPlannerGeometry.isPointInPolygon(a, poly)) return 0.0;

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

  private static boolean gateOnGoalSide(
      Translation2d pos, Translation2d goal, GatedAttractorObstacle gate) {
    if (pos == null || goal == null || gate == null || gate.center == null) return false;
    Translation2d gateToGoal = goal.minus(gate.center);
    double n = gateToGoal.getNorm();
    if (n <= 1e-6) return false;
    Translation2d gateToPos = pos.minus(gate.center);
    double proj = (gateToPos.getX() * gateToGoal.getX() + gateToPos.getY() * gateToGoal.getY()) / n;
    return proj >= STAGED_GOAL_SIDE_PROJ_M;
  }

  private GatedAttractorObstacle firstOccludingGateAlongSegment(
      Translation2d pos, Translation2d target) {
    if (gatedAttractors.isEmpty() || pos == null || target == null) return null;

    GatedAttractorObstacle best = null;
    double bestT = Double.POSITIVE_INFINITY;
    double bestLaneMetric = Double.POSITIVE_INFINITY;
    double laneY = 0.5 * (pos.getY() + target.getY());
    final double tieEps = 1e-6;

    for (GatedAttractorObstacle gate : gatedAttractors) {
      if (gate == null || gate.gatePoly == null) continue;
      if (gateIsBehind(pos, target, gate)) continue;

      Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);

      if (!FieldPlannerGeometry.segmentIntersectsPolygonOuter(pos, target, poly)) continue;
      double t = firstIntersectionT(pos, target, poly);
      if (t < bestT - tieEps) {
        bestT = t;
        bestLaneMetric = laneDistanceMetric(gate, laneY);
        best = gate;
      } else if (Math.abs(t - bestT) <= tieEps) {
        double laneMetric = laneDistanceMetric(gate, laneY);
        if (laneMetric < bestLaneMetric) {
          bestLaneMetric = laneMetric;
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
    // When exiting deep center toward an alliance side, avoid forced staging.
    // This reduces stop/slow behavior in open corridor return paths.
    if (goalSide != 0 && robotSide == 0) {
      double mid = Constants.FIELD_LENGTH * 0.5;
      boolean deepCenter = Math.abs(pos.getX() - mid) <= STAGED_DEEP_CENTER_BAND_M;
      return !deepCenter;
    }
    return goalSide != 0 && robotSide != 0 && goalSide != robotSide;
  }

  private boolean shouldDeferCenterReturnStage(
      Translation2d pos, Translation2d target, GatedAttractorObstacle gate) {
    if (pos == null || target == null || gate == null || gate.center == null) return false;

    int goalSide = sideSignXBand(target.getX(), STAGED_CENTER_BAND_M);
    int robotSide = sideSignXBand(pos.getX(), STAGED_CENTER_BAND_M);
    if (!(goalSide != 0 && robotSide == 0)) return false;

    double mid = Constants.FIELD_LENGTH * 0.5;
    boolean deepCenter = Math.abs(pos.getX() - mid) <= STAGED_DEEP_CENTER_BAND_M;
    if (!deepCenter) return false;

    Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);
    double t = firstIntersectionT(pos, target, poly);
    if (Double.isFinite(t) && t >= 0.0 && t <= 1.0) {
      double segDist = pos.getDistance(target);
      double hitDist = segDist * t;
      return hitDist > STAGED_CENTER_RETURN_INTERSECTION_TRIGGER_M;
    }

    return pos.getDistance(gate.center) > STAGED_CENTER_RETURN_STAGE_TRIGGER_M;
  }

  private boolean isCenterReturnTransition(Translation2d pos, Translation2d target) {
    int goalSide = sideSignXBand(target.getX(), STAGED_CENTER_BAND_M);
    int robotSide = sideSignXBand(pos.getX(), STAGED_CENTER_BAND_M);
    return goalSide != 0 && robotSide == 0;
  }

  private boolean isCorridorSideGate(GatedAttractorObstacle gate) {
    if (gate == null || gate.center == null) return false;
    double mid = Constants.FIELD_LENGTH * 0.5;
    return Math.abs(gate.center.getX() - mid) >= STAGED_CENTER_RETURN_GATE_MIN_OFFSET_M;
  }

  private Translation2d computeCenterReturnExitPoint(
      GatedAttractorObstacle gate, Translation2d target) {
    if (gate == null || gate.center == null || target == null) return null;

    double dx = target.getX() - gate.center.getX();
    double sign = Math.signum(dx);
    if (sign == 0.0) {
      double mid = Constants.FIELD_LENGTH * 0.5;
      sign = gate.center.getX() >= mid ? 1.0 : -1.0;
    }

    double advance =
        MathUtil.clamp(
            Math.abs(dx) * 0.45, STAGED_CENTER_RETURN_EXIT_MIN_M, STAGED_CENTER_RETURN_EXIT_MAX_M);

    double x =
        MathUtil.clamp(
            gate.center.getX() + sign * advance,
            STAGED_FIELD_EDGE_MARGIN_M,
            Constants.FIELD_LENGTH - STAGED_FIELD_EDGE_MARGIN_M);
    double yBase = stagedLaneY != null ? stagedLaneY.doubleValue() : gate.center.getY();
    double y =
        MathUtil.clamp(
            yBase, STAGED_FIELD_EDGE_MARGIN_M, Constants.FIELD_WIDTH - STAGED_FIELD_EDGE_MARGIN_M);

    return new Translation2d(x, y);
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
    boolean hit = FieldPlannerGeometry.segmentIntersectsPolygonOuter(pos, target, poly);

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

    pick = extendPullPointThroughGate(pick, center, target);

    if (gate == stagedGate) stagedLatchedPull = pick;
    return pick;
  }

  private static boolean hasPassedPointTowardGoal(
      Translation2d pos, Translation2d point, Translation2d goal, double projMeters) {
    if (pos == null || point == null || goal == null) return false;

    Translation2d toGoal = goal.minus(point);
    double n = toGoal.getNorm();
    if (n <= 1e-6) return pos.getDistance(point) <= Math.max(0.0, projMeters);

    Translation2d dir = toGoal.div(n);
    Translation2d toPos = pos.minus(point);
    double proj = toPos.getX() * dir.getX() + toPos.getY() * dir.getY();
    return proj >= Math.max(0.0, projMeters);
  }

  private static Translation2d extendPullPointThroughGate(
      Translation2d pullPoint, Translation2d gateCenter, Translation2d target) {
    if (pullPoint == null || gateCenter == null || target == null) return pullPoint;

    double dx = target.getX() - gateCenter.getX();
    double sign = Math.signum(dx);
    if (Math.abs(sign) < 1e-9) return pullPoint;

    double lead =
        MathUtil.clamp(
            Math.abs(dx) * STAGED_LEAD_THROUGH_SCALE,
            STAGED_LEAD_THROUGH_MIN_M,
            STAGED_LEAD_THROUGH_MAX_M);

    double x =
        MathUtil.clamp(
            pullPoint.getX() + sign * lead,
            STAGED_FIELD_EDGE_MARGIN_M,
            Constants.FIELD_LENGTH - STAGED_FIELD_EDGE_MARGIN_M);
    double y =
        MathUtil.clamp(
            pullPoint.getY(),
            STAGED_FIELD_EDGE_MARGIN_M,
            Constants.FIELD_WIDTH - STAGED_FIELD_EDGE_MARGIN_M);
    return new Translation2d(x, y);
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
      double laneLockPenalty = laneLockPenalty(gate);

      double score =
          pos.getDistance(pullTo)
              + pullTo.getDistance(target)
              + prefPenalty
              + lanePenalty
              + laneLockPenalty;

      if (score < bestScore) {
        bestScore = score;
        best = gate;
      }
    }

    return best;
  }

  private double laneDistanceMetric(GatedAttractorObstacle gate, double laneY) {
    if (gate == null || gate.center == null) return Double.POSITIVE_INFINITY;
    return Math.abs(gate.center.getY() - laneY) + laneLockPenalty(gate);
  }

  private double laneLockPenalty(GatedAttractorObstacle gate) {
    if (stagedLaneY == null || gate == null || gate.center == null) return 0.0;
    double dy = Math.abs(gate.center.getY() - stagedLaneY.doubleValue());
    if (dy <= 1e-6) return 0.0;
    double scaled = Math.min(dy, STAGED_LANE_LOCK_MAX_DELTA_M) / STAGED_LANE_LOCK_MAX_DELTA_M;
    return STAGED_LANE_LOCK_WEIGHT * scaled;
  }
}
