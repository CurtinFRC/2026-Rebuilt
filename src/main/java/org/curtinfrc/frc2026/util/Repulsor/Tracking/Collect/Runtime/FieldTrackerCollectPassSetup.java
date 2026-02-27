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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveLoop;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath;

public final class FieldTrackerCollectPassSetup {
  private FieldTrackerCollectPassSetup() {}

  public static FieldTrackerCollectPassSetupResult prepare(
      FieldTrackerCollectObjectiveLoop loop, Pose2d robotPoseBlue, double cap) {
    Translation2d[] pts = loop.collectObjectivePoints.get();

    if (pts == null || pts.length == 0) {
      loop.clearCollectSticky();
      return new FieldTrackerCollectPassSetupResult(null, loop.fallbackCollectPose(robotPoseBlue));
    }

    final double FORBID_MARGIN_M = 0.6;

    final double robotHalf =
        0.5
            * Math.max(
                org.curtinfrc.frc2026.Constants.ROBOT_X, org.curtinfrc.frc2026.Constants.ROBOT_Y);
    final double robotWallMargin = robotHalf + 0.03;

    double L = Constants.FIELD_LENGTH;
    double halfW = 1.1938 * 0.5 + FORBID_MARGIN_M;

    double leftSqCx = 4.625594;
    double leftRectCx = (L * 0.5) - 3.63982;

    double rightSqCx = L - 4.625594;
    double rightRectCx = (L * 0.5) + 3.63982;

    double leftBandX0 = Math.min(leftSqCx, leftRectCx) - halfW;
    double leftBandX1 = Math.max(leftSqCx, leftRectCx) + halfW;

    double rightBandX0 = Math.min(rightSqCx, rightRectCx) - halfW;
    double rightBandX1 = Math.max(rightSqCx, rightRectCx) + halfW;

    Function<Translation2d, Translation2d> clampToFieldRobotSafe =
        p -> {
          double x = p.getX();
          double y = p.getY();
          double minX = robotWallMargin;
          double maxX = Constants.FIELD_LENGTH - robotWallMargin;
          double minY = robotWallMargin;
          double maxY = Constants.FIELD_WIDTH - robotWallMargin;
          if (x < minX) x = minX;
          if (x > maxX) x = maxX;
          if (y < minY) y = minY;
          if (y > maxY) y = maxY;
          return new Translation2d(x, y);
        };

    Function<Translation2d, Double> wallClearance =
        p -> {
          double x = p.getX();
          double y = p.getY();
          double dL = x;
          double dR = Constants.FIELD_LENGTH - x;
          double dB = y;
          double dT = Constants.FIELD_WIDTH - y;
          return Math.min(Math.min(dL, dR), Math.min(dB, dT));
        };

    Translation2d robotPos = robotPoseBlue.getTranslation();

    Function<Translation2d, Pose2d> holdPose =
        p -> {
          Translation2d h = p;
          if (h == null) h = loop.collectStickyDriveTarget;
          if (h == null) h = robotPos;
          return new Pose2d(h, robotPoseBlue.getRotation());
        };

    Predicate<Translation2d> violatesWall = p -> wallClearance.apply(p) < robotWallMargin;

    Predicate<Translation2d> inForbidden =
        p -> {
          double x = p.getX();
          return (x >= leftBandX0 && x <= leftBandX1) || (x >= rightBandX0 && x <= rightBandX1);
        };

    Function<Translation2d, Translation2d> nudgeOutOfForbidden =
        p -> {
          if (!inForbidden.test(p)) return p;
          double x = p.getX();
          double y = p.getY();
          double pad = 0.06;
          if (x >= leftBandX0 && x <= leftBandX1) {
            double dl = Math.abs(x - leftBandX0);
            double dr = Math.abs(leftBandX1 - x);
            x = (dl < dr) ? (leftBandX0 - pad) : (leftBandX1 + pad);
          } else if (x >= rightBandX0 && x <= rightBandX1) {
            double dl = Math.abs(x - rightBandX0);
            double dr = Math.abs(rightBandX1 - x);
            x = (dl < dr) ? (rightBandX0 - pad) : (rightBandX1 + pad);
          }
          return clampToFieldRobotSafe.apply(new Translation2d(x, y));
        };

    Function<Translation2d, Translation2d> safePushedFromRobot =
        resource -> {
          Translation2d dir = resource.minus(robotPos);
          double n = dir.getNorm();
          if (n <= 1e-6) return clampToFieldRobotSafe.apply(resource);

          double dist = robotPos.getDistance(resource);
          double pushT = FieldTrackerCollectObjectiveMath.clamp01((dist - 0.35) / (2.50 - 0.35));
          double pushM =
              Math.min(
                  FieldTrackerCollectObjectiveMath.lerp(0.42, 0.18, pushT),
                  FieldTrackerCollectObjectiveLoop.COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M);

          Translation2d dirHat = dir.div(n);

          Translation2d targetFull =
              clampToFieldRobotSafe.apply(resource.plus(dirHat.times(pushM)));
          if (!inForbidden.test(targetFull) && !violatesWall.test(targetFull)) return targetFull;

          double lo = 0.0;
          double hi = 1.0;
          Translation2d best = clampToFieldRobotSafe.apply(resource);
          if (!inForbidden.test(best) && !violatesWall.test(best)) return best;

          for (int i = 0; i < 16; i++) {
            double mid = 0.5 * (lo + hi);
            Translation2d cand =
                clampToFieldRobotSafe.apply(resource.plus(dirHat.times(pushM * mid)));
            boolean ok2 = !inForbidden.test(cand) && !violatesWall.test(cand);
            if (ok2) {
              best = cand;
              lo = mid;
            } else {
              hi = mid;
            }
          }

          best = nudgeOutOfForbidden.apply(best);
          best = clampToFieldRobotSafe.apply(best);
          return best;
        };

    ArrayList<Translation2d> ok = new ArrayList<>(pts.length);
    for (Translation2d p : pts) {
      if (p == null) continue;
      Translation2d drive = safePushedFromRobot.apply(p);
      drive = clampToFieldRobotSafe.apply(drive);
      if (inForbidden.test(drive)) drive = nudgeOutOfForbidden.apply(drive);
      drive = clampToFieldRobotSafe.apply(drive);
      if (violatesWall.test(drive)) continue;
      if (!inForbidden.test(drive)) ok.add(p);
    }

    Translation2d[] usePts = ok.toArray(new Translation2d[0]);
    if (usePts.length == 0) {
      loop.clearCollectSticky();
      return new FieldTrackerCollectPassSetupResult(null, loop.fallbackCollectPose(robotPoseBlue));
    }

    double midX = Constants.FIELD_LENGTH * 0.5;
    boolean robotInCenterBand =
        Math.abs(robotPos.getX() - midX)
            <= FieldTrackerCollectObjectiveLoop.COLLECT_HALF_KEEP_MID_BAND_M;

    List<DynamicObject> dynAll = loop.snapshotDynamicObjects();

    int lockHalf = 0;
    int sensedHalf = 0;

    List<DynamicObject> dynUse = loop.filterDynamicsForCollectPredictor(dynAll);

    loop.predictor.setDynamicObjects(dynUse);

    long nowNs = System.nanoTime();
    long prevNs = loop.lastObjectiveTickNs;
    loop.lastObjectiveTickNs = nowNs;
    double dt = prevNs != 0L ? (nowNs - prevNs) / 1e9 : 0.02;
    if (dt < 1e-3) dt = 1e-3;
    if (dt > 0.08) dt = 0.08;

    double rawMoveM = robotPos.getDistance(loop.collectStickyStillLastPos);
    loop.collectStickyStillLastPos = robotPos;

    double a = 1.0 - Math.exp(-dt / 0.12);
    loop.collectStickyStillFiltPos = loop.collectStickyStillFiltPos.interpolate(robotPos, a);

    double filtMoveM =
        loop.collectStickyStillFiltPos.getDistance(loop.collectStickyStillFiltLastPos);
    if (filtMoveM <= 0.02) loop.collectStickyStillSec += dt;
    else loop.collectStickyStillSec = 0.0;
    loop.collectStickyStillFiltLastPos = loop.collectStickyStillFiltPos;

    int halfNow = FieldTrackerCollectObjectiveMath.sideSignXBand(robotPos.getX(), 0.08);
    if (halfNow != 0
        && loop.collectStickyRobotHalfLast != 0
        && halfNow != loop.collectStickyRobotHalfLast
        && rawMoveM <= 0.06) {
      loop.collectStickyRobotFlickerSec += dt;
    } else {
      loop.collectStickyRobotFlickerSec = 0.0;
    }
    if (halfNow != 0) loop.collectStickyRobotHalfLast = halfNow;

    int nearbyFuelCount = 0;
    double nearbySX = 0.0;
    double nearbySY = 0.0;

    if (dynUse != null && !dynUse.isEmpty()) {
      for (int i = 0; i < dynUse.size(); i++) {
        DynamicObject o = dynUse.get(i);
        if (!loop.isFreshCollectObservation(o)) continue;
        double d = o.pos.getDistance(robotPos);
        if (d <= FieldTrackerCollectObjectiveLoop.COLLECT_NEARBY_RADIUS_M) {
          nearbyFuelCount++;
          nearbySX += o.pos.getX();
          nearbySY += o.pos.getY();
        }
      }
    }

    Translation2d nearbyCentroid =
        nearbyFuelCount > 0
            ? new Translation2d(nearbySX / nearbyFuelCount, nearbySY / nearbyFuelCount)
            : null;

    FieldTrackerCollectPassContext context =
        new FieldTrackerCollectPassContext(
            robotPoseBlue,
            robotPos,
            cap,
            usePts,
            robotInCenterBand,
            dynAll,
            dynUse,
            lockHalf,
            sensedHalf,
            nowNs,
            dt,
            midX,
            leftBandX0,
            leftBandX1,
            rightBandX0,
            rightBandX1,
            clampToFieldRobotSafe,
            inForbidden,
            violatesWall,
            nudgeOutOfForbidden,
            safePushedFromRobot,
            holdPose,
            nearbyFuelCount,
            nearbyCentroid);
    return new FieldTrackerCollectPassSetupResult(context, null);
  }
}
