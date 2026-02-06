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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerCoarseSearch {
  private DragShotPlannerCoarseSearch() {}

  static DragShotPlannerCandidate coarseSearch(
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
      double coarseTolerance = DragShotPlannerConstants.ACCEPTABLE_VERTICAL_ERROR_METERS * 3.0;
      double maxTravelSq = DragShotPlannerConstants.MAX_ROBOT_TRAVEL_METERS_SQ;
      double degToRad = DragShotPlannerConstants.DEG_TO_RAD;

      int bearingCap = (int) Math.ceil(360.0 / bearingStepDegCoarse);
      double[] bearingRad = new double[bearingCap];
      double[] bearingCos = new double[bearingCap];
      double[] bearingSin = new double[bearingCap];
      int bearingCount = 0;
      for (double bearingDeg = 0.0; bearingDeg < 360.0 - 1e-9; bearingDeg += bearingStepDegCoarse) {
        double rad = bearingDeg * degToRad;
        bearingRad[bearingCount] = rad;
        bearingCos[bearingCount] = Math.cos(rad);
        bearingSin[bearingCount] = Math.sin(rad);
        bearingCount++;
      }

      double angleStartDeg = minAngleDeg;
      double angleEndDeg = maxAngleDeg;
      double angleStepDeg = fixedAngle ? 1.0 : angleStepCoarse;
      int angleCap = (int) Math.ceil((angleEndDeg - angleStartDeg) / angleStepDeg) + 1;
      double[] angleRad = new double[angleCap];
      double[] angleCos = new double[angleCap];
      double[] angleSin = new double[angleCap];
      int angleCount = 0;
      for (double ang = angleStartDeg; ang <= angleEndDeg + 1e-6; ang += angleStepDeg) {
        double rad = ang * degToRad;
        angleRad[angleCount] = rad;
        angleCos[angleCount] = Math.cos(rad);
        angleSin[angleCount] = Math.sin(rad);
        angleCount++;
      }

      DragShotPlannerCandidate bestCoarse = null;

      double rx = robotCurrentPosition.getX();
      double ry = robotCurrentPosition.getY();
      double targetX = targetFieldPosition.getX();
      double targetY = targetFieldPosition.getY();

      int ranges = 0;
      int bearings = 0;
      int posesChecked = 0;
      int posesRejected = 0;
      int sims = 0;
      int simsHit = 0;
      int simsAccepted = 0;

      DragShotPlannerSimulation.SimOut sim = DragShotPlannerSimulation.simOut();

      for (double range = DragShotPlannerConstants.MIN_RANGE_METERS;
          range <= DragShotPlannerConstants.MAX_RANGE_METERS + 1e-6;
          range += radialStepCoarse) {
        ranges++;
        for (int bi = 0; bi < bearingCount; bi++) {
          bearings++;
          double bearingAngleRad = bearingRad[bi];
          double cosB = bearingCos[bi];
          double sinB = bearingSin[bi];
          double shooterX = targetX - range * cosB;
          double shooterY = targetY - range * sinB;
          Translation2d shooterPos = new Translation2d(shooterX, shooterY);

          double dx = rx - shooterX;
          double dy = ry - shooterY;
          double robotDistanceSq = dx * dx + dy * dy;
          if (robotDistanceSq > maxTravelSq) {
            continue;
          }

          posesChecked++;
          boolean ok;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.coarse");
          try {
            ok =
                DragShotPlannerObstacles.isShooterPoseValidInternal(
                    shooterPos,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p1);
          }
          if (!ok) {
            posesRejected++;
            continue;
          }

          double horizontalDistance = range;
          double shooterYawRad = bearingAngleRad;

          for (int ai = 0; ai < angleCount; ai++) {
            double cos = angleCos[ai];
            if (cos <= 0.0) {
              continue;
            }
            double sin = angleSin[ai];
            double angleRadVal = angleRad[ai];

            for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += speedStepCoarse) {
              sims++;
              AutoCloseable _p2 = Profiler.section("DragShotPlanner.simulateToTargetPlane.coarse");
              try {
                DragShotPlannerSimulation.simulateToTargetPlaneInto(
                    sim,
                    gamePiece,
                    speed * cos,
                    speed * sin,
                    shooterReleaseHeightMeters,
                    horizontalDistance,
                    targetHeightMeters);
              } finally {
                DragShotPlannerUtil.closeQuietly(_p2);
              }

              if (!sim.hitPlane) {
                continue;
              }
              simsHit++;

              double error = sim.verticalErrorMeters;
              if (error < 0.0) {
                error = -error;
              }
              if (error > coarseTolerance) {
                continue;
              }
              simsAccepted++;

              DragShotPlannerCandidate next =
                  new DragShotPlannerCandidate(
                      shooterPos,
                      shooterYawRad,
                      speed,
                      angleRadVal,
                      sim.timeAtPlaneSeconds,
                      error,
                      robotDistanceSq);

              if (DragShotPlannerCandidate.isBetterCandidate(bestCoarse, next, shotStyle)) {
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
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
