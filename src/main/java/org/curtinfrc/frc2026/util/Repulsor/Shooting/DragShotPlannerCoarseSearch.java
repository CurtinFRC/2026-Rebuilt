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

import edu.wpi.first.math.geometry.Rotation2d;
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

      DragShotPlannerCandidate bestCoarse = null;

      double rx = robotCurrentPosition.getX();
      double ry = robotCurrentPosition.getY();

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
        for (double bearingDeg = 0.0; bearingDeg < 360.0; bearingDeg += bearingStepDegCoarse) {
          bearings++;
          Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
          Translation2d shooterPos = targetFieldPosition.minus(new Translation2d(range, bearing));

          double dx = rx - shooterPos.getX();
          double dy = ry - shooterPos.getY();
          double robotDistanceSq = dx * dx + dy * dy;
          if (robotDistanceSq > DragShotPlannerConstants.MAX_ROBOT_TRAVEL_METERS_SQ) {
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

              double error = Math.abs(sim.verticalErrorMeters);
              if (error > coarseTolerance) {
                continue;
              }
              simsAccepted++;

              DragShotPlannerCandidate next =
                  new DragShotPlannerCandidate(
                      shooterPos,
                      shooterYaw.getRadians(),
                      speed,
                      angleRad,
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
