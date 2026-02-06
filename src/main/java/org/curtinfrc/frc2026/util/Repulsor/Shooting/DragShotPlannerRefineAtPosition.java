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
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerRefineAtPosition {
  private DragShotPlannerRefineAtPosition() {}

  static ShotSolution refineShotAtPosition(
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

      Rotation2d shooterYaw = Rotation2d.fromRadians(Math.atan2(dyT, dxT));

      double speedWindow = Math.max(1.5, (maxSpeed - minSpeed) / 8.0);
      double speedMin = Math.max(minSpeed, coarseSpeed - speedWindow);
      double speedMax = Math.min(maxSpeed, coarseSpeed + speedWindow);
      double speedStepFine = Math.max(0.11, (speedMax - speedMin) / 18.0);
      double acceptableError = acceptableVerticalErrorMeters;

      double angleStartDeg;
      double angleEndDeg;
      double angleStepFineDeg;
      double radToDeg = DragShotPlannerConstants.RAD_TO_DEG;
      double degToRad = DragShotPlannerConstants.DEG_TO_RAD;
      double coarseAngleDeg = coarseAngleRad * radToDeg;

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

      DragShotPlannerSimulation.SimOut sim = DragShotPlannerSimulation.simOut();

      int angleCap = (int) Math.ceil((angleEndDeg - angleStartDeg) / angleStepFineDeg) + 1;
      double[] angleRad = new double[angleCap];
      double[] angleCos = new double[angleCap];
      double[] angleSin = new double[angleCap];
      int angleCount = 0;
      for (double ang = angleStartDeg; ang <= angleEndDeg + 1e-6; ang += angleStepFineDeg) {
        double rad = ang * degToRad;
        angleRad[angleCount] = rad;
        angleCos[angleCount] = Math.cos(rad);
        angleSin[angleCount] = Math.sin(rad);
        angleCount++;
      }

      for (int ai = 0; ai < angleCount; ai++) {
        double cos = angleCos[ai];
        if (cos <= 0.0) {
          continue;
        }
        double sin = angleSin[ai];
        double angleRad = angleRad[ai];

        for (double speed = speedMin; speed <= speedMax + 1e-6; speed += speedStepFine) {
          sims++;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.simulateToTargetPlane.refine");
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
            DragShotPlannerUtil.closeQuietly(_p1);
          }

          if (!sim.hitPlane) {
            continue;
          }
          simsHit++;

          double error = sim.verticalErrorMeters;
          if (error < 0.0) {
            error = -error;
          }
          if (error > acceptableError) {
            continue;
          }
          simsWithin++;

          if (best == null
              || error < bestError - DragShotPlannerConstants.EPS
              || (Math.abs(error - bestError) <= DragShotPlannerConstants.EPS
                  && speed < best.launchSpeedMetersPerSecond() - DragShotPlannerConstants.EPS)) {
            bestError = error;
            best =
                new ShotSolution(
                    shooterFieldPosition,
                    shooterYaw,
                    speed,
                    Rotation2d.fromRadians(angleRad),
                    sim.timeAtPlaneSeconds,
                    targetFieldPosition,
                    sim.verticalErrorMeters);
          }
        }
      }

      Profiler.counterAdd("DragShotPlanner.refine.sims", sims);
      Profiler.counterAdd("DragShotPlanner.refine.sims_hitplane", simsHit);
      Profiler.counterAdd("DragShotPlanner.refine.sims_within", simsWithin);

      return best;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
