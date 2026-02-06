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

final class DragShotPlannerSolveAtPosition {
  private DragShotPlannerSolveAtPosition() {}

  static ShotSolution solveBestAtShooterPosition(
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
          DragShotPlannerCache.solveKey(
              shooterFieldPosition,
              targetFieldPosition,
              targetHeightMeters,
              shooterReleaseHeightMeters,
              minSpeed,
              maxSpeed,
              minAngleDeg,
              maxAngleDeg,
              shotStyle);

      ShotSolution cached = DragShotPlannerCache.get(ck);
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

      Rotation2d shooterYaw = Rotation2d.fromRadians(Math.atan2(dyT, dxT));

      double angleStep = fixedAngle ? 1.0 : Math.max(0.18, angleStepDeg);
      boolean fastMode =
          acceptableError >= DragShotPlannerConstants.FAST_ACCEPTABLE_VERTICAL_ERROR_METERS - 1e-9;
      if (!fixedAngle && fastMode) {
        angleStep = Math.max(angleStep, 0.5);
      }
      double degToRad = DragShotPlannerConstants.DEG_TO_RAD;
      double acceptableError = acceptableVerticalErrorMeters;
      double earlyAccept = acceptableError * 0.35;
      double refineBreak = acceptableError * 0.18;

      double bestErrorAbs = Double.POSITIVE_INFINITY;
      double bestAngleRad = 0.0;
      double bestSpeed = 0.0;
      double bestTime = 0.0;
      double bestSignedErr = 0.0;
      boolean found = false;

      int sims = 0;
      int simsHit = 0;
      int simsWithin = 0;

      DragShotPlannerSimulation.SimOut sim = DragShotPlannerSimulation.simOut();

      double speedRange = maxSpeed - minSpeed;
      double coarseStep = Math.max(Math.max(0.9, speedStep * 3.0), speedRange / 10.0);
      if (fastMode) {
        coarseStep = Math.max(coarseStep, 1.2);
      }

      int angleCap = (int) Math.ceil((maxAngleDeg - minAngleDeg) / angleStep) + 1;
      double[] angleRad = new double[angleCap];
      double[] angleCos = new double[angleCap];
      double[] angleSin = new double[angleCap];
      int angleCount = 0;
      for (double ang = minAngleDeg; ang <= maxAngleDeg + 1e-6; ang += angleStep) {
        double rad = ang * degToRad;
        angleRad[angleCount] = rad;
        angleCos[angleCount] = Math.cos(rad);
        angleSin[angleCount] = Math.sin(rad);
        angleCount++;
      }

      for (int ai = 0; ai < angleCount; ai++) {
        double cos = angleCos[ai];
        if (cos <= 0.0) continue;
        double sin = angleSin[ai];
        double angleRad = angleRad[ai];

        double localBestSpeed = 0.0;
        double localBestErrAbs = Double.POSITIVE_INFINITY;
        double localBestTime = 0.0;
        double localBestSigned = 0.0;
        boolean localFound = false;

        for (double speed = minSpeed; speed <= maxSpeed + 1e-6; speed += coarseStep) {
          sims++;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.simulateToTargetPlane.solveAtPos");
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

          if (!sim.hitPlane) continue;
          simsHit++;

          double errAbs = sim.verticalErrorMeters;
          if (errAbs < 0.0) {
            errAbs = -errAbs;
          }
          if (errAbs <= acceptableError) simsWithin++;

          if (!localFound || errAbs < localBestErrAbs - 1e-9) {
            localFound = true;
            localBestSpeed = speed;
            localBestErrAbs = errAbs;
            localBestTime = sim.timeAtPlaneSeconds;
            localBestSigned = sim.verticalErrorMeters;
            if (localBestErrAbs <= earlyAccept) break;
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

        int refineIters = fastMode ? 6 : 9;
        for (int it = 0; it < refineIters; it++) {
          double m1 = lo + (hi - lo) * (1.0 / 3.0);
          double m2 = hi - (hi - lo) * (1.0 / 3.0);

          double e1Abs;
          double t1;
          double s1;
          sims++;
          AutoCloseable _p2 = Profiler.section("DragShotPlanner.simulateToTargetPlane.solveAtPos");
          try {
            DragShotPlannerSimulation.simulateToTargetPlaneInto(
                sim,
                gamePiece,
                m1 * cos,
                m1 * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p2);
          }
          if (sim.hitPlane) {
            simsHit++;
            e1Abs = sim.verticalErrorMeters;
            if (e1Abs < 0.0) {
              e1Abs = -e1Abs;
            }
            t1 = sim.timeAtPlaneSeconds;
            s1 = sim.verticalErrorMeters;
            if (e1Abs <= acceptableError) simsWithin++;
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
            DragShotPlannerSimulation.simulateToTargetPlaneInto(
                sim,
                gamePiece,
                m2 * cos,
                m2 * sin,
                shooterReleaseHeightMeters,
                horizontalDistance,
                targetHeightMeters);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p3);
          }
          if (sim.hitPlane) {
            simsHit++;
            e2Abs = sim.verticalErrorMeters;
            if (e2Abs < 0.0) {
              e2Abs = -e2Abs;
            }
            t2 = sim.timeAtPlaneSeconds;
            s2 = sim.verticalErrorMeters;
            if (e2Abs <= acceptableError) simsWithin++;
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

          if (refineBestErrAbs <= refineBreak) break;
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
                else take = refineBestSpeed < bestSpeed - DragShotPlannerConstants.EPS;
              } else {
                if (aNext > aBest + aEps) take = true;
                else if (aBest > aNext + aEps) take = false;
                else take = refineBestSpeed < bestSpeed - DragShotPlannerConstants.EPS;
              }
            } else {
              take = refineBestSpeed < bestSpeed - DragShotPlannerConstants.EPS;
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

      ShotSolution outSolution =
          new ShotSolution(
              shooterFieldPosition,
              shooterYaw,
              bestSpeed,
              Rotation2d.fromRadians(bestAngleRad),
              bestTime,
              targetFieldPosition,
              bestSignedErr);

      DragShotPlannerCache.put(ck, outSolution);
      return outSolution;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
