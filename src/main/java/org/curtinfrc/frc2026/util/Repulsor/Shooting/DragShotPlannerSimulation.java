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

import java.util.concurrent.ConcurrentHashMap;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerSimulation {
  private static final class SimParams {
    final double kOverM;

    SimParams(double kOverM) {
      this.kOverM = kOverM;
    }
  }

  static final class SimOut {
    boolean hitPlane;
    double timeAtPlaneSeconds;
    double verticalErrorMeters;

    void set(boolean hitPlane, double timeAtPlaneSeconds, double verticalErrorMeters) {
      this.hitPlane = hitPlane;
      this.timeAtPlaneSeconds = timeAtPlaneSeconds;
      this.verticalErrorMeters = verticalErrorMeters;
    }
  }

  private static final ThreadLocal<SimOut> SIM_OUT_TL =
      ThreadLocal.withInitial(
          () -> {
            SimOut o = new SimOut();
            o.set(false, 0.0, Double.POSITIVE_INFINITY);
            return o;
          });

  private static final ConcurrentHashMap<GamePiecePhysics, SimParams> SIM_PARAMS_CACHE =
      new ConcurrentHashMap<>();

  private DragShotPlannerSimulation() {}

  static SimOut simOut() {
    return SIM_OUT_TL.get();
  }

  static void simulateToTargetPlaneInto(
      SimOut out,
      GamePiecePhysics gamePiece,
      double vx0,
      double vy0,
      double shooterReleaseHeightMeters,
      double targetHorizontalDistanceMeters,
      double targetHeightMeters) {
    simulateToTargetPlaneIntoInternal(
        out,
        gamePiece,
        vx0,
        vy0,
        shooterReleaseHeightMeters,
        targetHorizontalDistanceMeters,
        targetHeightMeters,
        false);
  }

  static void simulateToTargetPlaneIntoFast(
      SimOut out,
      GamePiecePhysics gamePiece,
      double vx0,
      double vy0,
      double shooterReleaseHeightMeters,
      double targetHorizontalDistanceMeters,
      double targetHeightMeters) {
    simulateToTargetPlaneIntoInternal(
        out,
        gamePiece,
        vx0,
        vy0,
        shooterReleaseHeightMeters,
        targetHorizontalDistanceMeters,
        targetHeightMeters,
        true);
  }

  private static void simulateToTargetPlaneIntoInternal(
      SimOut out,
      GamePiecePhysics gamePiece,
      double vx0,
      double vy0,
      double shooterReleaseHeightMeters,
      double targetHorizontalDistanceMeters,
      double targetHeightMeters,
      boolean fastMode) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.simulateToTargetPlane.body");
    try {
      final double g = 9.81;

      double avx0 = vx0 >= 0.0 ? vx0 : -vx0;
      if (avx0 < 1e-6) {
        out.set(false, 0.0, Double.POSITIVE_INFINITY);
        return;
      }

      double minHorizClamp = fastMode ? 0.7 : 0.6;
      double maxTimeFactor = fastMode ? 1.25 : 1.45;
      double minTime = fastMode ? 0.35 : 0.45;
      double maxTime = fastMode ? 3.2 : 4.2;
      double dxDiv = fastMode ? 48.0 : 70.0;
      double dxMin = fastMode ? 0.07 : 0.05;
      double dxMax = fastMode ? 0.22 : 0.14;

      double minHorizontalSpeed = avx0 < minHorizClamp ? minHorizClamp : avx0;
      double timeNoDrag = targetHorizontalDistanceMeters / minHorizontalSpeed;
      double maxTimeSeconds = timeNoDrag * maxTimeFactor;
      if (maxTimeSeconds < minTime) maxTimeSeconds = minTime;
      if (maxTimeSeconds > maxTime) maxTimeSeconds = maxTime;

      double kOverM = simParams(gamePiece).kOverM;

      double x = 0.0;
      double y = shooterReleaseHeightMeters;
      double vx = vx0;
      double vy = vy0;
      double t = 0.0;

      double xPrev = 0.0;
      double yPrev = y;
      double tPrev = 0.0;

      double dxStep = targetHorizontalDistanceMeters / dxDiv;
      if (dxStep < dxMin) dxStep = dxMin;
      if (dxStep > dxMax) dxStep = dxMax;

      int steps = 0;

      while (t < maxTimeSeconds && y >= -0.25 && x <= targetHorizontalDistanceMeters + 0.9) {
        steps++;

        xPrev = x;
        yPrev = y;
        tPrev = t;

        double vv = vx * vx + vy * vy;
        if (vv < 1e-12) {
          out.set(false, 0.0, Double.POSITIVE_INFINITY);
          return;
        }

        double v = Math.sqrt(vv);
        double ax = -kOverM * v * vx;
        double ay = -g - kOverM * v * vy;

        double avx = vx >= 0.0 ? vx : -vx;
        double dt = dxStep / (avx < 0.35 ? 0.35 : avx);

        vx += ax * dt;
        vy += ay * dt;
        x += vx * dt;
        y += vy * dt;
        t += dt;

        if (xPrev <= targetHorizontalDistanceMeters && x >= targetHorizontalDistanceMeters) {
          double denom = (x - xPrev);
          double frac =
              (Math.abs(denom) > 1e-12) ? (targetHorizontalDistanceMeters - xPrev) / denom : 1.0;
          if (frac < 0.0) frac = 0.0;
          if (frac > 1.0) frac = 1.0;

          double yCross = yPrev + frac * (y - yPrev);
          double tCross = tPrev + frac * (t - tPrev);
          double verticalError = yCross - targetHeightMeters;

          Profiler.counterAdd("DragShotPlanner.sim.steps", steps);
          out.set(true, tCross, verticalError);
          return;
        }
      }

      Profiler.counterAdd("DragShotPlanner.sim.steps", steps);
      out.set(false, 0.0, Double.POSITIVE_INFINITY);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static SimParams simParams(GamePiecePhysics gp) {
    return SIM_PARAMS_CACHE.computeIfAbsent(
        gp,
        k -> {
          double m = gp.massKg();
          double A = gp.crossSectionAreaM2();
          double Cd = gp.dragCoefficient();
          double rho = gp.airDensityKgPerM3();
          double kOverM = 0.5 * rho * Cd * A / m;
          return new SimParams(kOverM);
        });
  }
}
