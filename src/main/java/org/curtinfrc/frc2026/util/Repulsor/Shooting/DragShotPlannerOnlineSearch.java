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
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerOnlineSearch {
  private static final double[][] ONLINE_OFFS =
      new double[][] {
        {0.0, 0.0},
        {1.0, 0.0},
        {-1.0, 0.0},
        {0.0, 1.0},
        {0.0, -1.0},
        {0.7071067811865476, 0.7071067811865476},
        {0.7071067811865476, -0.7071067811865476},
        {-0.7071067811865476, 0.7071067811865476},
        {-0.7071067811865476, -0.7071067811865476}
      };
  private static final double[][] ONLINE_OFFS_FAST =
      new double[][] {
        {0.0, 0.0},
        {1.0, 0.0},
        {-1.0, 0.0},
        {0.0, 1.0},
        {0.0, -1.0}
      };

  private DragShotPlannerOnlineSearch() {}

  static Optional<ShotSolution> findBestShotOnlineRefine(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints,
      OnlineSearchState state,
      long budgetNanos) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotOnlineRefine");
    try {
      Profiler.ensureInit();

      if (gamePiece == null
          || targetFieldPosition == null
          || robotPose == null
          || constraints == null
          || state == null) {
        Profiler.counterAdd("DragShotPlanner.online.bad_inputs", 1);
        return Optional.empty();
      }

      double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      double minAngleDeg = constraints.minLaunchAngleDeg();
      double maxAngleDeg = constraints.maxLaunchAngleDeg();
      boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;
      Constraints.ShotStyle shotStyle = constraints.shotStyle();
      double maxTravelSq = DragShotPlannerConstants.MAX_ROBOT_TRAVEL_METERS_SQ;
      double acceptableError = DragShotPlannerConstants.FAST_ACCEPTABLE_VERTICAL_ERROR_METERS;
      boolean fastMode =
          acceptableError >= DragShotPlannerConstants.FAST_ACCEPTABLE_VERTICAL_ERROR_METERS - 1e-9;

      if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
        Profiler.counterAdd("DragShotPlanner.online.bad_speed_range", 1);
        return Optional.empty();
      }
      if (!fixedAngle && maxAngleDeg <= minAngleDeg) {
        Profiler.counterAdd("DragShotPlanner.online.bad_angle_range", 1);
        return Optional.empty();
      }

      Translation2d robotPos = robotPose.getTranslation();
      double robotX = robotPos.getX();
      double robotY = robotPos.getY();
      double targetX = targetFieldPosition.getX();
      double targetY = targetFieldPosition.getY();
      int robotXmm = (int) Math.round(robotX * 1000.0);
      int robotYmm = (int) Math.round(robotY * 1000.0);
      int targetXmm = (int) Math.round(targetX * 1000.0);
      int targetYmm = (int) Math.round(targetY * 1000.0);
      int obsHash = obstaclesStableHash(dynamicObstacles);

      ShotSolution cached = state.lastSolution();
      if (cached != null) {
        int dtx = Math.abs(targetXmm - state.lastTargetXmm());
        int dty = Math.abs(targetYmm - state.lastTargetYmm());
        if (dtx <= 20 && dty <= 20 && obsHash == state.lastObsHash()) {
          Translation2d sp = cached.shooterPosition();
          double dxr = robotX - sp.getX();
          double dyr = robotY - sp.getY();
          double distSq = dxr * dxr + dyr * dyr;
          if (distSq <= maxTravelSq + 1e-6) {
            double err = cached.verticalErrorMeters();
            if (err < 0.0) err = -err;
            if (err <= acceptableError) {
              boolean ok;
              AutoCloseable _pCached =
                  Profiler.section("DragShotPlanner.isShooterPoseValid.cached");
              try {
                ok =
                    DragShotPlannerObstacles.isShooterPoseValidInternal(
                        sp,
                        targetFieldPosition,
                        robotHalfLengthMeters,
                        robotHalfWidthMeters,
                        dynamicObstacles,
                        true);
              } finally {
                DragShotPlannerUtil.closeQuietly(_pCached);
              }
              if (ok) {
                state.setLastSolution(cached, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
                return Optional.of(cached);
              }
            }
          }
        }
      }

      Translation2d seed = state.seed();
      if (seed == null) {
        seed = projectToValidRing(robotX, robotY, targetX, targetY);
        state.seed(seed);
      }

      double speedStep = Math.max(0.35, (maxSpeed - minSpeed) / 55.0);
      double angleStep = fixedAngle ? 1.0 : Math.max(0.65, (maxAngleDeg - minAngleDeg) / 55.0);
      if (fastMode) {
        speedStep = Math.max(speedStep, 0.8);
        if (!fixedAngle) {
          angleStep = Math.max(angleStep, 1.2);
        }
      }

      DragShotPlannerCandidate best = null;

      ShotSolution seedSol;
      AutoCloseable _p1 = Profiler.section("DragShotPlanner.solveBestAtShooterPosition.seed");
      try {
        seedSol =
            DragShotPlannerSolveAtPosition.solveBestAtShooterPosition(
                gamePiece,
                seed,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                acceptableError,
                shotStyle,
                speedStep,
                angleStep);
      } finally {
        DragShotPlannerUtil.closeQuietly(_p1);
      }

      if (seedSol != null) {
        boolean ok;
        AutoCloseable _p2 = Profiler.section("DragShotPlanner.isShooterPoseValid.seed");
        try {
          ok =
              DragShotPlannerObstacles.isShooterPoseValidInternal(
                  seed,
                  targetFieldPosition,
                  robotHalfLengthMeters,
                  robotHalfWidthMeters,
                  dynamicObstacles,
                  true);
        } finally {
          DragShotPlannerUtil.closeQuietly(_p2);
        }
        if (ok) {
          double dx = robotX - seedSol.shooterPosition().getX();
          double dy = robotY - seedSol.shooterPosition().getY();
          double distSq = dx * dx + dy * dy;
          if (distSq <= maxTravelSq + 1e-6) {
            double errSeed = seedSol.verticalErrorMeters();
            if (errSeed < 0.0) {
              errSeed = -errSeed;
            }
            if (errSeed <= acceptableError) {
              ShotSolution out =
                  new ShotSolution(
                      seedSol.shooterPosition(),
                      seedSol.shooterYaw(),
                      seedSol.launchSpeedMetersPerSecond(),
                      seedSol.launchAngle(),
                      seedSol.timeToPlaneSeconds(),
                      targetFieldPosition,
                      errSeed);
              state.setLastSolution(out, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
              return Optional.of(out);
            }
            best =
                new DragShotPlannerCandidate(
                    seedSol.shooterPosition(),
                    seedSol.shooterYaw().getRadians(),
                    seedSol.launchSpeedMetersPerSecond(),
                    seedSol.launchAngle().getRadians(),
                    seedSol.timeToPlaneSeconds(),
                    errSeed,
                    distSq);
          }
        }
      }

      long start = System.nanoTime();
      double step = Math.max(0.11, Math.min(0.95, state.stepMeters()));
      Translation2d current = seed;

      int outer = 0;
      int inner = 0;
      int rejectedTravel = 0;
      int rejectedPose = 0;
      int rejectedNoSol = 0;
      int improvedCount = 0;

      double[][] offsets = fastMode ? ONLINE_OFFS_FAST : ONLINE_OFFS;
      while ((System.nanoTime() - start) < budgetNanos) {
        outer++;
        boolean improved = false;

        double cx = current.getX();
        double cy = current.getY();

        for (double[] o : offsets) {
          inner++;
          double px = cx + o[0] * step;
          double py = cy + o[1] * step;
          Translation2d clipped = clipToRingAndField(px, py, targetX, targetY);

          double clippedX = clipped.getX();
          double clippedY = clipped.getY();
          double dxr = robotX - clippedX;
          double dyr = robotY - clippedY;
          double distSq = dxr * dxr + dyr * dyr;
          if (distSq > maxTravelSq) {
            rejectedTravel++;
            continue;
          }

          boolean ok;
          AutoCloseable _p3 = Profiler.section("DragShotPlanner.isShooterPoseValid.online");
          try {
            ok =
                DragShotPlannerObstacles.isShooterPoseValidInternal(
                    clipped,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles,
                    true);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p3);
          }
          if (!ok) {
            rejectedPose++;
            continue;
          }

          ShotSolution sol;
          AutoCloseable _p4 = Profiler.section("DragShotPlanner.solveBestAtShooterPosition.online");
          try {
            sol =
                DragShotPlannerSolveAtPosition.solveBestAtShooterPosition(
                    gamePiece,
                    clipped,
                    targetFieldPosition,
                    targetHeightMeters,
                    shooterReleaseHeightMeters,
                    minSpeed,
                    maxSpeed,
                    minAngleDeg,
                    maxAngleDeg,
                    fixedAngle,
                    acceptableError,
                    shotStyle,
                    speedStep,
                    angleStep);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p4);
          }

          if (sol == null) {
            rejectedNoSol++;
            continue;
          }

          double dx2 = robotX - sol.shooterPosition().getX();
          double dy2 = robotY - sol.shooterPosition().getY();
          double distSq2 = dx2 * dx2 + dy2 * dy2;

          double errSol = sol.verticalErrorMeters();
          if (errSol < 0.0) {
            errSol = -errSol;
          }
          DragShotPlannerCandidate cand =
              new DragShotPlannerCandidate(
                  sol.shooterPosition(),
                  sol.shooterYaw().getRadians(),
                  sol.launchSpeedMetersPerSecond(),
                  sol.launchAngle().getRadians(),
                  sol.timeToPlaneSeconds(),
                  errSol,
                  distSq2);

          if (errSol <= acceptableError) {
            ShotSolution out =
                new ShotSolution(
                    cand.shooterPosition,
                    Rotation2d.fromRadians(cand.shooterYawRad),
                    cand.speed,
                    Rotation2d.fromRadians(cand.angleRad),
                    cand.timeToPlane,
                    targetFieldPosition,
                    cand.verticalError);
            state.setLastSolution(out, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
            return Optional.of(out);
          }

          if (DragShotPlannerCandidate.isBetterCandidate(best, cand, shotStyle)) {
            best = cand;
            current = sol.shooterPosition();
            improved = true;
            improvedCount++;
          }
        }

        if (!improved) {
          step *= fastMode ? 0.5 : 0.58;
          if (step < (fastMode ? 0.12 : 0.075)) {
            break;
          }
        }
      }

      Profiler.counterAdd("DragShotPlanner.online.outer", outer);
      Profiler.counterAdd("DragShotPlanner.online.inner", inner);
      Profiler.counterAdd("DragShotPlanner.online.rejected_travel", rejectedTravel);
      Profiler.counterAdd("DragShotPlanner.online.rejected_pose", rejectedPose);
      Profiler.counterAdd("DragShotPlanner.online.rejected_no_solution", rejectedNoSol);
      Profiler.counterAdd("DragShotPlanner.online.improved", improvedCount);
      Profiler.gaugeSet("DragShotPlanner.online.final_step_mm", (long) (step * 1000.0));

      if (best == null) {
        Profiler.counterAdd("DragShotPlanner.online.no_best", 1);
        return Optional.empty();
      }

      if (best.verticalError <= acceptableError) {
        ShotSolution out =
            new ShotSolution(
                best.shooterPosition,
                Rotation2d.fromRadians(best.shooterYawRad),
                best.speed,
                Rotation2d.fromRadians(best.angleRad),
                best.timeToPlane,
                targetFieldPosition,
                best.verticalError);
        state.setLastSolution(out, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
        return Optional.of(out);
      }

      state.seed(best.shooterPosition);
      state.stepMeters(Math.max(0.10, Math.min(0.80, step)));
      state.touch();

      ShotSolution refined;
      AutoCloseable _p5 = Profiler.section("DragShotPlanner.refineShotAtPosition.online");
      try {
        refined =
            DragShotPlannerRefineAtPosition.refineShotAtPosition(
                gamePiece,
                best.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                acceptableError,
                best.speed,
                best.angleRad);
      } finally {
        DragShotPlannerUtil.closeQuietly(_p5);
      }

      if (refined != null) {
        state.setLastSolution(refined, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
        return Optional.of(refined);
      }

      ShotSolution out =
          new ShotSolution(
              best.shooterPosition,
              Rotation2d.fromRadians(best.shooterYawRad),
              best.speed,
              Rotation2d.fromRadians(best.angleRad),
              best.timeToPlane,
              targetFieldPosition,
              best.verticalError);
      state.setLastSolution(out, robotXmm, robotYmm, targetXmm, targetYmm, obsHash);
      return Optional.of(out);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static Translation2d projectToValidRing(
      double px, double py, double targetX, double targetY) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.projectToValidRing");
    try {
      double dx = px - targetX;
      double dy = py - targetY;
      double r2 = dx * dx + dy * dy;
      if (r2 < 1e-12) {
        return new Translation2d(targetX - 3.0, targetY);
      }
      double r = Math.sqrt(r2);
      double clamped =
          Math.max(
              DragShotPlannerConstants.MIN_RANGE_METERS,
              Math.min(DragShotPlannerConstants.MAX_RANGE_METERS, r));
      double scale = clamped / r;
      return new Translation2d(targetX + dx * scale, targetY + dy * scale);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static Translation2d clipToRingAndField(
      double px, double py, double targetX, double targetY) {
    AutoCloseable _p0 = Profiler.section("DragShotPlanner.clipToRingAndField");
    try {
      Translation2d ring = projectToValidRing(px, py, targetX, targetY);
      double x = ring.getX();
      double y = ring.getY();
      if (!Double.isFinite(x) || !Double.isFinite(y)) {
        return new Translation2d(targetX - 3.0, targetY);
      }
      return ring;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p0);
    }
  }

  private static int obstaclesStableHash(List<? extends Obstacle> obs) {
    if (obs == null || obs.isEmpty()) return 0;
    return (System.identityHashCode(obs) * 31) ^ obs.size();
  }
}
