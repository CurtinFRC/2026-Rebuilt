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
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerLibrarySearch {
  private DragShotPlannerLibrarySearch() {}

  static Optional<ShotSolution> findBestShotFromLibrary(
      ShotLibrary library,
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Constraints constraints) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotFromLibrary");
    try {
      Profiler.ensureInit();

      if (library == null || library.entries().isEmpty()) {
        Profiler.counterAdd("DragShotPlanner.library.empty", 1);
        return Optional.empty();
      }

      Profiler.gaugeSet("DragShotPlanner.library.entries", library.entries().size());

      Translation2d robotCurrentPosition = robotPose.getTranslation();
      DragShotPlannerCandidate best = null;

      boolean needDynamicCheck = dynamicObstacles != null && !dynamicObstacles.isEmpty();
      if (needDynamicCheck) {
        Profiler.gaugeSet("DragShotPlanner.dynamicObstacles.count", dynamicObstacles.size());
      }

      double rx = robotCurrentPosition.getX();
      double ry = robotCurrentPosition.getY();
      Constraints.ShotStyle shotStyle = constraints.shotStyle();
      double maxTravelSq = DragShotPlannerConstants.MAX_ROBOT_TRAVEL_METERS_SQ;
      double acceptableError = DragShotPlannerConstants.FAST_ACCEPTABLE_VERTICAL_ERROR_METERS;
      double nearEnoughSq = 0.25;

      int iter = 0;
      int rejectedTravel = 0;
      int rejectedDyn = 0;
      int accepted = 0;

      for (ShotLibraryEntry e : library.entries()) {
        iter++;
        Translation2d shooterPos = e.shooterPosition();

        double dx = rx - shooterPos.getX();
        double dy = ry - shooterPos.getY();
        double robotDistanceSq = dx * dx + dy * dy;
        if (robotDistanceSq > maxTravelSq) {
          rejectedTravel++;
          continue;
        }

        if (needDynamicCheck) {
          boolean okDyn;
          AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.dynamic");
          try {
            okDyn =
                DragShotPlannerObstacles.isShooterPoseValidInternal(
                    shooterPos,
                    targetFieldPosition,
                    robotHalfLengthMeters,
                    robotHalfWidthMeters,
                    dynamicObstacles);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p1);
          }
          if (!okDyn) {
            rejectedDyn++;
            continue;
          }
        }

        double err = e.verticalErrorMeters();
        if (err < 0.0) {
          err = -err;
        }
        DragShotPlannerCandidate next =
            new DragShotPlannerCandidate(
                shooterPos,
                e.shooterYawRad(),
                e.launchSpeedMetersPerSecond(),
                e.launchAngleRad(),
                e.timeToPlaneSeconds(),
                err,
                robotDistanceSq);

        if (DragShotPlannerCandidate.isBetterCandidate(best, next, shotStyle)) {
          best = next;
        }
        accepted++;

        if (best.verticalError <= acceptableError && best.robotDistanceSq <= nearEnoughSq) {
          break;
        }
      }

      Profiler.counterAdd("DragShotPlanner.library.loop_iter", iter);
      Profiler.counterAdd("DragShotPlanner.library.rejected_travel", rejectedTravel);
      Profiler.counterAdd("DragShotPlanner.library.rejected_dynamic", rejectedDyn);
      Profiler.counterAdd("DragShotPlanner.library.accepted", accepted);

      if (best == null) {
        Profiler.counterAdd("DragShotPlanner.library.no_best", 1);
        return Optional.empty();
      }

      if (best.verticalError <= acceptableError) {
        return Optional.of(
            new ShotSolution(
                best.shooterPosition,
                Rotation2d.fromRadians(best.shooterYawRad),
                best.speed,
                Rotation2d.fromRadians(best.angleRad),
                best.timeToPlane,
                targetFieldPosition,
                best.verticalError));
      }

      ShotSolution refined;
      AutoCloseable _p2 = Profiler.section("DragShotPlanner.refineShotAtPosition.library");
      try {
        refined =
            DragShotPlannerRefineAtPosition.refineShotAtPosition(
                gamePiece,
                best.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                constraints.minLaunchSpeedMetersPerSecond(),
                constraints.maxLaunchSpeedMetersPerSecond(),
                constraints.minLaunchAngleDeg(),
                constraints.maxLaunchAngleDeg(),
                Math.abs(constraints.maxLaunchAngleDeg() - constraints.minLaunchAngleDeg()) < 1e-6,
                acceptableError,
                best.speed,
                best.angleRad);
      } finally {
        DragShotPlannerUtil.closeQuietly(_p2);
      }

      if (refined != null) {
        return Optional.of(refined);
      }

      return Optional.of(
          new ShotSolution(
              best.shooterPosition,
              Rotation2d.fromRadians(best.shooterYawRad),
              best.speed,
              Rotation2d.fromRadians(best.angleRad),
              best.timeToPlane,
              targetFieldPosition,
              best.verticalError));
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
