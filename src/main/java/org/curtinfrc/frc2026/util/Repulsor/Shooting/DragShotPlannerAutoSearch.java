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
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerAutoSearch {
  private DragShotPlannerAutoSearch() {}

  static Optional<ShotSolution> findBestShotAuto(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      edu.wpi.first.math.geometry.Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Constraints constraints) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.findBestShotAuto");
    try {
      Profiler.ensureInit();

      double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      double minAngleDeg = constraints.minLaunchAngleDeg();
      double maxAngleDeg = constraints.maxLaunchAngleDeg();
      Constraints.ShotStyle shotStyle = constraints.shotStyle();

      if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
        Profiler.counterAdd("DragShotPlanner.auto.bad_speed_range", 1);
        return Optional.empty();
      }
      if (!(Math.abs(maxAngleDeg - minAngleDeg) < 1e-6) && maxAngleDeg <= minAngleDeg) {
        Profiler.counterAdd("DragShotPlanner.auto.bad_angle_range", 1);
        return Optional.empty();
      }

      Translation2d robotCurrentPosition = robotPose.getTranslation();
      boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

      DragShotPlannerCandidate coarse;
      AutoCloseable _p1 = Profiler.section("DragShotPlanner.coarseSearch");
      try {
        coarse =
            DragShotPlannerCoarseSearch.coarseSearch(
                gamePiece,
                targetFieldPosition,
                targetHeightMeters,
                robotCurrentPosition,
                shooterReleaseHeightMeters,
                robotHalfLengthMeters,
                robotHalfWidthMeters,
                dynamicObstacles,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                shotStyle);
      } finally {
        DragShotPlannerUtil.closeQuietly(_p1);
      }

      if (coarse == null) {
        Profiler.counterAdd("DragShotPlanner.auto.coarse_none", 1);
        return Optional.empty();
      }

      ShotSolution refined;
      AutoCloseable _p2 = Profiler.section("DragShotPlanner.refineShotAtPosition.auto");
      try {
        refined =
            DragShotPlannerRefineAtPosition.refineShotAtPosition(
                gamePiece,
                coarse.shooterPosition,
                targetFieldPosition,
                targetHeightMeters,
                shooterReleaseHeightMeters,
                minSpeed,
                maxSpeed,
                minAngleDeg,
                maxAngleDeg,
                fixedAngle,
                DragShotPlannerConstants.ACCEPTABLE_VERTICAL_ERROR_METERS,
                coarse.speed,
                coarse.angleRad);
      } finally {
        DragShotPlannerUtil.closeQuietly(_p2);
      }

      if (refined == null) {
        Profiler.counterAdd("DragShotPlanner.auto.refine_none", 1);
        return Optional.empty();
      }

      return Optional.of(refined);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
