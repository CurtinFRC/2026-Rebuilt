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
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerStaticPositionSearch {
  private DragShotPlannerStaticPositionSearch() {}

  static Optional<ShotSolution> calculateShotAngleAndSpeedFromStaticPosition(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      Constraints constraints) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.calculateStaticShotAngleAndSpeed");
    try {
      if (constraints == null) {
        Profiler.counterAdd("DragShotPlanner.static.constraints_null", 1);
        return Optional.empty();
      }

      double minSpeed = constraints.minLaunchSpeedMetersPerSecond();
      double maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
      double minAngleDeg = constraints.minLaunchAngleDeg();
      double maxAngleDeg = constraints.maxLaunchAngleDeg();
      Constraints.ShotStyle shotStyle = constraints.shotStyle();
      boolean fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

      if (minSpeed <= 0.0 || maxSpeed <= minSpeed) {
        Profiler.counterAdd("DragShotPlanner.static.bad_speed_range", 1);
        return Optional.empty();
      }
      if (!fixedAngle && maxAngleDeg <= minAngleDeg) {
        Profiler.counterAdd("DragShotPlanner.static.bad_angle_range", 1);
        return Optional.empty();
      }

      double acceptableError = DragShotPlannerConstants.ACCEPTABLE_VERTICAL_ERROR_METERS;
      ShotSolution coarse =
          DragShotPlannerSolveAtPosition.solveBestAtShooterPosition(
              gamePiece,
              shooterFieldPosition,
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
              0.35,
              0.6);
      if (coarse == null) {
        return Optional.empty();
      }

      double coarseErrorAbs = Math.abs(coarse.verticalErrorMeters());
      if (coarseErrorAbs <= acceptableError) {
        return Optional.of(coarse);
      }

      ShotSolution refined =
          DragShotPlannerRefineAtPosition.refineShotAtPosition(
              gamePiece,
              shooterFieldPosition,
              targetFieldPosition,
              targetHeightMeters,
              shooterReleaseHeightMeters,
              minSpeed,
              maxSpeed,
              minAngleDeg,
              maxAngleDeg,
              fixedAngle,
              acceptableError,
              coarse.launchSpeedMetersPerSecond(),
              coarse.launchAngle().getRadians());
      if (refined != null) {
        return Optional.of(refined);
      }

      return Optional.empty();
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
