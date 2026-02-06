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
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerObstacles {
  private static volatile List<FieldPlanner.Obstacle> STATIC_OBSTACLES;

  private DragShotPlannerObstacles() {}

  static boolean isShooterPoseValid(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.isShooterPoseValid");
    try {
      return isShooterPoseValidInternal(
          shooterPos,
          targetFieldPosition,
          robotHalfLengthMeters,
          robotHalfWidthMeters,
          dynamicObstacles);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  static boolean isShooterPoseValidInternal(
      Translation2d shooterPos,
      Translation2d targetFieldPosition,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {

    AutoCloseable _p = Profiler.section("DragShotPlanner.isShooterPoseValidInternal.body");
    try {
      double x = shooterPos.getX();
      double y = shooterPos.getY();
      if (!Double.isFinite(x) || !Double.isFinite(y)) {
        return false;
      }

      final double SHOOT_X_END_BAND_M = 13.49;
      double minBand = SHOOT_X_END_BAND_M;
      double maxBand = Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M;
      if (x < minBand && x > maxBand) {
        return false;
      }

      double dx = targetFieldPosition.getX() - x;
      double dy = targetFieldPosition.getY() - y;
      Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(dy, dx));
      Translation2d[] rect =
          FieldPlanner.robotRect(shooterPos, yaw, robotHalfLengthMeters, robotHalfWidthMeters);

      for (FieldPlanner.Obstacle sObs : staticObstacles()) {
        if (sObs.intersectsRectangle(rect)) {
          return false;
        }
      }

      if (dynamicObstacles != null && !dynamicObstacles.isEmpty()) {
        for (FieldPlanner.Obstacle d : dynamicObstacles) {
          if (d.intersectsRectangle(rect)) {
            return false;
          }
        }
      }

      if (x < 0.0 || x > Constants.FIELD_LENGTH) {
        return false;
      }
      if (y < 0.0 || y > Constants.FIELD_WIDTH) {
        return false;
      }

      return true;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static List<FieldPlanner.Obstacle> staticObstacles() {
    AutoCloseable _p = Profiler.section("DragShotPlanner.staticObstacles");
    try {
      List<FieldPlanner.Obstacle> v = STATIC_OBSTACLES;
      if (v != null) {
        return v;
      }
      ArrayList<FieldPlanner.Obstacle> out = new ArrayList<>(64);
      try {
        out.addAll(Constants.FIELD.walls());
      } catch (Throwable ignored) {
      }
      try {
        out.addAll(Constants.FIELD.fieldObstacles());
      } catch (Throwable ignored) {
      }
      STATIC_OBSTACLES = List.copyOf(out);
      Profiler.gaugeSet("DragShotPlanner.staticObstacles.size", STATIC_OBSTACLES.size());
      return STATIC_OBSTACLES;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
