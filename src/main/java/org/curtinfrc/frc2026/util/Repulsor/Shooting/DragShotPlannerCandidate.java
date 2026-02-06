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
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

final class DragShotPlannerCandidate {
  final Translation2d shooterPosition;
  final double shooterYawRad;
  final double speed;
  final double angleRad;
  final double timeToPlane;
  final double verticalError;
  final double robotDistanceSq;

  DragShotPlannerCandidate(
      Translation2d shooterPosition,
      double shooterYawRad,
      double speed,
      double angleRad,
      double timeToPlane,
      double verticalError,
      double robotDistanceSq) {
    this.shooterPosition = shooterPosition;
    this.shooterYawRad = shooterYawRad;
    this.speed = speed;
    this.angleRad = angleRad;
    this.timeToPlane = timeToPlane;
    this.verticalError = verticalError;
    this.robotDistanceSq = robotDistanceSq;
  }

  static boolean isBetterCandidate(
      DragShotPlannerCandidate best, DragShotPlannerCandidate next, Constraints.ShotStyle style) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.isBetterCandidate");
    try {
      if (best == null) {
        return true;
      }

      if (next.robotDistanceSq < best.robotDistanceSq - 1e-9) {
        return true;
      }
      if (best.robotDistanceSq < next.robotDistanceSq - 1e-9) {
        return false;
      }

      if (style == Constraints.ShotStyle.DIRECT || style == Constraints.ShotStyle.ARC) {
        double angleBest = Math.abs(best.angleRad);
        double angleNext = Math.abs(next.angleRad);
        double angleEps = 1e-3;
        if (style == Constraints.ShotStyle.DIRECT) {
          if (angleNext < angleBest - angleEps) {
            return true;
          }
          if (angleBest < angleNext - angleEps) {
            return false;
          }
        } else {
          if (angleNext > angleBest + angleEps) {
            return true;
          }
          if (angleBest > angleNext + angleEps) {
            return false;
          }
        }
      }

      if (next.speed < best.speed - DragShotPlannerConstants.EPS) {
        return true;
      }
      if (best.speed < next.speed - DragShotPlannerConstants.EPS) {
        return false;
      }

      return next.verticalError < best.verticalError - DragShotPlannerConstants.EPS;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
