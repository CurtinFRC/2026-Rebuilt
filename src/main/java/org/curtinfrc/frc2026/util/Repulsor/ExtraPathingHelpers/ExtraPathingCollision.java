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

package org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

public final class ExtraPathingCollision {
  private ExtraPathingCollision() {}

  public static Translation2d[] rectCorners(Translation2d center, double length, double width) {
    double hx = length * 0.5, hy = width * 0.5;
    return new Translation2d[] {
      new Translation2d(center.getX() - hx, center.getY() - hy),
      new Translation2d(center.getX() + hx, center.getY() - hy),
      new Translation2d(center.getX() + hx, center.getY() + hy),
      new Translation2d(center.getX() - hx, center.getY() + hy)
    };
  }

  public static boolean robotIntersects(
      Translation2d center,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    Translation2d[] rect = rectCorners(center, robotLengthMeters, robotWidthMeters);
    for (Obstacle ob : obstacles) {
      if (ob != null && ob.intersectsRectangle(rect)) return true;
    }
    return false;
  }

  static boolean segmentCompletelyBlocked(
      Translation2d start,
      Translation2d goal,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    if (obstacles == null || obstacles.isEmpty()) return false;
    int samples = 6;
    for (int i = 0; i <= samples; i++) {
      double s = i / (double) samples;
      double x = start.getX() + (goal.getX() - start.getX()) * s;
      double y = start.getY() + (goal.getY() - start.getY()) * s;
      Translation2d p = new Translation2d(x, y);
      if (!robotIntersects(p, robotLengthMeters, robotWidthMeters, obstacles)) {
        return false;
      }
    }
    return true;
  }
}
