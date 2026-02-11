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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class VerticalObstacle extends Obstacle {
  public double x;

  public VerticalObstacle(double x, double strength, boolean positive) {
    super(strength, positive);
    this.x = x;
  }

  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    return new Force(distToForceMag(x - position.getX(), 1), 0);
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    for (Translation2d a : rectCorners) if (Math.abs(a.getX() - x) < 0.1) return true;
    return false;
  }
}
