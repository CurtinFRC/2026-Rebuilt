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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class PointObstacle extends Obstacle {
  public Translation2d loc;
  public double radius = 0.5;

  public PointObstacle(Translation2d loc, double strength, boolean positive) {
    super(strength, positive);
    this.loc = loc;
  }

  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    var dist = loc.getDistance(position);
    if (dist > 4) return new Force();
    if (dist < EPS) return new Force();

    var outwardsMag = distToForceMag(dist - radius);
    var away = position.minus(loc);

    var theta = target.minus(position).getAngle().minus(away.getAngle());
    double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

    var combined =
        plus(
            new Translation2d(mag, away.getAngle().rotateBy(Rotation2d.kCCW_90deg)),
            new Translation2d(outwardsMag, away.getAngle()));

    if (combined.getNorm() < EPS) return new Force();
    return new Force(combined.getNorm(), combined.getAngle());
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
    for (Translation2d corner : rectCorners) if (corner.getDistance(loc) < radius) return true;
    for (int i = 0; i < rectCorners.length; i++) {
      Translation2d a = rectCorners[i];
      Translation2d b = rectCorners[(i + 1) % rectCorners.length];
      if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
    }
    return false;
  }
}
