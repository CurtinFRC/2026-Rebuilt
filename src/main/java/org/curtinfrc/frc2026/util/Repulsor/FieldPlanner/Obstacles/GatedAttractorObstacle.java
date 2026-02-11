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
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class GatedAttractorObstacle extends Obstacle {
  public final Translation2d center;
  public final double maxRange;
  public final double soften;
  public final Translation2d[] gatePoly;
  public final Translation2d bypassPoint;
  public final double bypassStrengthScale;
  public final double bypassRange;
  public final boolean waypoint;

  public GatedAttractorObstacle(
      Translation2d center,
      double strength,
      double maxRange,
      Translation2d[] gatePoly,
      Translation2d bypassPoint,
      double bypassStrengthScale,
      double bypassRange) {
    this(
        center,
        strength,
        maxRange,
        gatePoly,
        bypassPoint,
        bypassStrengthScale,
        bypassRange,
        0.18,
        false);
  }

  public GatedAttractorObstacle(
      Translation2d center,
      double strength,
      double maxRange,
      Translation2d[] gatePoly,
      Translation2d bypassPoint,
      double bypassStrengthScale,
      double bypassRange,
      boolean waypoint) {
    this(
        center,
        strength,
        maxRange,
        gatePoly,
        bypassPoint,
        bypassStrengthScale,
        bypassRange,
        0.18,
        waypoint);
  }

  public GatedAttractorObstacle(
      Translation2d center,
      double strength,
      double maxRange,
      Translation2d[] gatePoly,
      Translation2d bypassPoint,
      double bypassStrengthScale,
      double bypassRange,
      double soften,
      boolean waypoint) {
    super(strength, true);
    this.center = center;
    this.maxRange = Math.max(0.0, maxRange);
    this.gatePoly = gatePoly;
    this.bypassPoint = bypassPoint;
    this.bypassStrengthScale = Math.max(1.0, bypassStrengthScale);
    this.bypassRange = Math.max(0.0, bypassRange);
    this.soften = Math.max(1e-6, soften);
    this.waypoint = waypoint;
  }

  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    Translation2d pullTo = center;
    double range = maxRange;
    double strengthScale = 1.0;

    if (gatePoly != null
        && bypassPoint != null
        && FieldPlanner.segmentIntersectsPolygonOuter(position, center, gatePoly)) {
      pullTo = bypassPoint;
      range = bypassRange;
      strengthScale = bypassStrengthScale;
    }

    double dist = position.getDistance(pullTo);
    if (dist < EPS || dist > range) return new Force();

    double magBase = (strength * strengthScale) / (soften + dist * dist);
    double magFalloff = (strength * strengthScale) / (soften + range * range);
    double mag = Math.max(magBase - magFalloff, 0.0);
    if (mag < EPS) return new Force();

    Translation2d toward = pullTo.minus(position);
    if (toward.getNorm() < EPS) return new Force();

    return new Force(mag, toward.getAngle());
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    return false;
  }
}
