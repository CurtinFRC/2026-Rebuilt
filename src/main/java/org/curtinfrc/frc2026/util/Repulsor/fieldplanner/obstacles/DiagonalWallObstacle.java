package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class DiagonalWallObstacle extends Obstacle {
  public final Translation2d a;
  public final Translation2d b;
  public final double maxRange;

  public DiagonalWallObstacle(
      Translation2d a, Translation2d b, double strength, double maxRange) {
    super(strength, true);
    this.a = a;
    this.b = b;
    this.maxRange = maxRange;
  }

  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    double dist = distanceFromPointToSegment(position, a, b);
    if (dist < EPS || dist > maxRange) return new Force();

    double mag = distToForceMag(dist, maxRange);

    Translation2d closest = closestPoint(position);
    Translation2d away = position.minus(closest);
    if (away.getNorm() < EPS) return new Force();

    Translation2d vec = new Translation2d(mag, away.getAngle());
    return new Force(vec.getNorm(), vec.getAngle());
  }

  private Translation2d closestPoint(Translation2d p) {
    Translation2d ab = b.minus(a);
    double abLenSq = ab.getNorm() * ab.getNorm();
    if (abLenSq < EPS) return a;
    double t = Math.max(0, Math.min(1, dot(p.minus(a), ab) / abLenSq));
    return a.plus(ab.times(t));
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    for (Translation2d corner : rectCorners) {
      if (distanceFromPointToSegment(corner, a, b) < 0.1) return true;
    }
    if (FieldPlanner.isPointInPolygon(a, rectCorners)) return true;
    if (FieldPlanner.isPointInPolygon(b, rectCorners)) return true;
    return false;
  }
}

