package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

