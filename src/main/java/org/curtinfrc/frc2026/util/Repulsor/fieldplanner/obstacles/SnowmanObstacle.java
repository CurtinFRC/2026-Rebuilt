package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class SnowmanObstacle extends Obstacle {
  public Translation2d loc;
  public double radius = 0.5;

  public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
    super(strength, positive);
    this.loc = loc;
    this.radius = radius;
  }

  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    var targetToLoc = loc.minus(target);
    var targetToLocAngle = angleOr(targetToLoc, Rotation2d.kZero);

    var sidewaysCircle = new Translation2d(1, targetToLocAngle).plus(loc);

    var distLoc = Math.max(EPS, loc.getDistance(position));
    var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
    var outwardsMag = distToForceMag(Math.max(0.01, distLoc - radius));

    var away = position.minus(loc);
    var awayAngle = angleOr(away, targetToLocAngle);

    var sidewaysTheta =
        target
            .minus(position)
            .getAngle()
            .minus(angleOr(position.minus(sidewaysCircle), awayAngle));
    double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
    var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);

    var combined =
        plus(
            new Translation2d(sideways, sidewaysAngle),
            new Translation2d(outwardsMag, awayAngle));
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

