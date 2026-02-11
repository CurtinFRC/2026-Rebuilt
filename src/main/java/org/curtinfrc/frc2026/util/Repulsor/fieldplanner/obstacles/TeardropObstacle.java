package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class TeardropObstacle extends Obstacle {
  public final Translation2d loc;
  public final double primaryMaxRange;
  public final double primaryRadius;
  public final double tailStrength;
  public final double tailLength;
  final double tiny = EPS;

  public TeardropObstacle(
      Translation2d loc,
      double primaryStrength,
      double primaryMaxRange,
      double primaryRadius,
      double tailStrength,
      double tailLength) {
    super(primaryStrength, true);
    this.loc = loc;
    this.primaryMaxRange = primaryMaxRange;
    this.primaryRadius = primaryRadius;
    this.tailStrength = tailStrength;
    this.tailLength = tailLength + primaryMaxRange;
  }

  Rotation2d angleFromVec(Translation2d v, Rotation2d fallback) {
    double x = v.getX(), y = v.getY();
    double n = Math.hypot(x, y);
    return (n > tiny) ? Rotation2d.fromRadians(Math.atan2(y, x)) : fallback;
  }

  Rotation2d angleBetween(Translation2d from, Translation2d to, Rotation2d fallback) {
    double dx = to.getX() - from.getX(), dy = to.getY() - from.getY();
    double n = Math.hypot(dx, dy);
    return (n > tiny) ? Rotation2d.fromRadians(Math.atan2(dy, dx)) : fallback;
  }

  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    var targetToLoc = new Translation2d(loc.getX() - target.getX(), loc.getY() - target.getY());
    var targetToLocAngle = angleFromVec(targetToLoc, Rotation2d.kZero);
    var sidewaysPoint = new Translation2d(tailLength, targetToLocAngle).plus(loc);

    var posToLoc = new Translation2d(position.getX() - loc.getX(), position.getY() - loc.getY());
    double distPosLoc = posToLoc.getNorm();

    Translation2d outwardsForce;
    if (distPosLoc <= primaryMaxRange && distPosLoc >= tiny) {
      double mag =
          distToForceMag(
              Math.max(distPosLoc - primaryRadius, 0), primaryMaxRange - primaryRadius);
      var dir = angleFromVec(posToLoc, targetToLocAngle);
      outwardsForce = new Translation2d(mag, dir);
    } else {
      outwardsForce = Translation2d.kZero;
    }

    var positionRel =
        new Translation2d(position.getX() - loc.getX(), position.getY() - loc.getY())
            .rotateBy(targetToLocAngle.unaryMinus());
    double distanceAlongLine = positionRel.getX();

    Translation2d sidewaysForce;
    double distanceScalar =
        (Math.abs(tailLength) > tiny) ? (distanceAlongLine / tailLength) : 0.0;
    if (distanceScalar >= 0 && distanceScalar <= 1) {
      double secondaryMaxRange =
          MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
      double distanceToLine = Math.abs(positionRel.getY());
      if (distanceToLine <= secondaryMaxRange) {
        double strength;
        if (distanceAlongLine < primaryMaxRange) {
          strength = tailStrength * (distanceAlongLine / Math.max(primaryMaxRange, tiny));
        } else {
          double denom = Math.max(tailLength - primaryMaxRange, tiny);
          strength =
              -tailStrength * distanceAlongLine / denom + tailLength * tailStrength / denom;
        }
        strength *= 1 - distanceToLine / Math.max(secondaryMaxRange, tiny);

        double sidewaysMagBase = tailStrength * strength * (secondaryMaxRange - distanceToLine);

        var posMinusSideways =
            new Translation2d(
                position.getX() - sidewaysPoint.getX(), position.getY() - sidewaysPoint.getY());
        var posMinusSidewaysAngle = angleFromVec(posMinusSideways, targetToLocAngle);
        var toTarget = angleBetween(position, target, posMinusSidewaysAngle);
        var sidewaysTheta =
            new Rotation2d(toTarget.getCos(), toTarget.getSin()).minus(posMinusSidewaysAngle);

        double dir = Math.signum(Math.sin(sidewaysTheta.getRadians()));
        double sidewaysMag = (Math.abs(dir) < tiny) ? 0.0 : sidewaysMagBase * dir;

        sidewaysForce =
            new Translation2d(sidewaysMag, targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
      } else {
        sidewaysForce = Translation2d.kZero;
      }
    } else {
      sidewaysForce = Translation2d.kZero;
    }

    var sum =
        new Translation2d(
            outwardsForce.getX() + sidewaysForce.getX(),
            outwardsForce.getY() + sidewaysForce.getY());
    if (sum.getNorm() < tiny) return new Force();
    return new Force(sum.getNorm(), angleFromVec(sum, Rotation2d.kZero));
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
    for (Translation2d corner : rectCorners)
      if (corner.getDistance(loc) < primaryMaxRange) return true;
    for (int i = 0; i < rectCorners.length; i++) {
      Translation2d a = rectCorners[i];
      Translation2d b = rectCorners[(i + 1) % rectCorners.length];
      if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < primaryMaxRange) return true;
    }

    Rotation2d tailDir = new Rotation2d();
    Translation2d tailStart = loc;
    Translation2d tailEnd = tailStart.plus(new Translation2d(tailLength, tailDir));

    for (Translation2d corner : rectCorners) {
      Translation2d delta = tailEnd.minus(tailStart);
      Translation2d unit = delta.div(delta.getNorm());
      double projection = dot(corner.minus(tailStart), unit);
      if (projection >= 0 && projection <= tailLength) {
        double lateral =
            Math.abs((corner.minus(tailStart)).rotateBy(tailDir.unaryMinus()).getY());
        if (lateral < primaryMaxRange) return true;
      }
    }
    return false;
  }
}

