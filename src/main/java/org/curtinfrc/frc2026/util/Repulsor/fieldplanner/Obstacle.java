package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public abstract class Obstacle {
  protected static final double EPS = 1e-9;

  double strength = 1.0;
  boolean positive = true;

  public Obstacle(double strength, boolean positive) {
    this.strength = strength;
    this.positive = positive;
  }

  public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

  protected double distToForceMag(double dist) {
    var forceMag = strength / (0.00001 + Math.abs(dist * dist));
    forceMag *= positive ? 1 : -1;
    return forceMag;
  }

  protected double distToForceMag(double dist, double falloff) {
    var original = strength / (0.00001 + Math.abs(dist * dist));
    var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
    return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
  }

  protected static Rotation2d angleOr(Translation2d v, Rotation2d fallback) {
    return v.getNorm() > EPS ? v.getAngle() : fallback;
  }

  protected static Translation2d plus(Translation2d a, Translation2d b) {
    return new Translation2d(a.getX() + b.getX(), a.getY() + b.getY());
  }

  protected static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  protected static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }

  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    return false;
  }
}

