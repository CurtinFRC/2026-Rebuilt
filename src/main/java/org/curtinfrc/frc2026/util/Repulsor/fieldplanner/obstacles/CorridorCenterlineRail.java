package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class CorridorCenterlineRail extends Obstacle {
  private final double xCenter;
  private final double xHalfWindow;
  private final double yCenter;
  private final double yHalfWidth;
  private final double maxForce;

  public CorridorCenterlineRail(
      double xCenter,
      double xHalfWindow,
      double yCenter,
      double yHalfWidth,
      double strength,
      double maxForce) {
    super(strength, true);
    this.xCenter = xCenter;
    this.xHalfWindow = Math.max(0.05, xHalfWindow);
    this.yCenter = yCenter;
    this.yHalfWidth = Math.max(0.10, yHalfWidth);
    this.maxForce = Math.max(0.1, maxForce);
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  private static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    double dx = Math.abs(position.getX() - xCenter);
    if (dx >= xHalfWindow) return new Force();

    double wx = smooth01(1.0 - (dx / xHalfWindow));

    double ey = position.getY() - yCenter;

    double e = ey / yHalfWidth;

    double f = -strength * wx * (e) / (0.35 + e * e);

    if (f > maxForce) f = maxForce;
    if (f < -maxForce) f = -maxForce;

    Translation2d v = new Translation2d(0.0, f);
    double n = v.getNorm();
    if (n < 1e-9) return new Force();
    return new Force(n, v.getAngle());
  }
}

