package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
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

