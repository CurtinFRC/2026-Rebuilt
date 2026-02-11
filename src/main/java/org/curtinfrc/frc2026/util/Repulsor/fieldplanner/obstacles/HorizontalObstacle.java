package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class HorizontalObstacle extends Obstacle {
  public double y;

  public HorizontalObstacle(double y, double strength, boolean positive) {
    super(strength, positive);
    this.y = y;
  }

  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    return new Force(0, distToForceMag(y - position.getY(), 1));
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    for (Translation2d a : rectCorners) if (Math.abs(a.getY() - y) < 0.1) return true;
    return false;
  }
}

