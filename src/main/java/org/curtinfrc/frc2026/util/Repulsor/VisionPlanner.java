package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.Kind;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision.ObstacleType;

public class VisionPlanner {
  public static class VisionObstacle extends Obstacle {
    public Translation2d loc;
    public double sizeX;
    public double sizeY;
    public Kind kind;

    public VisionObstacle(Translation2d loc, double strength, ObstacleType type) {
      super(strength, true);
      this.loc = loc;
      this.sizeX = type.getSize().getFirst();
      this.sizeY = type.getSize().getSecond();
      this.kind = type.getKind();
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double distance = loc.getDistance(position);
      if (distance > 3.0) return new Force();

      var dsBase = RepulsorDriverStation.getInstance();
      double clearanceScale = 1.0;
      if (dsBase instanceof NtRepulsorDriverStation ds) {
        clearanceScale = ds.getConfigDouble("clearance_scale");
      }

      double scaledRadius = Math.max(sizeX, sizeY) * 0.5 * clearanceScale;
      double radial = distance - scaledRadius;

      double mag = distToForceMag(radial);
      Translation2d delta = position.minus(loc);
      if (delta.getNorm() < 1e-9 || Math.abs(mag) < 1e-12) {
        return new Force();
      }

      double angleRad = Math.atan2(delta.getY(), delta.getX());

      return new Force(mag, new edu.wpi.first.math.geometry.Rotation2d(angleRad));
    }

    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;

      double rx = sizeX / 2;
      double ry = sizeY / 2;
      for (Translation2d corner : rectCorners) {
        double dx = corner.getX() - loc.getX();
        double dy = corner.getY() - loc.getY();
        if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1) {
          return true;
        }
      }

      double boundingRadius = Math.max(rx, ry);
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < boundingRadius) return true;
      }

      return false;
    }
  }

  private List<RepulsorVision> m_vision = new ArrayList<RepulsorVision>();

  public VisionPlanner() {}

  public VisionPlanner withVision(RepulsorVision vision) {
    m_vision.add(vision);
    return this;
  }

  public void addVision(RepulsorVision vision) {
    m_vision.add(vision);
  }

  public List<VisionObstacle> getObstacles() {
    return m_vision.stream()
        .flatMap(v -> Arrays.stream(v.getObstacles()))
        .map(o -> new VisionObstacle(new Translation2d(o.x(), o.y()), 1.5, o.type()))
        .collect(Collectors.toList());
  }

  public void tick() {
    for (RepulsorVision vision : m_vision) {
      vision.tick();
    }
  }
}
