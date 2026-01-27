package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;

public final class IntakeFootprint {

  public static IntakeFootprint robotSquare(double robotSideMeters) {
    double h = 0.5 * robotSideMeters;
    return new IntakeFootprint(new Rect(new Translation2d(0.0, 0.0), h, h));
  }

  public static IntakeFootprint robotRect(double robotLengthMeters, double robotWidthMeters) {
    return new IntakeFootprint(
        new Rect(new Translation2d(0.0, 0.0), 0.5 * robotLengthMeters, 0.5 * robotWidthMeters));
  }

  public static IntakeFootprint frontRect(
      double robotLengthMeters, double intakeDepthMeters, double intakeWidthMeters) {
    double hx = 0.5 * intakeDepthMeters;
    double hy = 0.5 * intakeWidthMeters;
    double cx = 0.5 * robotLengthMeters + hx;
    return new IntakeFootprint(new Rect(new Translation2d(cx, 0.0), hx, hy));
  }

  private final Shape shape;

  private IntakeFootprint(Shape shape) {
    this.shape = Objects.requireNonNull(shape);
  }

  public boolean containsPointRobotFrame(Translation2d pRobot) {
    return shape.contains(pRobot);
  }

  public Translation2d supportPointRobotFrame(Translation2d dirRobot) {
    return shape.support(dirRobot);
  }

  public Translation2d snapCenterSoFootprintTouchesPoint(
      Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

    Translation2d dirField = pointField.minus(desiredCenterField);
    Translation2d dirRobot = dirField.rotateBy(robotHeading.unaryMinus());

    double n2 = dirRobot.getX() * dirRobot.getX() + dirRobot.getY() * dirRobot.getY();
    if (n2 < 1e-12) {
      dirRobot = new Translation2d(1.0, 0.0);
    }

    Translation2d contactRobot = shape.support(dirRobot);
    Translation2d contactField = contactRobot.rotateBy(robotHeading);

    return pointField.minus(contactField);
  }

  public Translation2d snapCenterSoPointIsInsideFootprint(
      Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

    Translation2d pRobot = pointField.minus(desiredCenterField).rotateBy(robotHeading.unaryMinus());
    Translation2d clampedRobot = shape.closestPointInside(pRobot);
    Translation2d deltaRobot = pRobot.minus(clampedRobot);

    return desiredCenterField.plus(deltaRobot.rotateBy(robotHeading));
  }

  private interface Shape {
    boolean contains(Translation2d p);

    Translation2d support(Translation2d dir);

    Translation2d closestPointInside(Translation2d p);
  }

  private static final class Rect implements Shape {
    private final Translation2d c;
    private final double hx;
    private final double hy;

    Rect(Translation2d center, double halfX, double halfY) {
      this.c = Objects.requireNonNull(center);
      this.hx = Math.max(0.0, halfX);
      this.hy = Math.max(0.0, halfY);
    }

    @Override
    public boolean contains(Translation2d p) {
      double dx = p.getX() - c.getX();
      double dy = p.getY() - c.getY();
      return Math.abs(dx) <= hx + 1e-9 && Math.abs(dy) <= hy + 1e-9;
    }

    @Override
    public Translation2d support(Translation2d dir) {
      double sx = dir.getX() >= 0.0 ? hx : -hx;
      double sy = dir.getY() >= 0.0 ? hy : -hy;
      return new Translation2d(c.getX() + sx, c.getY() + sy);
    }

    @Override
    public Translation2d closestPointInside(Translation2d p) {
      double x = clamp(p.getX(), c.getX() - hx, c.getX() + hx);
      double y = clamp(p.getY(), c.getY() - hy, c.getY() + hy);
      return new Translation2d(x, y);
    }

    private static double clamp(double v, double lo, double hi) {
      return Math.max(lo, Math.min(hi, v));
    }
  }
}
