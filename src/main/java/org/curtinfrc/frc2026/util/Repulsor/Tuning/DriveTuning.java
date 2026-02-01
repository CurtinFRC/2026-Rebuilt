package org.curtinfrc.frc2026.util.Repulsor.Tuning;

public abstract class DriveTuning extends Tuning {
  protected DriveTuning(String key) {
    super(key);
  }

  public abstract double maxLinearSpeedMps();

  public abstract double minStepMeters();

  public abstract double baseStepMeters(double distanceMeters, boolean slowDown);

  public abstract double nearGoalScale(double distanceMeters);

  public abstract double scaleForTurning(double yawDeltaRad, boolean isScoring);

  public double stepSizeMeters(double distanceMeters, double obstacleMag, boolean isScoring, boolean slowDown) {
    double base = baseStepMeters(Math.max(0.0, distanceMeters), slowDown);
    return base;
    // double near = nearGoalScale(distanceMeters);
    // double obs = 1.0 / (1.0 + obstacleMag);
    // return Math.max(
    //     minStepMeters(), Math.min(maxLinearSpeedMps() * dtSeconds(), base * near * obs));
  }
}
