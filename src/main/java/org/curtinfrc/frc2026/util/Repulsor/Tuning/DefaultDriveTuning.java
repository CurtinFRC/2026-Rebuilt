package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.MathUtil;

public class DefaultDriveTuning extends DriveTuning {
  private double maxSpeed = 5.14;
  private double sqrtScale = 6.0;
  private double minStep = 0.02;
  private double nearStart = 0.40;
  private double nearEnd = 0.02;
  private final double MAX_SPEED = 8;

  public DefaultDriveTuning() {
    super("Drive/Default");
  }

  public DefaultDriveTuning withMaxSpeed(double mps) {
    this.maxSpeed = mps;
    return this;
  }

  public DefaultDriveTuning withSqrtScale(double s) {
    this.sqrtScale = s;
    return this;
  }

  public DefaultDriveTuning withMinStep(double m) {
    this.minStep = m;
    return this;
  }

  public DefaultDriveTuning withNearWindow(double startM, double endM) {
    this.nearStart = startM;
    this.nearEnd = endM;
    return this;
  }

  @Override
  public void applyDefaults() {
    setDtSeconds(0.02);
  }

  @Override
  public void reset() {}

  @Override
  public double maxLinearSpeedMps() {
    return maxSpeed;
  }

  @Override
  public double minStepMeters() {
    return minStep;
  }

  // @Override public double baseStepMeters(double distanceMeters) { var speed = Math.min(8
  // /*5.14*/, Math.sqrt(24 * distanceMeters)); // Logger.recordOutput("Repulsor/Speed", speed);
  // return
  // speed * dtSeconds(); }

  @Override
  public double baseStepMeters(double distanceMeters, boolean slowDown) {
    double d = Math.max(0.0, distanceMeters);
    if (d <= 0.0) {
      // Logger.recordOutput("Repulsor/Speed", 0.0);
      // Logger.recordOutput("Repulsor/Remaining", 0.0);
      // Logger.recordOutput("Repulsor/Step", 0.0);
      return 0.0;
    }

    if (!slowDown) {
      return Math.min(maxSpeed * dtSeconds(), d);
    }

    double dt = dtSeconds();
    if (dt <= 0.0) {
      dt = 0.02;
    }

    double vMax = Math.min(maxSpeed, MAX_SPEED);
    double aMax = Math.max(0.01, sqrtScale);

    double dBrake = vMax * vMax / (2.0 * aMax);

    double v;
    if (d > dBrake) {
      v = vMax;
    } else {
      v = Math.sqrt(2.0 * aMax * d);
    }

    if (d < nearStart && nearStart > nearEnd) {
      double t = (d - nearEnd) / (nearStart - nearEnd);
      t = MathUtil.clamp(t, 0.0, 1.0);
      double s = t * t * (3.0 - 2.0 * t);
      v *= s;
    }

    double step = v * dt;

    if (step < minStep && d > minStep) {
      step = minStep;
    }

    if (step > d) {
      step = d;
    }

    // Logger.recordOutput("Repulsor/Speed", v);
    // Logger.recordOutput("Repulsor/Remaining", d);
    // Logger.recordOutput("Repulsor/Step", step);

    return step;
  }

  @Override
  public double nearGoalScale(double distanceMeters) {
    return 1.0;
  }

  @Override
  public double scaleForTurning(double yawDeltaRad, boolean isScoring) {
    return 1.0;
  }
}
