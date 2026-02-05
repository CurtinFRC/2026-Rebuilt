/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;

public class DriveTuningHeat extends DriveTuning {
  private double baseMaxSpeed = 5.14;
  private double sqrtScale = 6.0;
  private double minStep = 0.02;
  private double nearStart = 0.40;
  private double nearEnd = 0.02;
  private final double MAX_SPEED = 5.14;

  private final Heatmap heatmap;
  private final Supplier<Pose2d> robotPoseSupplier;

  public DriveTuningHeat(Supplier<Pose2d> robotPoseSupplier) {
    super("Drive/Heat");
    this.heatmap = Constants.FIELD.getHeatmap();
    this.robotPoseSupplier = robotPoseSupplier;
  }

  public DriveTuningHeat withBaseMaxSpeed(double mps) {
    this.baseMaxSpeed = mps;
    return this;
  }

  public DriveTuningHeat withSqrtScale(double s) {
    this.sqrtScale = s;
    return this;
  }

  public DriveTuningHeat withMinStep(double m) {
    this.minStep = m;
    return this;
  }

  public DriveTuningHeat withNearWindow(double startM, double endM) {
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

  public double maxLinearSpeedMps(Pose2d robotPose) {
    if (robotPose == null) return baseMaxSpeed;
    double heat = heatmap.heatAt(robotPose.getTranslation());
    double scale = MathUtil.clamp(heat, 0.0, 1.0);
    return baseMaxSpeed * scale;
  }

  @Override
  public double maxLinearSpeedMps() {
    return baseMaxSpeed;
  }

  @Override
  public double minStepMeters() {
    return minStep;
  }

  @Override
  public double baseStepMeters(double distanceMeters, boolean slowDown) {
    double d = Math.max(0.0, distanceMeters);
    if (d <= 0.0) {
      // Logger.recordOutput("Repulsor/Speed", 0.0);
      // Logger.recordOutput("Repulsor/Remaining", 0.0);
      // Logger.recordOutput("Repulsor/Step", 0.0);
      // Logger.recordOutput("Repulsor/Heat", 0.0);
      // Logger.recordOutput("Repulsor/VMaxHeat", 0.0);
      return 0.0;
    }

    if (!slowDown) {
      return Math.min(baseMaxSpeed * dtSeconds(), d);
    }

    double dt = dtSeconds();
    if (dt <= 0.0) {
      dt = 0.02;
    }

    Pose2d pose = getRobotPoseOrNull();
    double vMaxHeat = baseMaxSpeed;
    double heat = 1.0;
    if (pose != null) {
      Translation2d p = pose.getTranslation();
      heat = heatmap.heatAt(p);
      vMaxHeat = baseMaxSpeed * MathUtil.clamp(heat, 0.0, 1.0);
    }

    double vMax = Math.min(vMaxHeat, MAX_SPEED);
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
    // Logger.recordOutput("Repulsor/Heat", heat);
    // Logger.recordOutput("Repulsor/VMaxHeat", vMaxHeat);

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

  private Pose2d getRobotPoseOrNull() {
    try {
      return robotPoseSupplier.get();
    } catch (Throwable t) {
      return null;
    }
  }
}
