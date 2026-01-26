package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double velocityMetresPerSecond;
    public double accelerationMetresPerSecondPerSecond;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}
}
