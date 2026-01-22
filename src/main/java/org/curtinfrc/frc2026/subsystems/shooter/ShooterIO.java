package org.curtinfrc.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double angularVelocityRotationsPerSecond;
    public double accelerationRotationsPerSecondPerSecond;
    public boolean atTargetSpeed;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double angularVelocityRotationsPerSecond) {}
}
