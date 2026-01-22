package org.curtinfrc.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double positionRotations;
    public double angularVelocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
    public boolean atTargetPosition; // unused, setting angle of hood
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double positionAngle) {}
}
