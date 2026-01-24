package org.curtinfrc.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
    public boolean frontSensor;
    public boolean backSensor;
  }

  public default void setVoltage(double volts) {
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}

