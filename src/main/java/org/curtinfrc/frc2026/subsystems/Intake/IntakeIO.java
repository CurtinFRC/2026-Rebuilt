package org.curtinfrc.frc2026.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double AppliedVoltage;
    double CurrentAmps;
    double angularVelocity;
  }

  public default void setVoltage(double Volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVelocity(double Velocity) {}
}
