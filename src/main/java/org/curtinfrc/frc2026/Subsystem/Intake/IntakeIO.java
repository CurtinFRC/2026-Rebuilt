package org.curtinfrc.frc2026.Subsystem.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double AppliedVoltage;
    double CurrentAmps;
    double angularVelocity;
  }

  public default void setvoltage(double Volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVelocity(double Velocity) {}
}
