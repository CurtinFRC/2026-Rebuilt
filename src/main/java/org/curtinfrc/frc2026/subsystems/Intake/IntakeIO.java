package org.curtinfrc.frc2026.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double frontMotorAppliedVoltage;
    double frontMotorCurrentAmps;
    double frontMotorAngularVelocity;
    double frontMotorPosition;

    double backMotorAppliedVoltage;
    double backMotorCurrentAmps;
    double backMotorAngularVelocity;
    double backMotorPosition;
  }

  public default void setVoltage(double Volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVelocity(double Velocity) {}
}
