package Subsystem.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface intakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double AppliedVoltage;
    double CurrentAmps;
  }

  public default void setvoltage(double Volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
