package org.curtinfrc.frc2026.subsystems.intake;

import com.revrobotics.spark.SparkMax;

public class IntakeIOComp implements IntakeIO {
  private static final int ID = 43;

  private final SparkMax motor = new SparkMax(ID, SparkMax.MotorType.kBrushless);

  public IntakeIOComp() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.angularVelocityRotationsPerMinute = motor.getEncoder().getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(-volts);
  }
}
