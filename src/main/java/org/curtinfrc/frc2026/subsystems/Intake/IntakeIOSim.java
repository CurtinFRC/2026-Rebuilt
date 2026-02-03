package org.curtinfrc.frc2026.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim intakeSim;

  private final DCMotor intakeMotor = DCMotor.getNEO(1);
  private double voltage = 0.0;

  public IntakeIOSim() {
    intakeSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeMotor, 0.02, 100), intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(0.02);
    inputs.frontMotorAppliedVoltage = voltage;
    inputs.frontMotorCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.frontMotorAngularVelocity = intakeSim.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = MathUtil.clamp(voltage, -12, 12);
    intakeSim.setInputVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocity) {
    this.voltage = MathUtil.clamp(voltage, -12, 12);
    intakeSim.setAngularVelocity(velocity);
  }
}