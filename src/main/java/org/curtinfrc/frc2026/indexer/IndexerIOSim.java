package org.curtinfrc.frc2026.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotor motor = DCMotor.getNEO(1);
  private final DCMotorSim motorSim;
  private double volts = 0;

  public IndexerIOSim() {
    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 3, 1), motor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    motorSim.setInputVoltage(volts);
    motorSim.update(0.02);
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = motorSim.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
