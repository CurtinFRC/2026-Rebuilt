package org.curtinfrc.frc2026.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotor indexerMotor = DCMotor.getNEO(1);
  private final DCMotorSim indexerMotorSim;
  private double volts = 0;

  public IndexerIOSim() {
    indexerMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(indexerMotor, 3, 1), indexerMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotorSim.setInputVoltage(volts);
    indexerMotorSim.update(0.02);
    inputs.appliedVolts = indexerMotorSim.getInputVoltage();
    inputs.currentAmps = indexerMotorSim.getCurrentDrawAmps();
    inputs.positionRotations = indexerMotorSim.getAngularPositionRotations();
    inputs.angularVelocityRotationsPerMinute = indexerMotorSim.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
