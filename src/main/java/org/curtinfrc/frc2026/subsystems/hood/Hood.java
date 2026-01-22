package org.curtinfrc.frc2026.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double positionAngle) {
    io.setPosition(positionAngle);
  }
}
