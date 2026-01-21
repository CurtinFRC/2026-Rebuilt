package org.curtinfrc.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0)).withName("Stop");
  }

  public Command intake(double volts) {
    return run(() -> io.setVoltage(volts)).withName("Intake " + volts);
  }

  public Command intake() {
    return run(() -> io.setVoltage(8));
  }
}
