package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MagRoller extends SubsystemBase {
  private final MagRollerIO io;

  private final MagRollerIOInputsAutoLogged inputs = new MagRollerIOInputsAutoLogged();

  public MagRoller(MagRollerIO expected_io) {
    this.io = expected_io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0));
  }

  public Command runMotor(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command stayAtCurrentPosition() {
    double currentPosition = io.getPosition();
    return run(() -> io.setPosition(currentPosition));
  }
}
