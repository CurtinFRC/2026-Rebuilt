package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MagRoller extends SubsystemBase {
  private final MagRollerIO io;
  private double currentPosition = 0;
  private String name;

  private final MagRollerIOInputsAutoLogged inputs = new MagRollerIOInputsAutoLogged();

  public MagRoller(MagRollerIO expected_io, String name) {
    this.io = expected_io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command stop() {
    return run(() -> io.setVoltage(0));
  }

  public Command runMotor(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command stayAtCurrentPosition() {
    return runOnce(() -> currentPosition = io.getPosition())
        .andThen(run(() -> io.setPosition(currentPosition)));
  }

  public Command runAtVelocityPID(double targetVelocityRPS) {
    return run(() -> io.setVelocityRPS(targetVelocityRPS));
  }
}
