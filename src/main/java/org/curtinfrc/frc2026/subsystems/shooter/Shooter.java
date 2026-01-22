package org.curtinfrc.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command setVoltage(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command stop() {
    return run(() -> io.setVoltage(0));
  }

  public Command setSpeed(double angularVelocityRotationsPerSecond) {
    return run(() -> io.setSpeed(angularVelocityRotationsPerSecond));
  }
}
