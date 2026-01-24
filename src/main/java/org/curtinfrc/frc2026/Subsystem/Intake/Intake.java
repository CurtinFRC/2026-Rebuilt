package org.curtinfrc.frc2026.Subsystem.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  // These variables are used for the voltage and velocity//
  private final double stopMotor = 0;
  // consume means intake//
  private final double consumeVel = 8;
  // vel means velocity//
  private final double consumeVolts = 8;
  private final double idleVolts = 2;
  private final double idleVel = 2;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command Stop() {
    return run(() -> io.setVoltage(stopMotor)).withName("stop");
  }

  public Command RawControlConsume() {
    return run(() -> io.setVoltage(consumeVolts)).withName("consumeVolts");
  }

  public Command RawIdle() {
    return run(() -> io.setVoltage(idleVolts)).withName("idleVolts");
  }

  public Command ControlConsume(double target) {
    return run(() -> io.setVelocity(target)).withName("consumeVel");
  }

  public Command Idle() {
    return run(() -> io.setVelocity(idleVel)).withName("idleVel");
  }
}
