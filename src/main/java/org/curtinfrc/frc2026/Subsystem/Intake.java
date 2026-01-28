package org.curtinfrc.frc2026.Subsystem;

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
  private static final double STOP_MOTOR_VOLTAGE = 0;
  // consume means intake//
  private final double CONSUME_VEL_RPS = 8;
  // vel means velocity//
  private static final double CONSUME_VOLTS = 8;
  private static final double IDLE_VOLTS = 2;
  private static final double IDLE_VEL_RPS = 2;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command Stop() {
    return run(() -> io.setVoltage(STOP_MOTOR_VOLTAGE)).withName("stop");
  }

  public Command RawControlConsume() {
    return run(() -> io.setVoltage(0.5)).withName("consumeVolts");
  }

  public Command RawIdle() {
    return run(() -> io.setVoltage(0)).withName("idleVolts");
  }

  public Command ControlConsume() {
    return run(() -> io.setVelocity(CONSUME_VEL_RPS)).withName("consumeVel");
  }

  public Command Idle() {
    return run(() -> io.setVelocity(IDLE_VEL_RPS)).withName("idleVel");
  }
}
