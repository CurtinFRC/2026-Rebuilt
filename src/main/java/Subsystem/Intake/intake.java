package Subsystem.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class intake extends SubsystemBase {
  private final intakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public intake(intakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return run(() -> io.setvoltage(0)).withName("stop");
  }

  public Command Consume() {
    return run(() -> io.setvoltage(8)).withName("consume");
  }

  public Command idle() {
    return run(() -> io.setvoltage(2)).withName("idle");
  }
}
