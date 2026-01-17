package org.curtinfrc.frc2026.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO parameter_io) {
    io = parameter_io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command indexerRaw(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command stop() {
    return run(() -> io.setVoltage(0));
  }

  public Command setSpeed(double targetSpeed) {
    return run(() -> io.setSpeed(targetSpeed));
  }
}
