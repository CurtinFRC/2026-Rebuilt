package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final double WHEEL_DIAMETER = 0.101;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public HoodedShooter(HoodIO hoodIO, ShooterIO shooterIO) {
    this.hoodIO = hoodIO;
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    shooterIO.updateInputs(shooterInputs);

    Logger.processInputs("Hood", hoodInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }

  public Command setHoodPosition(double position) {
    return run(() -> hoodIO.setPosition(position));
  }

  public Command stopHood() {
    return run(() -> hoodIO.setVoltage(0));
  }

  public Command setHoodVoltage(double voltage) {
    return run(() -> hoodIO.setVoltage(voltage));
  }

  public Command setShooterVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public Command stopShooter() {
    return run(() -> shooterIO.setVoltage(0));
  }

  public Command setShooterVelocity(double velocity) {
    return run(() -> shooterIO.setVelocity(velocity));
  }

  public Command setHoodedShooterPositionAndVelocity(double position, double velocity) {
    return run(() -> {
      hoodIO.setPosition(position);
      shooterIO.setVelocity(velocity);
    });
  }

  public Command resetHoodedShooter() {
    return run(() -> {
      hoodIO.setVoltage(0);
      shooterIO.setVoltage(0);
    });
  }
}
