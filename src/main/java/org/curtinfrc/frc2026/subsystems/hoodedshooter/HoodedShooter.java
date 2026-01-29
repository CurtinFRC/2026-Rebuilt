package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final double WHEEL_DIAMETER = 0.101;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Alert hoodMotorDisconnectedAlert;
  private final Alert hoodMotorTempAlert;
  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[3];
  private final Alert[] shooterMotorTempAlerts = new Alert[3];

  public HoodedShooter(HoodIO hoodIO, ShooterIO shooterIO) {
    this.hoodIO = hoodIO;
    this.shooterIO = shooterIO;

    this.hoodMotorDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
    this.hoodMotorTempAlert =
        new Alert("Hood motor temperature above 60°C.", AlertType.kWarning); // change
    for (int motor = 0; motor < 3; motor++) {
      this.shooterMotorDisconnectedAlerts[motor] =
          new Alert("Shooter motor " + String.valueOf(motor) + " disconnected.", AlertType.kError);
      this.shooterMotorTempAlerts[motor] =
          new Alert(
              "Shooter motor " + String.valueOf(motor) + " temperature above 60°C.",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Hood", hoodInputs);
    Logger.processInputs("Shooter", shooterInputs);

    // Update alerts
    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);
    hoodMotorTempAlert.set(hoodInputs.motorTemperature > 60); // in celcius
    for (int motor = 0; motor < 3; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(shooterInputs.motorTemperatures[motor] > 60);
    }
  }

  public Command setHoodPosition(double position) {
    return run(() -> hoodIO.setPosition(position));
  }

  public Command stopHood() {
    return run(() -> hoodIO.setVoltage(0));
  }

  public Command setHoodVoltage(Supplier<Double> voltage) {
    return run(() -> hoodIO.setVoltage(voltage.get()));
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
    return run(
        () -> {
          hoodIO.setPosition(position);
          shooterIO.setVelocity(velocity);
        });
  }

  public Command stopHoodedShooter() {
    return run(
        () -> {
          hoodIO.setVoltage(0);
          shooterIO.setVoltage(0);
        });
  }
}
