package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.Constants.Mode;
import org.curtinfrc.frc2026.sim.BallSim;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final double WHEEL_DIAMETER = 0.1;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;

  private final Alert[] hoodMotorDisconnectedAlerts = new Alert[2];
  private final Alert[] hoodMotorTempAlerts = new Alert[2];

  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[3];
  private final Alert[] shooterMotorTempAlerts = new Alert[3];

  public HoodedShooter(HoodIO hoodIO, ShooterIO shooterIO, Supplier<Pose2d> robotPose) {
    this.hoodIO = hoodIO;
    this.shooterIO = shooterIO;
    this.robotPose = robotPose;

    for (int motor = 0; motor < hoodMotorDisconnectedAlerts.length; motor++) {
      hoodMotorDisconnectedAlerts[motor] =
          new Alert("Hood motor " + String.valueOf(motor) + " disconnected.", AlertType.kError);
      hoodMotorTempAlerts[motor] =
          new Alert(
              "Hood motor " + String.valueOf(motor) + " temperature above 60°C.",
              AlertType.kWarning);
    }
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

    for (int motor = 0; motor < 1; motor++) {
      hoodMotorDisconnectedAlerts[motor].set(!hoodInputs.motorsConnected[motor]);
      hoodMotorTempAlerts[motor].set(hoodInputs.motorTemperatures[motor] > 60);
    }
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

          if (Constants.getMode() == Mode.SIM) {
            shooterIO.addSimBall(
                new BallSim(
                    velocity,
                    Rotation2d.fromDegrees(position + 90),
                    new Pose3d(robotPose.get())
                        .plus(new Transform3d(0.2, 0.0, 0.3, Rotation3d.kZero))));
          }
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
