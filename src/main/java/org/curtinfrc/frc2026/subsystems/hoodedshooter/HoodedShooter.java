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
  public static final double MOTOR_WARNING_TEMP = 60;
  public static final double WHEEL_DIAMETER =
      (Constants.robotType == Constants.RobotType.COMP) ? 0.1 : 0.101;
  public static final int SHOOTER_MOTOR_NUMBER =
      (Constants.robotType == Constants.RobotType.COMP) ? 4 : 4;
  public static final int HOOD_MOTOR_NUMBER =
      (Constants.robotType == Constants.RobotType.COMP) ? 2 : 1;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;

  private final Alert[] hoodMotorDisconnectedAlerts = new Alert[HOOD_MOTOR_NUMBER];
  private final Alert[] hoodMotorTempAlerts = new Alert[HOOD_MOTOR_NUMBER];

  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[SHOOTER_MOTOR_NUMBER];
  private final Alert[] shooterMotorTempAlerts = new Alert[SHOOTER_MOTOR_NUMBER];

  private double hoodTarget = 0;
  private double shooterTarget = 0;

  public HoodedShooter(HoodIO hoodIO, ShooterIO shooterIO, Supplier<Pose2d> robotPose) {
    this.hoodIO = hoodIO;
    this.shooterIO = shooterIO;
    this.robotPose = robotPose;

    for (int motor = 0; motor < HOOD_MOTOR_NUMBER; motor++) {
      hoodMotorDisconnectedAlerts[motor] =
          new Alert("Hood motor " + String.valueOf(motor) + " disconnected.", AlertType.kError);
      hoodMotorTempAlerts[motor] =
          new Alert(
              "Hood motor " + String.valueOf(motor) + " temperature above 60°C.",
              AlertType.kWarning);
    }
    for (int motor = 0; motor < SHOOTER_MOTOR_NUMBER; motor++) {
      shooterMotorDisconnectedAlerts[motor] =
          new Alert("Shooter motor " + String.valueOf(motor) + " disconnected.", AlertType.kError);
      shooterMotorTempAlerts[motor] =
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
    Logger.recordOutput("HoodedShooter/hoodTarget", hoodTarget);
    Logger.recordOutput("HoodedShooter/shooterTarget", shooterTarget);

    for (int motor = 0; motor < HOOD_MOTOR_NUMBER; motor++) {
      hoodMotorDisconnectedAlerts[motor].set(!hoodInputs.motorsConnected[motor]);
      hoodMotorTempAlerts[motor].set(hoodInputs.motorTemperatures[motor] > MOTOR_WARNING_TEMP);
    }
    for (int motor = 0; motor < SHOOTER_MOTOR_NUMBER; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(
          shooterInputs.motorTemperatures[motor] > MOTOR_WARNING_TEMP);
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
          hoodTarget = position;
          shooterTarget = velocity;
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
