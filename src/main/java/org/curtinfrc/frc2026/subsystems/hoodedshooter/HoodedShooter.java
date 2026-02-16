package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.Constants.Mode;
import org.curtinfrc.frc2026.sim.BallSim;
import org.curtinfrc.frc2026.util.FieldConstants;
import org.curtinfrc.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  // TODO fix
  public static final Translation2d HUB_LOCATION =
      ChoreoAllianceFlipUtil.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d());
  // new Translation2d(
  //     FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY());

  public static final double WHEEL_DIAMETER = 0.101;

  public static final double READY_SHOOTER_VELOCITY_TOLERANCE = 1;
  public static final double READY_HOOD_POSITION_TOLERANCE = 1.0;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOTER_VELOCITY =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_BALL_FLIGHT_TIME =
      new InterpolatingDoubleTreeMap();
  public static final double SCORING_SHOOTER_VELOCITY = 25;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private final LoggedTunableNumber tunableHoodSetpoint =
      new LoggedTunableNumber("HoodSetpoint", 90);
  private final LoggedTunableNumber tunableShooterSetpoint =
      new LoggedTunableNumber("ShooterSetpoint", 19.5);
  private double shooterTarget = 0;
  private double hoodTarget = 0;

  private final Alert hoodMotorDisconnectedAlert;
  private final Alert hoodMotorTempAlert;
  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[3];
  private final Alert[] shooterMotorTempAlerts = new Alert[3];

  public final Trigger hoodedShooterReady =
      new Trigger(
          () ->
              Math.abs(hoodTarget - hoodInputs.positionRotations) <= READY_HOOD_POSITION_TOLERANCE
                  && Math.abs(shooterTarget - shooterInputs.velocityMetresPerSecond)
                      <= READY_SHOOTER_VELOCITY_TOLERANCE);

  public HoodedShooter(
      ShooterIO shooterIO,
      HoodIO hoodIO,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    this.shooterIO = shooterIO;
    this.hoodIO = hoodIO;
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    DISTANCE_TO_SHOOTER_VELOCITY.put(2.45, 19.5);
    DISTANCE_TO_HOOD_ANGLE.put(2.45, 82.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(2.45, 1.15);

    DISTANCE_TO_SHOOTER_VELOCITY.put(3.6, 19.5);
    DISTANCE_TO_HOOD_ANGLE.put(3.6, 75.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(3.6, 1.2);

    DISTANCE_TO_SHOOTER_VELOCITY.put(3.8, 19.5);
    DISTANCE_TO_HOOD_ANGLE.put(3.8, 73.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(3.8, 1.43);

    DISTANCE_TO_SHOOTER_VELOCITY.put(5.11, 19.5);
    DISTANCE_TO_HOOD_ANGLE.put(5.11, 65.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(5.11, 1.2);

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

    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);
    hoodMotorTempAlert.set(hoodInputs.motorTemperature > 60); // in celcius
    for (int motor = 0; motor < 3; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(shooterInputs.motorTemperatures[motor] > 60);
    }
  }

  public Trigger readyToShoot() {
    return hoodedShooterReady;
  }

  public Translation2d getVirtualHubLocation(Supplier<Translation2d> hub_location) {
    double realDistanceLength =
        hub_location.get().minus(robotPose.get().getTranslation()).getNorm();
    Translation2d robotVel =
        new Translation2d(
            robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond);
    double airTime = DISTANCE_TO_BALL_FLIGHT_TIME.get(realDistanceLength);

    Translation2d hubCompensationOffset = robotVel.times(-airTime);
    Translation2d compensatedHubLocation = hub_location.get().plus(hubCompensationOffset);
    return (realDistanceLength > 1) ? compensatedHubLocation : hub_location.get();
  }

  public Command shootAtHub() {
    return run(
        () -> {
          Translation2d compensatedHubLocation = getVirtualHubLocation(() -> HUB_LOCATION);

          double compensatedDistanceLength =
              compensatedHubLocation.minus(robotPose.get().getTranslation()).getNorm();

          if (Constants.tuningMode == true) {
            hoodTarget = tunableHoodSetpoint.get();
            shooterTarget = tunableShooterSetpoint.get();
          } else {
            hoodTarget = DISTANCE_TO_HOOD_ANGLE.get(compensatedDistanceLength);
            shooterTarget = DISTANCE_TO_SHOOTER_VELOCITY.get(compensatedDistanceLength);
          }

          hoodIO.setPosition(hoodTarget);
          shooterIO.setVelocity(shooterTarget);

          if (Constants.getMode() == Mode.SIM) {
            shooterIO.addSimBall(
                new BallSim(
                    shooterTarget,
                    Rotation2d.fromDegrees(hoodTarget + 90),
                    new Pose3d(robotPose.get())
                        .plus(new Transform3d(0.2, 0.0, 0.3, Rotation3d.kZero))));
          }
        });
  }

  public Command setHoodPosition(double positionDegrees) {
    return run(
        () -> {
          hoodIO.setPosition(positionDegrees);
        });
  }

  public Command setHoodVoltage(double voltage) {
    return run(() -> hoodIO.setVoltage(voltage));
  }

  public Command stopHood() {
    return run(() -> hoodIO.setVoltage(0));
  }

  public Command setShooterVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public Command stopShooter() {
    return run(() -> shooterIO.setVoltage(0));
  }

  public Command setShooterVelocity(double velocityMetresPerSecond) {
    return run(() -> shooterIO.setVelocity(velocityMetresPerSecond));
  }

  public Command setHoodedShooterPositionAndVelocity(
      double positionDegrees, double velocityMetresPerSecond) {
    return run(
        () -> {
          hoodIO.setPosition(positionDegrees);
          shooterIO.setVelocity(velocityMetresPerSecond);
        });
  }

  public Command stopHoodedShooter() {
    return run(
        () -> {
          hoodIO.setVoltage(0);
          shooterIO.setVelocity(0);
        });
  }
}
