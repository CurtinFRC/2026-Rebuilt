package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import choreo.util.ChoreoAllianceFlipUtil;
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
import org.curtinfrc.frc2026.drive.Drive;
import org.curtinfrc.frc2026.sim.BallSim;
import org.curtinfrc.frc2026.util.FieldConstants;
import org.curtinfrc.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final double MOTOR_WARNING_TEMP = 60;
  public static final double WHEEL_DIAMETER =
      (Constants.robotType == Constants.RobotType.COMP) ? 0.1 : 0.101;
  public static final int SHOOTER_MOTOR_NUMBER =
      (Constants.robotType == Constants.RobotType.COMP) ? 4 : 4;
  public static final int HOOD_MOTOR_NUMBER =
      (Constants.robotType == Constants.RobotType.COMP) ? 2 : 1;
  public static final Translation2d HUB_LOCATION =
      ChoreoAllianceFlipUtil.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d());

  public static final double READY_SHOOTER_VELOCITY_TOLERANCE = 1.0;
  public static final double READY_HOOD_POSITION_TOLERANCE = 1.0;
  public static final double READY_ROBOT_ROTATION_TOLERANCE = 5.0;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOTER_VELOCITY =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE_FORWARDS =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE_BACKWARDS =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_BALL_FLIGHT_TIME =
      new InterpolatingDoubleTreeMap();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private final LoggedTunableNumber tunableHoodSetpoint =
      new LoggedTunableNumber("HoodSetpoint", 90);
  private final LoggedTunableNumber tunableShooterSetpoint =
      new LoggedTunableNumber("ShooterSetpoint", 16.7);
  private double shooterTarget = 0;
  private double hoodTarget = 0;

  private final Alert[] hoodMotorDisconnectedAlerts = new Alert[HOOD_MOTOR_NUMBER];
  private final Alert[] hoodMotorTempAlerts = new Alert[HOOD_MOTOR_NUMBER];
  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[SHOOTER_MOTOR_NUMBER];
  private final Alert[] shooterMotorTempAlerts = new Alert[SHOOTER_MOTOR_NUMBER];

  private boolean canshoot = false;

  public final Trigger hoodedShooterReady =
      new Trigger(
              () -> {
                double hoodPosition = hoodInputs.positionRotations * 360;
                boolean hoodReady =
                    Math.abs(hoodTarget - hoodPosition) <= READY_HOOD_POSITION_TOLERANCE;
                boolean shooterReady =
                    Math.abs(shooterTarget - shooterInputs.velocityMetresPerSecond)
                        <= READY_SHOOTER_VELOCITY_TOLERANCE;
                return hoodReady && shooterReady;
              })
          .debounce(0.1)
          .and(() -> canshoot);

  private final double SHOT_SPEED = 16.5;

  public HoodedShooter(
      HoodIO hoodIO,
      ShooterIO shooterIO,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    this.shooterIO = shooterIO;
    this.hoodIO = hoodIO;
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    DISTANCE_TO_SHOOTER_VELOCITY.put(0.0, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(0.2, 90.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(-0.2, 180 - 90.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(0.0, 1.42);

    DISTANCE_TO_SHOOTER_VELOCITY.put(1.21, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(1.21, 86.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(0.81, 180 - 86.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(1.21, 1.42);

    DISTANCE_TO_SHOOTER_VELOCITY.put(1.71, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(1.71, 85.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(1.31, 180 - 85.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(1.71, 1.46);

    DISTANCE_TO_SHOOTER_VELOCITY.put(2.45, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(2.45, 77.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(2.05, 100.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(2.45, 1.31);

    DISTANCE_TO_SHOOTER_VELOCITY.put(3.1, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(3.1, 75.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(2.7, 103.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(3.1, 1.28);

    DISTANCE_TO_SHOOTER_VELOCITY.put(3.7, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(3.7, 68.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(3.3, 107.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(3.7, 1.29);

    DISTANCE_TO_SHOOTER_VELOCITY.put(4.15, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(4.15, 62.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(3.75, 109.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(4.15, 1.23);

    DISTANCE_TO_SHOOTER_VELOCITY.put(5.11, SHOT_SPEED);
    DISTANCE_TO_HOOD_ANGLE_FORWARDS.put(5.11, 60.0);
    DISTANCE_TO_HOOD_ANGLE_BACKWARDS.put(4.71, 115.0);
    DISTANCE_TO_BALL_FLIGHT_TIME.put(5.11, 1.12);

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
    Logger.recordOutput("HoodedShooter/hoodedShooterReady", hoodedShooterReady.getAsBoolean());
    Logger.recordOutput("HoodedShooter/hoodTarget", hoodTarget);
    Logger.recordOutput("HoodedShooter/shooterTarget", shooterTarget);
    Logger.recordOutput("HoodedShooter/hoodPositionDegrees", hoodInputs.positionRotations * 360);
    Logger.recordOutput(
        "HoodedShooter/distanceFromHub",
        HUB_LOCATION.minus(robotPose.get().getTranslation()).getNorm());

    for (int motor = 0; motor < hoodMotorDisconnectedAlerts.length; motor++) {
      hoodMotorDisconnectedAlerts[motor].set(!hoodInputs.motorsConnected[motor]);
      hoodMotorTempAlerts[motor].set(hoodInputs.motorTemperatures[motor] > MOTOR_WARNING_TEMP);
    }
    for (int motor = 0; motor < shooterMotorDisconnectedAlerts.length; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(
          shooterInputs.motorTemperatures[motor] > MOTOR_WARNING_TEMP);
    }
  }

  public Translation2d getVirtualTargetLocation(Supplier<Translation2d> location) {
    double realDistanceLength = location.get().minus(robotPose.get().getTranslation()).getNorm();
    Translation2d robotVel =
        new Translation2d(
                robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond)
            .times(0.8);
    double robotAngle = robotPose.get().getRotation().rotateBy(Rotation2d.k180deg).getRadians();
    robotVel =
        Math.abs(robotAngle) < Rotation2d.kCCW_90deg.getRadians() ? robotVel : robotVel.times(-1);
    double airTime = DISTANCE_TO_BALL_FLIGHT_TIME.get(realDistanceLength);

    Translation2d hubCompensationOffset = robotVel.times(airTime);
    Translation2d compensatedHubLocation = location.get().plus(hubCompensationOffset);
    Logger.recordOutput(
        "HoodedShooter/compensatedLocation", new Pose2d(compensatedHubLocation, Rotation2d.kZero));
    return (realDistanceLength > 1) ? compensatedHubLocation : location.get();
  }

  public Command shootAtTarget(Supplier<Translation2d> shotLocation) {
    return run(
        () -> {
          Supplier<Translation2d> flippedTarget =
              () ->
                  ChoreoAllianceFlipUtil.shouldFlip()
                      ? ChoreoAllianceFlipUtil.flip(shotLocation.get())
                      : shotLocation.get();
          Translation2d compensatedHubLocation = getVirtualTargetLocation(flippedTarget);

          double compensatedDistanceLength =
              compensatedHubLocation.minus(robotPose.get().getTranslation()).getNorm();

          Logger.recordOutput(
              "Drive/TargetLocation", new Pose2d(compensatedHubLocation, Rotation2d.kZero));

          double target =
              Drive.angleToLocation(this.getVirtualTargetLocation(shotLocation), robotPose.get());

          double robotAngle = robotPose.get().getRotation().getRadians();
          // if ((robotPose.get().getX() > FieldConstants.fieldLength / 2 - 3
          //     && robotPose.get().getX() < FieldConstants.fieldLength / 2 + 3)) {
          //   robotAngle = robotPose.get().getRotation().rotateBy(Rotation2d.k180deg).getRadians();
          // }

          if (Constants.tuningMode) {
            hoodTarget = tunableHoodSetpoint.get();
            shooterTarget = tunableShooterSetpoint.get();
          } else {
            // Adjust hood angle based on robot angle compared to hood
            Logger.recordOutput(
                "HoodedShooter/Direction",
                Math.abs(robotAngle) < Rotation2d.kCCW_90deg.getRadians());
            if (Math.abs(robotAngle) < Rotation2d.kCCW_90deg.getRadians()) {
              hoodTarget = DISTANCE_TO_HOOD_ANGLE_BACKWARDS.get(compensatedDistanceLength);
            } else {
              hoodTarget = DISTANCE_TO_HOOD_ANGLE_FORWARDS.get(compensatedDistanceLength);
            }
            shooterTarget = DISTANCE_TO_SHOOTER_VELOCITY.get(compensatedDistanceLength);
          }

          double angleDiff =
              Math.atan2(Math.sin(target - robotAngle), Math.cos(target - robotAngle));
          if (Math.toDegrees(Math.abs(angleDiff)) < READY_ROBOT_ROTATION_TOLERANCE) {
            canshoot = true;
          } else {
            canshoot = false;
            // hoodIO.setVoltage(0);
            // shooterIO.setVoltage(0);
          }

          hoodIO.setPosition(hoodTarget / 360);
          shooterIO.setVelocity(shooterTarget, hoodedShooterReady.getAsBoolean());

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

  public Command setHoodPosition(double position) {
    return run(
        () -> {
          hoodIO.setPosition(position);
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
    return run(
        () -> shooterIO.setVelocity(velocityMetresPerSecond, hoodedShooterReady.getAsBoolean()));
  }

  public Command setHoodedShooterPositionAndVelocity(
      double position, double velocityMetresPerSecond) {
    return run(
        () -> {
          hoodIO.setPosition(position);
          shooterIO.setVelocity(velocityMetresPerSecond, hoodedShooterReady.getAsBoolean());
        });
  }

  public Command stopHoodedShooter() {
    return run(
        () -> {
          hoodIO.setVoltage(0);
          shooterIO.setVelocity(0, false);
        });
  }
}
