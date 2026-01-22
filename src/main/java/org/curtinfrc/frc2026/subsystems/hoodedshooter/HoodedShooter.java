package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.sim.BallSim;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final Translation2d HUB_LOCATION = new Translation2d(12, 4);
  public static final double HUB_HEIGHT = 1.83;
  public static final double AIM_HEIGHT_OFFSET = 0.3; // Offset above the goal for the ball to fall
  public static final double HUB_AIMED_HEIGHT = HUB_HEIGHT + AIM_HEIGHT_OFFSET;

  public static final double WHEEL_DIAMETER = 0.101;
  public static final double SHOOTER_TARGET_VELOCITY = 15;
  public static final Transform3d SHOOTER_TRANSFORM =
      new Transform3d(0, 0, 1, new Rotation3d()); // Not confirmed

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_OPTIMAL_VELOCITY =
      new InterpolatingDoubleTreeMap();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;

  public BallSim ballSim = new BallSim(0.0, new Rotation2d(0.0), new Pose3d());

  public HoodedShooter(ShooterIO shooterIO, HoodIO hoodIO, Supplier<Pose2d> robotPose) {
    this.shooterIO = shooterIO;
    this.hoodIO = hoodIO;
    this.robotPose = robotPose;

    // optimal horizontal velocities (to be tuned)
    DISTANCE_TO_OPTIMAL_VELOCITY.put(0.0, 0.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(1.0, 1.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(2.0, 2.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(3.0, 3.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(4.0, 4.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(5.0, 5.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(6.0, 6.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(7.0, 7.0);
    DISTANCE_TO_OPTIMAL_VELOCITY.put(8.0, 8.0);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Hood", hoodInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.recordOutput("Ball", ballSim.update(0.02));
  }

  public double calculateHoodRotations() {
    Pose3d shooterPose = new Pose3d(robotPose.get()).transformBy(SHOOTER_TRANSFORM);
    double distanceLength =
        Math.sqrt(
            Math.pow((HUB_LOCATION.getX() - shooterPose.getX()), 2)
                + Math.pow(
                    (HUB_LOCATION.getY() - shooterPose.getY()),
                    2)); // Distance from the robot to the bottom of the goal
    Translation2d targetVector = new Translation2d(distanceLength, HUB_AIMED_HEIGHT);

    double initialVelocity = DISTANCE_TO_OPTIMAL_VELOCITY.get(targetVector.getNorm());
    Translation2d shotVector = targetVector.div(distanceLength).times(initialVelocity);

    double velocityHorizontal = shotVector.getNorm();
    double ratioSpeed = Math.min((velocityHorizontal / SHOOTER_TARGET_VELOCITY), 1.0);
    double hoodAngle = Math.toDegrees(Math.acos(ratioSpeed));

    return hoodAngle / 360;
  }

  public double calculateHoodRotationsWithTrajectoryEquation() {
    Pose3d shooterPose = new Pose3d(robotPose.get()).transformBy(SHOOTER_TRANSFORM);
    double distanceLength =
        Math.sqrt(
            Math.pow((HUB_LOCATION.getX() - shooterPose.getX()), 2)
                + Math.pow(
                    (HUB_LOCATION.getY() - shooterPose.getY()),
                    2)); // Distance from the robot to the bottom of the goal
    double term =
        Math.pow(SHOOTER_TARGET_VELOCITY, 4)
            - 9.8
                * (9.8 * Math.pow(distanceLength, 2)
                    + 2
                        * (HUB_AIMED_HEIGHT - shooterPose.getZ())
                        * Math.pow(SHOOTER_TARGET_VELOCITY, 2));

    double hoodAngle =
        Math.toDegrees(
            Math.atan2(
                Math.pow(SHOOTER_TARGET_VELOCITY, 2) + Math.sqrt(term), 9.8 * distanceLength));

    return hoodAngle / 360;
  }

  public Command aimAtHub() { // this assumes that the robot is facing the target
    return run(
        () -> {
          double hoodAngle = calculateHoodRotations();
          hoodIO.setPosition(hoodAngle);
        });
  }

  public Command shoot() {
    return run(
        () -> {
          shooterIO.setVelocity(SHOOTER_TARGET_VELOCITY);
          ballSim =
              new BallSim(
                  SHOOTER_TARGET_VELOCITY,
                  new Rotation2d(Math.toRadians(hoodInputs.positionRotations * 360)),
                  new Pose3d(robotPose.get()).transformBy(SHOOTER_TRANSFORM));
        });
  }

  public Command setHoodPosition(double positionRotations) {
    return run(() -> hoodIO.setPosition(positionRotations));
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
      double positionRotations, double velocityMetresPerSecond) {
    return run(
        () -> {
          hoodIO.setPosition(positionRotations);
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
