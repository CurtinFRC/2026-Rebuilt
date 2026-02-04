package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.sim.BallSim;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final Translation2d HUB_LOCATION = new Translation2d(12, 4);
  public static final double WHEEL_DIAMETER = 0.101;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOTER_VELOCITY =
      new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE =
      new InterpolatingDoubleTreeMap();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;

  private final Alert hoodMotorDisconnectedAlert;
  private final Alert hoodMotorTempAlert;
  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[3];
  private final Alert[] shooterMotorTempAlerts = new Alert[3];

  private final MutVoltage appliedVoltageMut = Volts.mutable(0);
  private final MutAngle angleRadiansMut = Radians.mutable(0);
  private final MutAngularVelocity angularVelocityRadiansMut = RadiansPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutineShooter;
  private final SysIdRoutine sysIdRoutineHood;

  private boolean hoodSoftLimitedForward() {
    return hoodInputs.positionRotations > HoodIODev.FORWARD_LIMIT_ROTATIONS - 0.1;
  }

  private boolean hoodSoftLimitedReverse() {
    return hoodInputs.positionRotations < HoodIODev.REVERSE_LIMIT_ROTATIONS + 0.1;
  }

  public BallSim ballSim = new BallSim(0.0, new Rotation2d(0.0), new Pose3d());

  public HoodedShooter(ShooterIO shooterIO, HoodIO hoodIO, Supplier<Pose2d> robotPose) {
    this.shooterIO = shooterIO;
    this.hoodIO = hoodIO;
    this.robotPose = robotPose;

    DISTANCE_TO_SHOOTER_VELOCITY.put(3.05, 0.0);
    DISTANCE_TO_SHOOTER_VELOCITY.put(2.035, 0.0);
    DISTANCE_TO_SHOOTER_VELOCITY.put(5.474, 0.0);
    DISTANCE_TO_SHOOTER_VELOCITY.put(4.0, 0.0);

    DISTANCE_TO_HOOD_ANGLE.put(3.05, 0.0);
    DISTANCE_TO_HOOD_ANGLE.put(2.035, 0.0);
    DISTANCE_TO_HOOD_ANGLE.put(5.474, 0.0);
    DISTANCE_TO_HOOD_ANGLE.put(4.0, 0.0);

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

    sysIdRoutineShooter =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                shooterIO::setVoltageV,
                log -> {
                  log.motor("shooter")
                      .voltage(appliedVoltageMut.mut_replace(shooterInputs.appliedVolts, Volts))
                      .angularPosition(
                          angleRadiansMut.mut_replace(shooterInputs.positionRotations, Rotations))
                      .angularVelocity(
                          angularVelocityRadiansMut.mut_replace(
                              shooterInputs.velocityMetresPerSecond
                                  / (HoodedShooter.WHEEL_DIAMETER * Math.PI),
                              RotationsPerSecond));
                },
                this,
                "shooter"));

    sysIdRoutineHood =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                hoodIO::setVoltageV,
                log -> {
                  log.motor("hood")
                      .voltage(appliedVoltageMut.mut_replace(hoodInputs.appliedVolts, Volts))
                      .angularPosition(
                          angleRadiansMut.mut_replace(hoodInputs.positionRotations, Rotations))
                      .angularVelocity(
                          angularVelocityRadiansMut.mut_replace(
                              hoodInputs.angularVelocityRotationsPerSecond, RotationsPerSecond));
                },
                this,
                "hood"));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Hood", hoodInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.recordOutput("Ball", ballSim.update(0.02));

    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);
    hoodMotorTempAlert.set(hoodInputs.motorTemperature > 60); // in celcius
    for (int motor = 0; motor < 3; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(shooterInputs.motorTemperatures[motor] > 60);
    }
  }

  public Command shootAtHub() { // this assumes that the robot is facing the target
    return run(
        () -> {
          double distanceLength = HUB_LOCATION.minus(robotPose.get().getTranslation()).getNorm();

          double hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(3.04833887);
          hoodAngle = (0 / 360);
          double shooterVelocity = DISTANCE_TO_SHOOTER_VELOCITY.get(3.04833887);
          hoodIO.setPosition(0.36);
          shooterIO.setVelocity(20);

          ballSim =
              new BallSim(
                  shooterVelocity,
                  Rotation2d.fromDegrees(89),
                  new Pose3d(robotPose.get())
                      .transformBy(new Transform3d(0.2, 0.0, 0.4, Rotation3d.kZero)));
        });
  }

  public Command setHoodPosition(double positionRotations) {
    return run(() -> hoodIO.setPosition(positionRotations));
  }

  public Command setHoodVoltage(double voltage) {
    return run(() -> hoodIO.setVoltage(voltage));
  }

  public Command setHoodVoltage(Supplier<Double> voltage) {
    return run(() -> hoodIO.setVoltage(voltage.get()));
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

  public Command shooterSysIdQuasistaticForward() {
    return sysIdRoutineShooter.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command shooterSysIdQuasistaticBackward() {
    return sysIdRoutineShooter.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command shooterSysIdDynamicForward() {
    return sysIdRoutineShooter.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command shooterSysIdDynamicBackward() {
    return sysIdRoutineShooter.dynamic(SysIdRoutine.Direction.kReverse);
  }

  // do not use
  public Command hoodSysIdQuasistaticForward() {
    return sysIdRoutineHood
        .quasistatic(SysIdRoutine.Direction.kForward)
        .until(() -> hoodSoftLimitedForward());
  }

  public Command hoodSysIdQuasistaticBackward() {
    return sysIdRoutineHood
        .quasistatic(SysIdRoutine.Direction.kReverse)
        .until(() -> hoodSoftLimitedReverse());
  }

  public Command hoodSysIdDynamicForward() {
    return sysIdRoutineHood
        .dynamic(SysIdRoutine.Direction.kForward)
        .until(() -> hoodSoftLimitedForward());
  }

  public Command hoodSysIdDynamicBackward() {
    return sysIdRoutineHood
        .dynamic(SysIdRoutine.Direction.kReverse)
        .until(() -> hoodSoftLimitedReverse());
  }
}
