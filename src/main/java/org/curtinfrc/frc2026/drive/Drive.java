package org.curtinfrc.frc2026.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.Constants.Mode;
import org.curtinfrc.frc2026.util.Repulsor.DriveRepulsor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends DriveRepulsor {
  private final PIDController xController = new PIDController(10, 0.0, 0.0);
  private final PIDController yController = new PIDController(10, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private static final double ANGLE_MAX_ACCELERATION = 12.608 / DRIVE_BASE_RADIUS - 0.5;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Setting PID values for turning towards Hub.
  public static final double hubHeadingKP = 10;
  public static final double hubHeadingKI = 0;
  private static final double hubHeadingKD = 0;

  // Creating a new instance of the class PIDController and plugging in PID values from variables
  // above.
  ProfiledPIDController hubHeadingController =
      new ProfiledPIDController(
          hubHeadingKP,
          hubHeadingKI,
          hubHeadingKD,
          new TrapezoidProfile.Constraints(
              getMaxAngularSpeedRadPerSec() - 0.5, ANGLE_MAX_ACCELERATION));

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public PIDController getOmegaPID() {
    return headingController;
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // setting minimum and maximum angle in values in radians.
    hubHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // setting error tolerance when turning to an angle and the speed tolerance
    hubHeadingController.setTolerance(0.02, 0.1);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void followTrajectory(SwerveSample sample) {

    Pose2d pose = getPose();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            -sample.vx - xController.calculate(pose.getX(), sample.x),
            -sample.vy - yController.calculate(pose.getY(), sample.y),
            -sample.omega
                - headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    Logger.recordOutput("Odometry/target", sample.getPose());
    runVelocity(speeds);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Stops the drive. */
  private void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocit
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = omegaSupplier.getAsDouble();

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                  omega * getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
        });
  }

  public static double angleToLocation(Translation2d locationTransform, Pose2d currentPosition) {
    Pose2d locationPose =
        new Pose2d(locationTransform.getX(), locationTransform.getY(), Rotation2d.kZero);

    double targetAngle =
        locationPose
            .getTranslation()
            .minus(currentPosition.getTranslation())
            .getAngle()
            .getRadians();

    return targetAngle;
  }

  double targetAngle = 0;

  public Command faceLocation(Supplier<Translation2d> locationSupplier) {
    return run(
        () -> {
          Translation2d locationTransform = locationSupplier.get();
          // Get current position using the getPose method.
          Pose2d currentPosition = getPose();

          // Plugging in target angle to target position/pose
          Pose2d target =
              new Pose2d(
                  currentPosition.getX(),
                  currentPosition.getY(),
                  new Rotation2d(angleToLocation(locationTransform, currentPosition)));

          // Getting robot current angle in radians
          double robotAngle = currentPosition.getRotation().getRadians();

          // Getting optimal angle speed by providing the current robot angle and the angle we want
          // to go to
          double angleSpeed =
              hubHeadingController.calculate(
                  robotAngle, angleToLocation(locationTransform, currentPosition));
          targetAngle = angleToLocation(locationTransform, currentPosition);
          Trigger withinAngleRange =
              new Trigger(
                  () -> {
                    return (Math.abs(targetAngle) - robotAngle < 5);
                  });

          Logger.recordOutput("Target Angle", angleToLocation(locationTransform, currentPosition));
          Logger.recordOutput("Robot Angle", robotAngle);
          Logger.recordOutput("targetPose", target);

          // Creating a new instance of the class ChassisSpeeds and plugging in angleSpeed and
          // setting vxMetersPerSecond to 0 and vyMetersPerSecond to 0 (because we are turning)
          ChassisSpeeds speed = new ChassisSpeeds(0, 0, angleSpeed);

          // Running it
          runVelocity(speed);
        });
  }

  public Command locationHeadingjoyStickDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier aligningSupplier,
      Supplier<Translation2d> locationTransform) {
    return runOnce(() -> hubHeadingController.reset(getRotation().getRadians()))
        .andThen(
            run(
                () -> {

                  // Get current position using the getPose method.
                  Pose2d currentPosition = getPose();

                  // targetAngle = Math.toDegrees(targetAngle);

                  // Getting robot current angle in radians
                  double robotAngle = currentPosition.getRotation().getRadians();

                  double angleToHub = angleToLocation(locationTransform.get(), currentPosition);
                  Logger.recordOutput("Aiming Target", locationTransform.get());
                  // if (Math.abs(robotAngle) > Rotation2d.kCCW_90deg.getRadians()) {
                  //   angleToHub =
                  //       new Rotation2d(angleToHub).rotateBy(Rotation2d.k180deg).getRadians();
                  // }
                  double angleSpeed = hubHeadingController.calculate(robotAngle, angleToHub);

                  Logger.recordOutput("TargetAngle", angleToHub);
                  Logger.recordOutput("RobotAngle", robotAngle);

                  // Apply rotation deadband
                  double omega = rotationSupplier.getAsDouble();

                  // Square rotation value for more precise control
                  omega = Math.copySign(omega * omega, omega);

                  // Get linear velocity
                  Translation2d linearVelocity =
                      getLinearVelocityFromJoysticks(
                          xSupplier.getAsDouble(), ySupplier.getAsDouble());

                  // Convert to field relative speeds & send command
                  boolean isFlipped =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red;
                  boolean rotating = Math.abs(rotationSupplier.getAsDouble()) > 0.005;
                  if (!aligningSupplier.getAsBoolean() || rotating) {
                    ChassisSpeeds speeds =
                        new ChassisSpeeds(
                            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                            omega * getMaxAngularSpeedRadPerSec());
                    runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? getRotation().plus(new Rotation2d(Math.PI))
                                : getRotation()));
                  } else {
                    ChassisSpeeds speeds =
                        new ChassisSpeeds(
                            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                            -angleSpeed);
                    runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? getRotation().plus(new Rotation2d(Math.PI))
                                : getRotation()));
                  }
                }));
  }

  public Command prepareAutonomous(double x, double y) {
    return run(() -> {
          double omega = 0.0;

          ChassisSpeeds leftAutoSpeeds =
              new ChassisSpeeds(
                  xController.calculate(getPose().getX(), x) * getMaxLinearSpeedMetersPerSec(),
                  yController.calculate(getPose().getY(), y) * getMaxLinearSpeedMetersPerSec(),
                  omega * getMaxAngularSpeedRadPerSec());

          runVelocity(leftAutoSpeeds);
        })
        .until(
            () -> {
              double xPose = getPose().getX();
              double yPose = getPose().getY();
              return Math.abs(xPose - x) < 0.1 && Math.abs(yPose - y) < 0.1;
            });
  }

  public Rotation2d Goal = Rotation2d.kZero;

  public Command stopCmd() {
    return runOnce(() -> stop());
  }

  public Command TrenchAlign(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return runOnce(
            () -> {
              Pose2d currentPosition = getPose();
              double robotAngle = currentPosition.getRotation().getRadians();
              Logger.recordOutput("Robot Angle", robotAngle);
              hubHeadingController.reset(robotAngle);
              if (robotAngle >= -Math.PI / 2 && robotAngle <= Math.PI / 2) {
                Goal = Rotation2d.kZero;
              } else {
                Goal = Rotation2d.k180deg;
              }
            })
        .andThen(
            run(
                () -> {
                  Pose2d currentPosition = getPose();
                  double robotAngle =
                      MathUtil.angleModulus(currentPosition.getRotation().getRadians());
                  Logger.recordOutput("Robot Angle", robotAngle);
                  double angleSpeed = hubHeadingController.calculate(robotAngle, Goal.getRadians());
                  Logger.recordOutput("Angle Speed", angleSpeed);
                  Logger.recordOutput("Goal", Goal);

                  Translation2d linearVelocity =
                      getLinearVelocityFromJoysticks(
                          xSupplier.getAsDouble(), ySupplier.getAsDouble());

                  ChassisSpeeds speeds =
                      new ChassisSpeeds(
                          linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                          linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                          -angleSpeed);
                  runVelocity(speeds);
                }));
  }
}
