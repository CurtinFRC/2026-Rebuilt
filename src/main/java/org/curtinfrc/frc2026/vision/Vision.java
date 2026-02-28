package org.curtinfrc.frc2026.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.Constants.RobotType;
import org.curtinfrc.frc2026.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.common.hardware.VisionLEDMode;

public class Vision extends VirtualSubsystem {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static record CameraConfig(String name, Transform3d robotToCamera, double stdDev) {}

  public static CameraConfig[] devCameraConfigs =
      new CameraConfig[] {
        new CameraConfig(
            "Intake Left",
            new Transform3d(
                new Translation3d(0.293052, 0.2893110, 0.445512),
                new Rotation3d(0, -0.20944, 0.436332)),
            2.0),
        new CameraConfig(
            "Intake Right",
            new Transform3d(
                new Translation3d(0.293302, -0.289311, 0.445512),
                new Rotation3d(0, -0.20944, -0.436332)),
            1.0),
        new CameraConfig(
            "Shooter Right",
            new Transform3d(
                new Translation3d(-0.292205, 0.474879, 0.041151),
                new Rotation3d(0, 0.20944, 2.61799)),
            1.0),
        new CameraConfig(
            "PC_Camera",
            new Transform3d(
                new Translation3d(0.291955, 0.474879, -0.041151), new Rotation3d(0, 40, 135)),
            1.0),
      };

  public static CameraConfig[] compCameraConfigs =
      new CameraConfig[] {
        // todo
        new CameraConfig(
            "Intake Left",
            new Transform3d(
                new Translation3d(0.195339, -0.097254, 0.47759844),
                new Rotation3d(0, -0.270526034, -0.174533)),
            1.0),
        // todo
        new CameraConfig(
            "Intake Right",
            new Transform3d(
                new Translation3d(-0.09253957, -0.14919102, 0.47622083),
                new Rotation3d(0, -0.2530727, 1.48352986)),
            1.0),
        new CameraConfig(
            "Shooter Left",
            new Transform3d(
                new Translation3d(0.21424577, -0.27179598, 0.36319361),
                new Rotation3d(0, -0.388662903083, 0.174533)),
            1.0),
        new CameraConfig(
            "Shooter Right",
            new Transform3d(
                new Translation3d(0.21503940, 0.28474389, 0.36355173),
                new Rotation3d(0, -0.388662903083, -0.174533)),
            1.0),
      };

  private static double maxAmbiguity = 0.2;
  private static double maxZError = 0.4;

  private static double linearStdDevBaseline = 0.03;
  private static double angularStdDevBaseline = 0.1;

  private final PoseEstimateConsumer consumer;
  private final Supplier<Rotation2d> gyro;
  private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
      TimeInterpolatableBuffer.createBuffer(1.0);
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(PoseEstimateConsumer consumer, Supplier<Rotation2d> gyro, VisionIO... io) {
    this.consumer = consumer;
    this.gyro = gyro;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  public void setLEDMode(VisionLEDMode mode) {
    for (var i : io) {
      i.setLEDMode(mode);
    }
  }

  @Override
  public void periodic() {
    // Update heading data
    headingBuffer.addSample(RobotController.getTime(), gyro.get());

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Boolean> rejectPoseResults = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        var rotation = headingBuffer.getSample(observation.timestamp()).get();
        var pose = observation.pose();
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || Math.abs(pose.getZ()) > maxZError // Must have realistic Z coordinate
                // Must be within the field boundaries
                || pose.getX() < 0.0
                || pose.getX() > aprilTagLayout.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > aprilTagLayout.getFieldWidth()
                // If the measurement is high ambiguity check it against the gyro
                || ((observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                    && !(Math.abs(pose.getRotation().toRotation2d().minus(rotation).getDegrees())
                            < 5
                        && Math.signum(rotation.getDegrees())
                            == Math.signum(pose.toPose2d().getRotation().getDegrees())));

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (Constants.robotType == RobotType.DEV) {
          if (cameraIndex < devCameraConfigs.length) {
            linearStdDev *= devCameraConfigs[cameraIndex].stdDev;
            angularStdDev *= devCameraConfigs[cameraIndex].stdDev;
          }
        } else {
          if (cameraIndex < compCameraConfigs.length) {
            linearStdDev *= compCameraConfigs[cameraIndex].stdDev;
            angularStdDev *= compCameraConfigs[cameraIndex].stdDev;
          }
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface PoseEstimateConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
