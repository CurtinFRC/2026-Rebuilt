// File: src/main/java/org/curtinfrc/frc2025/util/Repulsor/Setpoints.java
package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Meters;
import static org.curtinfrc.frc2026.util.Repulsor.Constants.aprilTagLayout;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;

public class Setpoints {

  public static Alliance currentAllianceOrBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static Pose2d flipToRed(Pose2d bluePose) {
    return ChoreoAllianceFlipUtil.flip(bluePose);
  }

  public static Translation2d flipToRed(Translation2d blue) {
    return ChoreoAllianceFlipUtil.flip(new Pose2d(blue, Rotation2d.kZero)).getTranslation();
  }

  public enum SetpointType {
    kHumanPlayer,
    kScore,
    kProcessor,
    kDeepCage,
    kShallowCage,
    kOther
  }

  public record SetpointContext(
      Optional<Pose2d> robotPose,
      double robotLengthMeters,
      double robotWidthMeters,
      double coralOffsetMeters,
      double algaeOffsetMeters,
      double shooterReleaseHeightMeters,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles) {

    public static final SetpointContext EMPTY =
        new SetpointContext(Optional.empty(), 0.0, 0.0, 0.0, 0.0, 0.0, List.of());

    public SetpointContext {
      robotPose = robotPose == null ? Optional.empty() : robotPose;
      dynamicObstacles = dynamicObstacles == null ? List.of() : dynamicObstacles;
    }
  }

  public abstract static class GameSetpoint {
    private final String name;
    private final SetpointType type;

    protected GameSetpoint(String name, SetpointType type) {
      this.name = name;
      this.type = type;
    }

    public final String name() {
      return name;
    }

    public final SetpointType type() {
      return type;
    }

    public abstract Pose2d bluePose(SetpointContext ctx);

    public Pose2d redPose(SetpointContext ctx) {
      return flipToRed(bluePose(ctx));
    }

    public final Pose2d poseForAlliance(Alliance alliance, SetpointContext ctx) {
      return alliance == Alliance.Red ? redPose(ctx) : bluePose(ctx);
    }

    public final Pose2d poseForCurrentAlliance(SetpointContext ctx) {
      return poseForAlliance(currentAllianceOrBlue(), ctx);
    }

    public Pose2d approximateBluePose() {
      return bluePose(SetpointContext.EMPTY);
    }

    public Pose2d approximateRedPose() {
      return redPose(SetpointContext.EMPTY);
    }
  }

  public static final class RepulsorSetpoint {
    private final GameSetpoint point;
    private final HeightSetpoint height;

    public RepulsorSetpoint(GameSetpoint point, HeightSetpoint height) {
      if (point == null) throw new IllegalArgumentException("point cannot be null");
      this.point = point;
      this.height = height == null ? HeightSetpoint.NONE : height;
    }

    public GameSetpoint point() {
      return point;
    }

    public HeightSetpoint height() {
      return height;
    }

    public Pose2d getBlue(SetpointContext ctx) {
      return point.bluePose(ctx == null ? SetpointContext.EMPTY : ctx);
    }

    public Pose2d getRed(SetpointContext ctx) {
      return point.redPose(ctx == null ? SetpointContext.EMPTY : ctx);
    }

    public Pose2d getForAlliance(Alliance alliance, SetpointContext ctx) {
      return point.poseForAlliance(alliance == null ? Alliance.Blue : alliance, ctx);
    }

    public Pose2d get(SetpointContext ctx) {
      return point.poseForCurrentAlliance(ctx == null ? SetpointContext.EMPTY : ctx);
    }
  }

  public static Pose2d getSetPose(RepulsorSetpoint sp, SetpointContext ctx) {
    if (sp == null) return Pose2d.kZero;
    return sp.get(ctx);
  }

  public enum HeightSetpoint {
    L1(Meters.of(0.46)),
    L2(Meters.of(0.81)),
    L3(Meters.of(1.21)),
    L4(Meters.of(1.83)),
    PROCESSOR(Meters.of(0.18)),
    DEEP_CAGE(Meters.of(0.75)),
    SHALLOW_CAGE(Meters.of(0.09)),
    CORAL_STATION(Meters.of(0.95)),
    NET(Meters.of(1.93)),
    NONE(null);

    private final Distance height;

    HeightSetpoint(Distance height) {
      this.height = height;
    }

    public Distance getHeight() {
      return height;
    }
  }

  public static final class Reefscape2025 {
    private Reefscape2025() {}

    public static final GameSetpoint A = new ReefTagScoreSetpoint("A", 18, true, true);
    public static final GameSetpoint B = new ReefTagScoreSetpoint("B", 18, false, true);
    public static final GameSetpoint C = new ReefTagScoreSetpoint("C", 17, true, true);
    public static final GameSetpoint D = new ReefTagScoreSetpoint("D", 17, false, true);
    public static final GameSetpoint E = new ReefTagScoreSetpoint("E", 22, true, true);
    public static final GameSetpoint F = new ReefTagScoreSetpoint("F", 22, false, true);
    public static final GameSetpoint G = new ReefTagScoreSetpoint("G", 21, true, true);
    public static final GameSetpoint H = new ReefTagScoreSetpoint("H", 21, false, true);
    public static final GameSetpoint I = new ReefTagScoreSetpoint("I", 20, true, true);
    public static final GameSetpoint J = new ReefTagScoreSetpoint("J", 20, false, true);
    public static final GameSetpoint K = new ReefTagScoreSetpoint("K", 19, true, true);
    public static final GameSetpoint L = new ReefTagScoreSetpoint("L", 19, false, true);

    public static final GameSetpoint CLOSE = new ReefTagScoreSetpoint("CLOSE", 18, true, false);
    public static final GameSetpoint CLOSE_LEFT =
        new ReefTagScoreSetpoint("CLOSE_LEFT", 19, true, false);
    public static final GameSetpoint CLOSE_RIGHT =
        new ReefTagScoreSetpoint("CLOSE_RIGHT", 17, true, false);
    public static final GameSetpoint FAR_RIGHT =
        new ReefTagScoreSetpoint("FAR_RIGHT", 22, true, false);
    public static final GameSetpoint FAR_LEFT =
        new ReefTagScoreSetpoint("FAR_LEFT", 20, true, false);
    public static final GameSetpoint FAR = new ReefTagScoreSetpoint("FAR", 21, true, false);

    public static final GameSetpoint LEFT_HP =
        new StaticPoseSetpoint(
            "LEFT_HP",
            SetpointType.kHumanPlayer,
            new Pose2d(
                1.148711085319519, 7.199769020080566, Rotation2d.fromDegrees(125.989 + 180)));

    public static final GameSetpoint RIGHT_HP =
        new StaticPoseSetpoint(
            "RIGHT_HP",
            SetpointType.kHumanPlayer,
            new Pose2d(
                0.9220133423805237,
                0.9964936375617981,
                Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

    private static final class StaticPoseSetpoint extends GameSetpoint {
      private final Pose2d bluePose;

      StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
        super(name, type);
        this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
      }

      @Override
      public Pose2d bluePose(SetpointContext ctx) {
        return bluePose;
      }
    }

    private static final class ReefTagScoreSetpoint extends GameSetpoint {
      private final int tagID;
      private final boolean isLeft;
      private final boolean isCoral;

      ReefTagScoreSetpoint(String name, int tagID, boolean isLeft, boolean isCoral) {
        super(name, SetpointType.kScore);
        this.tagID = tagID;
        this.isLeft = isLeft;
        this.isCoral = isCoral;
      }

      @Override
      public Pose2d bluePose(SetpointContext ctx) {
        var tagPose3dOpt = aprilTagLayout.getTagPose(tagID);
        if (tagPose3dOpt.isEmpty()) return Pose2d.kZero;

        Pose2d tag2d = tagPose3dOpt.get().toPose2d();

        double offset = isCoral ? ctx.coralOffsetMeters() : ctx.algaeOffsetMeters();
        Pose2d mappedPose = standoffFromTag(tag2d, ctx.robotLengthMeters(), ctx.robotWidthMeters());

        double yaw = mappedPose.getRotation().getRadians();
        double xOffset = offset * Math.sin(yaw);
        double yOffset = offset * Math.cos(yaw);

        Pose2d result;
        if (isLeft) {
          result =
              new Pose2d(
                  mappedPose.getX() + xOffset,
                  mappedPose.getY() - yOffset,
                  mappedPose.getRotation().plus(Rotation2d.kPi));
        } else {
          result =
              new Pose2d(
                  mappedPose.getX() - xOffset,
                  mappedPose.getY() + yOffset,
                  mappedPose.getRotation().plus(Rotation2d.kPi));
        }
        return result;
      }

      @Override
      public Pose2d approximateBluePose() {
        var tagPose3dOpt = aprilTagLayout.getTagPose(tagID);
        if (tagPose3dOpt.isEmpty()) return Pose2d.kZero;
        Pose2d tag2d = tagPose3dOpt.get().toPose2d();
        return new Pose2d(tag2d.getTranslation(), tag2d.getRotation().plus(Rotation2d.kPi));
      }

      private static Pose2d standoffFromTag(Pose2d tagPose, double robotLength, double robotWidth) {
        if (robotLength <= 0.0 && robotWidth <= 0.0) {
          return tagPose;
        }
        double halfL = Math.max(0.0, robotLength) / 2.0;
        double halfW = Math.max(0.0, robotWidth) / 2.0;

        double standoff = Math.hypot(halfL, halfW);
        Translation2d shifted =
            tagPose.getTranslation().plus(new Translation2d(standoff, tagPose.getRotation()));
        return new Pose2d(shifted, tagPose.getRotation());
      }
    }
  }

  public static final class Rebuilt2026 {
    private Rebuilt2026() {}

    public static final double HUB_OPENING_FRONT_EDGE_HEIGHT_M = 1.83;
    public static final double HUB_FACE_TO_AIMPOINT_METERS = 0.60;
    public static final String GAME_PIECE_ID_FUEL = "fuel";

    public static final DragShotPlanner.Constraints HUB_SHOT_CONSTRAINTS =
        new DragShotPlanner.Constraints(
            6.0, 28.0, 10.0, 75.0, DragShotPlanner.Constraints.ShotStyle.ARC);

    public static final int BLUE_HUB_ANCHOR_TAG_ID = 3;
    public static final int BLUE_OUTPOST_ANCHOR_TAG_ID = 13;

    public static final GameSetpoint HUB_SHOOT =
        new HubShootSetpoint("HUB_SHOOT", BLUE_HUB_ANCHOR_TAG_ID);

    public static final GameSetpoint CENTER_COLLECT =
        new StaticPoseSetpoint(
            "CENTER_COLLECT",
            SetpointType.kOther,
            new Pose2d(
                Constants.FIELD_LENGTH / 2.0, Constants.FIELD_WIDTH / 2.0, Rotation2d.kZero));

    public static final GameSetpoint OUTPOST_COLLECT =
        new ApproachFromTagSetpoint(
            "OUTPOST_COLLECT", SetpointType.kHumanPlayer, BLUE_OUTPOST_ANCHOR_TAG_ID, 1.25);

    private static final class StaticPoseSetpoint extends GameSetpoint {
      private final Pose2d bluePose;

      StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
        super(name, type);
        this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
      }

      @Override
      public Pose2d bluePose(SetpointContext ctx) {
        return bluePose;
      }
    }

    private static final class ApproachFromTagSetpoint extends GameSetpoint {
      private final int tagId;
      private final double standoffMeters;

      ApproachFromTagSetpoint(String name, SetpointType type, int tagId, double standoffMeters) {
        super(name, type);
        this.tagId = tagId;
        this.standoffMeters = standoffMeters;
      }

      @Override
      public Pose2d bluePose(SetpointContext ctx) {
        Optional<Pose3d> pose3d = aprilTagLayout.getTagPose(tagId);
        if (pose3d.isEmpty()) return Pose2d.kZero;

        Pose2d tag2d = pose3d.get().toPose2d();
        Translation2d approachPos =
            tag2d.getTranslation().plus(new Translation2d(standoffMeters, tag2d.getRotation()));
        Rotation2d faceTag = tag2d.getRotation().plus(Rotation2d.kPi);
        return new Pose2d(approachPos, faceTag);
      }
    }

    private static final class HubShootSetpoint extends GameSetpoint {
      private final int blueHubAnchorTagId;

      HubShootSetpoint(String name, int blueHubAnchorTagId) {
        super(name, SetpointType.kScore);
        this.blueHubAnchorTagId = blueHubAnchorTagId;
      }

      @Override
      public Pose2d bluePose(SetpointContext ctx) {
        Translation2d target = hubAimpointFromAnchorTagBlue(blueHubAnchorTagId);
        return computeShootPose(ctx, target);
      }

      @Override
      public Pose2d redPose(SetpointContext ctx) {
        Translation2d blueTarget = hubAimpointFromAnchorTagBlue(blueHubAnchorTagId);
        Translation2d redTarget = flipToRed(blueTarget);
        return computeShootPose(ctx, redTarget);
      }

      @Override
      public Pose2d approximateBluePose() {
        Translation2d target = hubAimpointFromAnchorTagBlue(blueHubAnchorTagId);
        Translation2d shooter = target.minus(new Translation2d(3.0, Rotation2d.kZero));
        Rotation2d yaw = target.minus(shooter).getAngle();
        return new Pose2d(shooter, yaw);
      }

      private static Pose2d computeShootPose(SetpointContext ctx, Translation2d targetFieldPos) {
        if (ctx == null || ctx.robotPose().isEmpty()) {
          Translation2d shooter = targetFieldPos.minus(new Translation2d(3.0, Rotation2d.kZero));
          Rotation2d yaw = targetFieldPos.minus(shooter).getAngle();
          return new Pose2d(shooter, yaw);
        }

        Pose2d robotPose = ctx.robotPose().get();

        final DragShotPlanner.GamePiecePhysics physics;
        try {
          physics = DragShotPlanner.loadGamePieceFromDeployYaml(GAME_PIECE_ID_FUEL);
        } catch (Exception ex) {
          Translation2d shooter = targetFieldPos.minus(new Translation2d(3.0, Rotation2d.kZero));
          Rotation2d yaw = targetFieldPos.minus(shooter).getAngle();
          return new Pose2d(shooter, yaw);
        }

        double releaseH = Math.max(0.0, ctx.shooterReleaseHeightMeters());
        double halfL = Math.max(0.0, ctx.robotLengthMeters()) / 2.0;
        double halfW = Math.max(0.0, ctx.robotWidthMeters()) / 2.0;

        Optional<DragShotPlanner.ShotSolution> sol =
            DragShotPlanner.findBestShotAuto(
                physics,
                targetFieldPos,
                HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                robotPose,
                releaseH,
                halfL,
                halfW,
                ctx.dynamicObstacles(),
                HUB_SHOT_CONSTRAINTS);

        if (sol.isEmpty()) {
          Translation2d robotPos = robotPose.getTranslation();
          Translation2d dir = robotPos.minus(targetFieldPos);
          if (dir.getNorm() < 1e-6) dir = new Translation2d(1.0, 0.0);
          Translation2d shooter = targetFieldPos.plus(dir.div(dir.getNorm()).times(3.0));
          Rotation2d yaw = targetFieldPos.minus(shooter).getAngle();
          return new Pose2d(shooter, yaw);
        }

        Translation2d shooterPos = sol.get().shooterPosition();
        Rotation2d yaw = targetFieldPos.minus(shooterPos).getAngle();
        return new Pose2d(shooterPos, yaw);
      }

      private static Translation2d hubAimpointFromAnchorTagBlue(int anchorTagId) {
        Optional<Pose3d> pose3d = aprilTagLayout.getTagPose(anchorTagId);
        if (pose3d.isEmpty()) {
          return new Translation2d(Constants.FIELD_LENGTH / 2.0, Constants.FIELD_WIDTH / 2.0);
        }

        Pose2d tag2d = pose3d.get().toPose2d();
        Rotation2d intoHub = tag2d.getRotation().plus(Rotation2d.kPi);
        return tag2d.getTranslation().plus(new Translation2d(HUB_FACE_TO_AIMPOINT_METERS, intoHub));
      }
    }
  }
}
