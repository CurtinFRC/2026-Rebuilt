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
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.littletonrobotics.junction.Logger;

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

  public static final class MutablePoseSetpoint extends GameSetpoint {
    private final AtomicReference<Pose2d> bluePoseRef;

    public MutablePoseSetpoint(
        String name, SetpointType type, AtomicReference<Pose2d> bluePoseRef) {
      super(name, type == null ? SetpointType.kOther : type);
      this.bluePoseRef = bluePoseRef == null ? new AtomicReference<>(Pose2d.kZero) : bluePoseRef;
    }

    @Override
    public Pose2d bluePose(SetpointContext ctx) {
      Pose2d p = bluePoseRef.get();
      return p != null ? p : Pose2d.kZero;
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

        if (isLeft) {
          return new Pose2d(
              mappedPose.getX() + xOffset,
              mappedPose.getY() - yOffset,
              mappedPose.getRotation().plus(Rotation2d.kPi));
        }
        return new Pose2d(
            mappedPose.getX() - xOffset,
            mappedPose.getY() + yOffset,
            mappedPose.getRotation().plus(Rotation2d.kPi));
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
            6.0, 26.0, 25.0, 80.0, DragShotPlanner.Constraints.ShotStyle.ARC);

    public static final int BLUE_HUB_ANCHOR_TAG_ID = 20;
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

    private static final ConcurrentHashMap<Integer, Translation2d> HUB_AIMPOINT_BLUE_CACHE =
        new ConcurrentHashMap<>();

    public static Translation2d hubAimpointBlue() {
      return HUB_AIMPOINT_BLUE_CACHE.computeIfAbsent(
          BLUE_HUB_ANCHOR_TAG_ID, Rebuilt2026::hubAimpointFromAnchorTagBlue);
    }

    public static Translation2d hubAimpointForAlliance(Alliance alliance) {
      Translation2d blue = hubAimpointBlue();
      return alliance == Alliance.Red ? flipToRed(blue) : blue;
    }

    public static void onShootArcTick(Pose2d robotPose, Pose2d shootingPose, double progress01) {}

    private record HubShotCacheKey(
        int targetXmm, int targetYmm, int releaseHmm, int halfLmm, int halfWmm) {}

    private static final ConcurrentHashMap<HubShotCacheKey, HubShotLibraryState> HUB_LIB_CACHE =
        new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<HubShotCacheKey, DragShotPlanner.OnlineSearchState>
        HUB_ONLINE_STATE = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<HubShotCacheKey, HubLastPoseCache> HUB_LAST_POSE =
        new ConcurrentHashMap<>();

    private static final AtomicBoolean HUB_BG_STARTED = new AtomicBoolean(false);
    private static ScheduledExecutorService HUB_BG_EXEC;

    private static final class HubLastPoseCache {
      volatile int robotXmm;
      volatile int robotYmm;
      volatile int obsHash;
      volatile long tNs;
      volatile Pose2d pose;
    }

    private static final class HubShotLibraryState {
      final HubShotCacheKey key;
      final DragShotPlanner.GamePiecePhysics physics;
      final Translation2d targetFieldPos;
      final double releaseH;
      final double halfL;
      final double halfW;

      volatile DragShotPlanner.ShotLibrary published;
      volatile boolean done;
      final Object lock = new Object();
      DragShotPlanner.ShotLibraryBuilder builder;

      HubShotLibraryState(
          HubShotCacheKey key,
          DragShotPlanner.GamePiecePhysics physics,
          Translation2d targetFieldPos,
          double releaseH,
          double halfL,
          double halfW) {
        this.key = key;
        this.physics = physics;
        this.targetFieldPos = targetFieldPos;
        this.releaseH = releaseH;
        this.halfL = halfL;
        this.halfW = halfW;
        this.published = null;
        this.done = false;
        this.builder = null;
      }
    }

    private static void ensureHubBackground() {
      if (HUB_BG_STARTED.get()) {
        return;
      }
      if (!HUB_BG_STARTED.compareAndSet(false, true)) {
        return;
      }
      ThreadFactory tf =
          r -> {
            Thread t = new Thread(r, "HubShotPrecompute");
            t.setDaemon(true);
            t.setPriority(Thread.NORM_PRIORITY - 1);
            return t;
          };
      HUB_BG_EXEC = Executors.newSingleThreadScheduledExecutor(tf);
      HUB_BG_EXEC.scheduleAtFixedRate(
          Rebuilt2026::hubBackgroundTick, 0L, 20L, TimeUnit.MILLISECONDS);
    }

    private static void hubBackgroundTick() {
      try {
        long budgetNanos = 2_000_000L;
        long start = System.nanoTime();
        for (HubShotLibraryState st : HUB_LIB_CACHE.values()) {
          if ((System.nanoTime() - start) >= budgetNanos) {
            break;
          }
          if (st.done) {
            continue;
          }
          DragShotPlanner.ShotLibrary publish = null;
          synchronized (st.lock) {
            if (st.done) {
              continue;
            }
            if (st.builder == null) {
              double speedStep =
                  Math.max(
                      0.18,
                      (HUB_SHOT_CONSTRAINTS.maxLaunchSpeedMetersPerSecond()
                              - HUB_SHOT_CONSTRAINTS.minLaunchSpeedMetersPerSecond())
                          / 70.0);
              double angleStep =
                  Math.max(
                      0.6,
                      (HUB_SHOT_CONSTRAINTS.maxLaunchAngleDeg()
                              - HUB_SHOT_CONSTRAINTS.minLaunchAngleDeg())
                          / 80.0);
              double radialStep = 0.30;
              double bearingStep = 12.0;

              st.builder =
                  new DragShotPlanner.ShotLibraryBuilder(
                      st.physics,
                      st.targetFieldPos,
                      HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                      st.releaseH,
                      st.halfL,
                      st.halfW,
                      HUB_SHOT_CONSTRAINTS,
                      speedStep,
                      angleStep,
                      radialStep,
                      bearingStep);
            }
            publish = st.builder.maybeStep(900_000L);
            if (publish != null) {
              if (!publish.entries().isEmpty() || publish.complete()) {
                st.published = publish;
              }
              st.done = publish.complete();
            }
          }
        }
      } catch (Throwable t) {
        DriverStation.reportError("HubShot background tick crashed: " + t.getMessage(), false);
      }
    }

    private static DragShotPlanner.ShotLibrary getOrStartHubLibrary(
        DragShotPlanner.GamePiecePhysics physics,
        Translation2d targetFieldPos,
        double releaseH,
        double halfL,
        double halfW) {

      ensureHubBackground();

      HubShotCacheKey key =
          new HubShotCacheKey(
              (int) Math.round(targetFieldPos.getX() * 1000.0),
              (int) Math.round(targetFieldPos.getY() * 1000.0),
              (int) Math.round(releaseH * 1000.0),
              (int) Math.round(halfL * 1000.0),
              (int) Math.round(halfW * 1000.0));

      HubShotLibraryState st =
          HUB_LIB_CACHE.computeIfAbsent(
              key,
              k -> new HubShotLibraryState(k, physics, targetFieldPos, releaseH, halfL, halfW));

      DragShotPlanner.ShotLibrary pub = st.published;
      if (pub != null) return pub;
      return null;
    }

    private static DragShotPlanner.OnlineSearchState getOrCreateOnlineState(
        Translation2d targetFieldPos,
        double releaseH,
        double halfL,
        double halfW,
        Translation2d seed) {
      HubShotCacheKey key =
          new HubShotCacheKey(
              (int) Math.round(targetFieldPos.getX() * 1000.0),
              (int) Math.round(targetFieldPos.getY() * 1000.0),
              (int) Math.round(releaseH * 1000.0),
              (int) Math.round(halfL * 1000.0),
              (int) Math.round(halfW * 1000.0));
      return HUB_ONLINE_STATE.computeIfAbsent(
          key, k -> new DragShotPlanner.OnlineSearchState(seed, 0.60));
    }

    private static HubShotCacheKey hubKey(
        Translation2d targetFieldPos, double releaseH, double halfL, double halfW) {
      return new HubShotCacheKey(
          (int) Math.round(targetFieldPos.getX() * 1000.0),
          (int) Math.round(targetFieldPos.getY() * 1000.0),
          (int) Math.round(releaseH * 1000.0),
          (int) Math.round(halfL * 1000.0),
          (int) Math.round(halfW * 1000.0));
    }

    private static int obstaclesStableHash(List<? extends FieldPlanner.Obstacle> obs) {
      if (obs == null || obs.isEmpty()) return 0;
      int h = 1;
      for (int i = 0; i < obs.size(); i++) {
        Object o = obs.get(i);
        h = h * 31 + System.identityHashCode(o);
      }
      h = h * 31 + obs.size();
      return h;
    }

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
        Translation2d target = hubAimpointFromAnchorTagBlueCached(blueHubAnchorTagId);
        return computeShootPose(ctx, target);
      }

      @Override
      public Pose2d redPose(SetpointContext ctx) {
        Translation2d blueTarget = hubAimpointFromAnchorTagBlueCached(blueHubAnchorTagId);
        Translation2d redTarget = flipToRed(blueTarget);
        return computeShootPose(ctx, redTarget);
      }

      @Override
      public Pose2d approximateBluePose() {
        Translation2d target = hubAimpointFromAnchorTagBlueCached(blueHubAnchorTagId);
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

        HubShotCacheKey key = hubKey(targetFieldPos, releaseH, halfL, halfW);

        int rxmm = (int) Math.round(robotPose.getX() * 1000.0);
        int rymm = (int) Math.round(robotPose.getY() * 1000.0);

        int obsHash = obstaclesStableHash(ctx.dynamicObstacles());
        long now = System.nanoTime();

        HubLastPoseCache cached = HUB_LAST_POSE.get(key);
        if (cached != null) {
          Pose2d p = cached.pose;
          if (p != null) {
            int dx = Math.abs(rxmm - cached.robotXmm);
            int dy = Math.abs(rymm - cached.robotYmm);
            long age = now - cached.tNs;
            if (cached.obsHash == obsHash && dx <= 90 && dy <= 90 && age <= 140_000_000L) {
              return p;
            }
          }
        }

        DragShotPlanner.ShotLibrary lib =
            getOrStartHubLibrary(physics, targetFieldPos, releaseH, halfL, halfW);

        Optional<DragShotPlanner.ShotSolution> libSol = Optional.empty();
        Translation2d seed = robotPose.getTranslation();

        if (lib != null && !lib.entries().isEmpty()) {
          libSol =
              DragShotPlanner.findBestShotFromLibrary(
                  lib,
                  physics,
                  targetFieldPos,
                  HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                  robotPose,
                  releaseH,
                  halfL,
                  halfW,
                  ctx.dynamicObstacles(),
                  HUB_SHOT_CONSTRAINTS);
          if (libSol.isPresent()) {
            seed = libSol.get().shooterPosition();
          }
        }

        DragShotPlanner.OnlineSearchState online =
            getOrCreateOnlineState(targetFieldPos, releaseH, halfL, halfW, seed);

        if (online.seed() == null) {
          online.seed(seed);
        } else {
          long ageNs = now - online.lastUpdateNs();
          if (ageNs > 2_000_000_000L) {
            online.seed(seed);
            online.stepMeters(0.60);
          }
        }

        long refineBudget = cached != null && cached.pose != null ? 280_000L : 420_000L;

        Optional<DragShotPlanner.ShotSolution> refined =
            DragShotPlanner.findBestShotOnlineRefine(
                physics,
                targetFieldPos,
                HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                robotPose,
                releaseH,
                halfL,
                halfW,
                ctx.dynamicObstacles(),
                HUB_SHOT_CONSTRAINTS,
                online,
                refineBudget);

        Optional<DragShotPlanner.ShotSolution> sol = refined.isPresent() ? refined : libSol;

        Logger.recordOutput("shot", sol.isPresent() ? sol.get().shooterPosition() : null);
        Logger.recordOutput("shot_lib_entries", lib != null ? lib.entries().size() : 0);
        Logger.recordOutput("shot_lib_complete", lib != null && lib.complete());
        Logger.recordOutput("shot_lib_null", lib == null);

        Pose2d out;
        if (sol.isEmpty()) {
          Translation2d fallbackShooter =
              targetFieldPos.minus(new Translation2d(3.0, Rotation2d.kZero));
          Rotation2d fallbackYaw = targetFieldPos.minus(fallbackShooter).getAngle();
          out = new Pose2d(fallbackShooter, fallbackYaw);
        } else {
          Translation2d shooterPos = sol.get().shooterPosition();
          Rotation2d yaw = sol.get().shooterYaw();
          out = new Pose2d(shooterPos, yaw);
        }

        HubLastPoseCache write = HUB_LAST_POSE.computeIfAbsent(key, k -> new HubLastPoseCache());
        write.robotXmm = rxmm;
        write.robotYmm = rymm;
        write.obsHash = obsHash;
        write.tNs = now;
        write.pose = out;

        return out;
      }

      private static Translation2d hubAimpointFromAnchorTagBlueCached(int anchorTagId) {
        return HUB_AIMPOINT_BLUE_CACHE.computeIfAbsent(
            anchorTagId, Rebuilt2026::hubAimpointFromAnchorTagBlue);
      }
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
