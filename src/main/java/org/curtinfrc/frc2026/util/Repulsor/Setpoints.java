/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Meters;
import static org.curtinfrc.frc2026.util.Repulsor.Constants.aprilTagLayout;

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
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.OnlineSearchState;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotLibrary;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotLibraryBuilder;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;

public class Setpoints {

  public static Alliance currentAllianceOrBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private static Pose2d flipAcrossField(Pose2d p) {
    if (p == null) return Pose2d.kZero;
    double x = p.getX();
    double y = p.getY();
    double r = p.getRotation().getRadians();
    double fx = Constants.FIELD_LENGTH - x;
    double fr = Math.PI - r;
    return new Pose2d(fx, y, Rotation2d.fromRadians(fr));
  }

  private static Translation2d flipAcrossField(Translation2d t) {
    if (t == null) return new Translation2d(0.0, 0.0);
    return new Translation2d(Constants.FIELD_LENGTH - t.getX(), t.getY());
  }

  public static Pose2d flipToRed(Pose2d bluePose) {
    return flipAcrossField(bluePose);
  }

  public static Translation2d flipToRed(Translation2d blue) {
    return flipAcrossField(blue);
  }

  public static Pose2d flipToBlue(Pose2d redPose) {
    return flipAcrossField(redPose);
  }

  public static Translation2d flipToBlue(Translation2d red) {
    return flipAcrossField(red);
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
      List<? extends Obstacle> dynamicObstacles) {

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
    private final boolean canFlip;

    protected GameSetpoint(String name, SetpointType type) {
      this.name = name;
      this.type = type;
      this.canFlip = true;
    }

    protected GameSetpoint(String name, SetpointType type, boolean canFlip) {
      this.name = name;
      this.type = type;
      this.canFlip = canFlip;
    }

    public final String name() {
      return name;
    }

    public final SetpointType type() {
      return type;
    }

    public abstract Pose2d bluePose(SetpointContext ctx);

    public Pose2d redPose(SetpointContext ctx) {
      return canFlip ? flipToRed(bluePose(ctx)) : bluePose(ctx);
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
      super(name, type == null ? SetpointType.kOther : type, false);
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

    public static final double HUB_OPENING_FRONT_EDGE_HEIGHT_M = 1.43;
    public static final double HUB_FACE_TO_AIMPOINT_METERS = 0.60;
    public static final String GAME_PIECE_ID_FUEL = "fuel";

    private static final long HUB_CACHE_MAX_AGE_NS = 280_000_000L;
    private static final int HUB_REUSE_POS_MM = 160;
    private static final double HUB_SIMPLE_FAR_DIST_M = 4.35;
    private static final double HUB_SIMPLE_RADIUS_M = 3.0;
    private static final double HUB_OBS_ENABLE_DIST_M = 2.15;

    private static final GamePiecePhysics HUB_PHYSICS = loadHubPhysicsOnce();

    private static GamePiecePhysics loadHubPhysicsOnce() {
      try {
        return DragShotPlanner.loadGamePieceFromDeployYaml(GAME_PIECE_ID_FUEL);
      } catch (Throwable t) {
        return new GamePiecePhysics() {
          @Override
          public double massKg() {
            return 0.27;
          }

          @Override
          public double crossSectionAreaM2() {
            return 0.014;
          }

          @Override
          public double dragCoefficient() {
            return 0.95;
          }
        };
      }
    }

    private static final class HubLastPoseCache {
      volatile int robotXmm;
      volatile int robotYmm;
      volatile int obsHash;
      volatile long tNs;
      volatile Pose2d pose;

      volatile long lastSolveNs;
      volatile int lastSolveRobotXmm;
      volatile int lastSolveRobotYmm;
      volatile int lastSolveObsHash;

      volatile long lastRefineNs;
      volatile ShotSolution lastSolution;
    }

    private static final double[][] SIMPLE_OFFS =
        new double[][] {
          {0.0, 0.0},
          {0.12, 0.0},
          {-0.12, 0.0},
          {0.0, 0.12},
          {0.0, -0.12},
          {0.18, 0.18},
          {0.18, -0.18},
          {-0.18, 0.18},
          {-0.18, -0.18},
          {0.26, 0.0},
          {-0.26, 0.0},
          {0.34, 0.0},
          {-0.34, 0.0}
        };

    private static final double[] RING_RADII =
        new double[] {2.4, 2.7, 3.0, 3.3, 3.6, 4.0, 4.4, 4.9, 5.4, 6.0};

    private static final double[] BEARING_OFFS_DEG =
        new double[] {
          0.0, 10.0, -10.0, 20.0, -20.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 75.0, -75.0, 90.0,
          -90.0, 120.0, -120.0, 150.0, -150.0, 180.0
        };

    private static final Translation2d[] SAFE_FALLBACKS =
        new Translation2d[] {
          new Translation2d(1.20, 1.20),
          new Translation2d(1.20, Constants.FIELD_WIDTH * 0.50),
          new Translation2d(1.20, Constants.FIELD_WIDTH - 1.20),
          new Translation2d(Constants.FIELD_LENGTH - 1.20, 1.20),
          new Translation2d(Constants.FIELD_LENGTH - 1.20, Constants.FIELD_WIDTH * 0.50),
          new Translation2d(Constants.FIELD_LENGTH - 1.20, Constants.FIELD_WIDTH - 1.20)
        };

    private static Pose2d toPoseFacing(Translation2d pos, Translation2d target) {
      Rotation2d yaw = target.minus(pos).getAngle();
      return new Pose2d(pos, yaw);
    }

    private static boolean validShootPose(
        Translation2d shooterPos,
        Translation2d targetFieldPos,
        double halfL,
        double halfW,
        List<? extends Obstacle> obs) {
      return DragShotPlanner.isShooterPoseValid(shooterPos, targetFieldPos, halfL, halfW, obs);
    }

    private static Pose2d safeValidFallback(
        Translation2d target,
        double halfL,
        double halfW,
        List<? extends Obstacle> obs) {
      for (Translation2d p : SAFE_FALLBACKS) {
        if (validShootPose(p, target, halfL, halfW, obs)) {
          return toPoseFacing(p, target);
        }
      }
      Translation2d p = SAFE_FALLBACKS[0];
      return toPoseFacing(p, target);
    }

    private static Pose2d findValidAround(
        Pose2d robotPose,
        Translation2d target,
        double halfL,
        double halfW,
        List<? extends Obstacle> obs,
        double baseRadius) {

      Translation2d r = robotPose.getTranslation();
      Translation2d fromTarget = r.minus(target);
      double baseBearingDeg;
      if (fromTarget.getNorm() < 1e-6) {
        baseBearingDeg = 0.0;
      } else {
        baseBearingDeg = fromTarget.getAngle().getDegrees();
      }

      for (int rr = 0; rr < RING_RADII.length; rr++) {
        double rad = RING_RADII[rr];
        double radius = rr == 0 ? Math.max(0.8, Math.min(6.8, baseRadius)) : rad;
        for (int bi = 0; bi < BEARING_OFFS_DEG.length; bi++) {
          double b = baseBearingDeg + BEARING_OFFS_DEG[bi];
          Rotation2d ang = Rotation2d.fromDegrees(b);
          Translation2d p = target.plus(new Translation2d(radius, ang));
          if (validShootPose(p, target, halfL, halfW, obs)) {
            return toPoseFacing(p, target);
          }
          for (int oi = 0; oi < SIMPLE_OFFS.length; oi++) {
            Translation2d pp =
                new Translation2d(p.getX() + SIMPLE_OFFS[oi][0], p.getY() + SIMPLE_OFFS[oi][1]);
            if (validShootPose(pp, target, halfL, halfW, obs)) {
              return toPoseFacing(pp, target);
            }
          }
        }
      }
      return safeValidFallback(target, halfL, halfW, obs);
    }

    private static Pose2d findFastValidPose(
        Pose2d robotPose,
        Translation2d targetFieldPos,
        double halfL,
        double halfW,
        List<? extends Obstacle> obs,
        double baseRadius) {

      Pose2d candidate = findValidAround(robotPose, targetFieldPos, halfL, halfW, obs, baseRadius);
      Translation2d p = candidate.getTranslation();
      if (validShootPose(p, targetFieldPos, halfL, halfW, obs)) {
        return candidate;
      }
      return safeValidFallback(targetFieldPos, halfL, halfW, obs);
    }

    public static final Constraints HUB_SHOT_CONSTRAINTS =
        new Constraints(0, 30, 60, 90.0, Constraints.ShotStyle.ARC);

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

    public static Optional<ShotSolution> getHubShotSolution(SetpointContext ctx) {
      if (ctx == null || ctx.robotPose().isEmpty()) {
        return Optional.empty();
      }

      Translation2d target = hubAimpointForAlliance(currentAllianceOrBlue());
      Pose2d pose = HubShootSetpoint.computeShootPose(ctx, target);

      double releaseH = Math.max(0.0, ctx.shooterReleaseHeightMeters());
      double halfL = Math.max(0.0, ctx.robotLengthMeters()) / 2.0;
      double halfW = Math.max(0.0, ctx.robotWidthMeters()) / 2.0;

      HubShotCacheKey key = hubKey(target, releaseH, halfL, halfW);
      HubLastPoseCache state = HUB_LAST_POSE.get(key);
      if (state == null || state.lastSolution == null) {
        return Optional.empty();
      }

      Translation2d sp = state.lastSolution.shooterPosition();
      if (pose.getTranslation().getDistance(sp) > 0.02) {
        return Optional.empty();
      }
      return Optional.of(state.lastSolution);
    }

    private record HubShotCacheKey(
        int targetXmm, int targetYmm, int releaseHmm, int halfLmm, int halfWmm) {}

    private static final ConcurrentHashMap<HubShotCacheKey, HubShotLibraryState> HUB_LIB_CACHE =
        new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<HubShotCacheKey, OnlineSearchState> HUB_ONLINE_STATE =
        new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<HubShotCacheKey, HubLastPoseCache> HUB_LAST_POSE =
        new ConcurrentHashMap<>();

    private static final AtomicBoolean HUB_BG_STARTED = new AtomicBoolean(false);
    private static ScheduledExecutorService HUB_BG_EXEC;

    private static final AtomicLong HUB_LAST_LOG_NS = new AtomicLong(0L);

    private static final class HubShotLibraryState {
      final HubShotCacheKey key;
      final GamePiecePhysics physics;
      final Translation2d targetFieldPos;
      final double releaseH;
      final double halfL;
      final double halfW;

      volatile ShotLibrary published;
      volatile boolean done;
      final Object lock = new Object();
      ShotLibraryBuilder builder;

      HubShotLibraryState(
          HubShotCacheKey key,
          GamePiecePhysics physics,
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
          synchronized (st.lock) {
            if (st.done) {
              continue;
            }
            if (st.builder == null) {
              double speedStep =
                  Math.max(
                      0.20,
                      (HUB_SHOT_CONSTRAINTS.maxLaunchSpeedMetersPerSecond()
                              - HUB_SHOT_CONSTRAINTS.minLaunchSpeedMetersPerSecond())
                          / 72.0);
              double angleStep =
                  Math.max(
                      0.55,
                      (HUB_SHOT_CONSTRAINTS.maxLaunchAngleDeg()
                              - HUB_SHOT_CONSTRAINTS.minLaunchAngleDeg())
                          / 84.0);
              double radialStep = 0.30;
              double bearingStep = 12.0;

              st.builder =
                  new ShotLibraryBuilder(
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
            ShotLibrary publish = st.builder.maybeStep(900_000L);
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

    private static ShotLibrary getOrStartHubLibrary(
        GamePiecePhysics physics,
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

      ShotLibrary pub = st.published;
      if (pub != null) return pub;
      return null;
    }

    private static OnlineSearchState getOrCreateOnlineState(
        HubShotCacheKey key, Translation2d seed) {
      return HUB_ONLINE_STATE.computeIfAbsent(key, k -> new OnlineSearchState(seed, 0.58));
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

    private static int obstaclesStableHash(List<? extends Obstacle> obs) {
      if (obs == null || obs.isEmpty()) return 0;
      return (System.identityHashCode(obs) * 31) ^ obs.size();
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
        Pose2d fakeRobot =
            new Pose2d(target.plus(new Translation2d(3.0, Rotation2d.kZero)), Rotation2d.kZero);
        return findValidAround(fakeRobot, target, 0.0, 0.0, List.of(), HUB_SIMPLE_RADIUS_M);
      }

      private static Pose2d computeShootPose(SetpointContext ctx, Translation2d targetFieldPos) {
        if (ctx == null || ctx.robotPose().isEmpty()) {
          Pose2d fakeRobot =
              new Pose2d(
                  targetFieldPos.plus(new Translation2d(3.0, Rotation2d.kZero)), Rotation2d.kZero);
          return findValidAround(
              fakeRobot, targetFieldPos, 0.0, 0.0, List.of(), HUB_SIMPLE_RADIUS_M);
        }

        Pose2d robotPose = ctx.robotPose().get();
        long now = System.nanoTime();

        double releaseH = Math.max(0.0, ctx.shooterReleaseHeightMeters());
        double halfL = Math.max(0.0, ctx.robotLengthMeters()) / 2.0;
        double halfW = Math.max(0.0, ctx.robotWidthMeters()) / 2.0;

        double distToTarget = robotPose.getTranslation().getDistance(targetFieldPos);

        boolean useObs = distToTarget <= HUB_OBS_ENABLE_DIST_M;
        List<? extends Obstacle> obs = useObs ? ctx.dynamicObstacles() : List.of();
        int obsHash = useObs ? obstaclesStableHash(obs) : 0;

        HubShotCacheKey key = hubKey(targetFieldPos, releaseH, halfL, halfW);
        int rxmm = (int) Math.round(robotPose.getX() * 1000.0);
        int rymm = (int) Math.round(robotPose.getY() * 1000.0);

        HubLastPoseCache cached = HUB_LAST_POSE.get(key);
        if (cached != null) {
          Pose2d p = cached.pose;
          if (p != null) {
            int dx = Math.abs(rxmm - cached.robotXmm);
            int dy = Math.abs(rymm - cached.robotYmm);
            long age = now - cached.tNs;
            if (cached.obsHash == obsHash
                && dx <= HUB_REUSE_POS_MM
                && dy <= HUB_REUSE_POS_MM
                && age <= HUB_CACHE_MAX_AGE_NS) {
              if (validShootPose(p.getTranslation(), targetFieldPos, halfL, halfW, obs)) {
                return p;
              }
            }
          }
        }

        Pose2d simple =
            findFastValidPose(robotPose, targetFieldPos, halfL, halfW, obs, HUB_SIMPLE_RADIUS_M);

        if (distToTarget >= HUB_SIMPLE_FAR_DIST_M) {
          HubLastPoseCache w = HUB_LAST_POSE.computeIfAbsent(key, k -> new HubLastPoseCache());
          w.robotXmm = rxmm;
          w.robotYmm = rymm;
          w.obsHash = obsHash;
          w.tNs = now;
          w.pose = simple;
          return simple;
        }

        HubLastPoseCache state = HUB_LAST_POSE.computeIfAbsent(key, k -> new HubLastPoseCache());

        if (state.lastSolution != null) {
          Translation2d sp = state.lastSolution.shooterPosition();
          if (validShootPose(sp, targetFieldPos, halfL, halfW, obs)) {
            int dx = Math.abs(rxmm - state.lastSolveRobotXmm);
            int dy = Math.abs(rymm - state.lastSolveRobotYmm);
            long sinceSolve = now - state.lastSolveNs;
            if (state.lastSolveObsHash == obsHash
                && dx <= 140
                && dy <= 140
                && sinceSolve <= 90_000_000L) {
              Pose2d out = new Pose2d(sp, state.lastSolution.shooterYaw());
              state.robotXmm = rxmm;
              state.robotYmm = rymm;
              state.obsHash = obsHash;
              state.tNs = now;
              state.pose = out;
              return out;
            }
          }
        }

        Translation2d seed = simple.getTranslation();

        ShotLibrary lib = getOrStartHubLibrary(HUB_PHYSICS, targetFieldPos, releaseH, halfL, halfW);

        ShotSolution libSol = null;
        if (lib != null && !lib.entries().isEmpty()) {
          Optional<ShotSolution> opt =
              DragShotPlanner.findBestShotFromLibrary(
                  lib,
                  HUB_PHYSICS,
                  targetFieldPos,
                  HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                  robotPose,
                  releaseH,
                  halfL,
                  halfW,
                  obs,
                  HUB_SHOT_CONSTRAINTS);
          if (opt.isPresent()) {
            libSol = opt.get();
            seed = libSol.shooterPosition();
          }
        }

        OnlineSearchState online = getOrCreateOnlineState(key, seed);

        int mdx = Math.abs(rxmm - state.lastSolveRobotXmm);
        int mdy = Math.abs(rymm - state.lastSolveRobotYmm);
        boolean movedFar = mdx > 220 || mdy > 220;
        boolean obsChanged = state.lastSolveObsHash != obsHash;

        if (obsChanged || movedFar) {
          online.seed(seed);
          online.stepMeters(0.58);
        }

        long sinceRefine = now - state.lastRefineNs;

        boolean haveRecentSol = false;
        if (state.lastSolution != null) {
          Translation2d sp = state.lastSolution.shooterPosition();
          if (validShootPose(sp, targetFieldPos, halfL, halfW, obs)) {
            long sinceSolve = now - state.lastSolveNs;
            haveRecentSol = sinceSolve <= 180_000_000L;
          }
        }

        boolean allowRefine = !haveRecentSol && sinceRefine >= 120_000_000L;

        ShotSolution refinedSol = null;
        if (allowRefine) {
          long refineBudgetNs = useObs ? 160_000L : 120_000L;
          Optional<ShotSolution> refined =
              DragShotPlanner.findBestShotOnlineRefine(
                  HUB_PHYSICS,
                  targetFieldPos,
                  HUB_OPENING_FRONT_EDGE_HEIGHT_M,
                  robotPose,
                  releaseH,
                  halfL,
                  halfW,
                  obs,
                  HUB_SHOT_CONSTRAINTS,
                  online,
                  refineBudgetNs);
          if (refined.isPresent()) {
            refinedSol = refined.get();
            state.lastRefineNs = now;
          } else {
            state.lastRefineNs = now;
          }
        }

        ShotSolution sol = refinedSol != null ? refinedSol : libSol;

        Pose2d out;
        if (sol == null) {
          out = simple;
        } else {
          Translation2d shooterPos = sol.shooterPosition();
          if (!validShootPose(shooterPos, targetFieldPos, halfL, halfW, obs)) {
            out = simple;
          } else {
            out = new Pose2d(shooterPos, sol.shooterYaw());
            state.lastSolution = sol;
            state.lastSolveNs = now;
            state.lastSolveRobotXmm = rxmm;
            state.lastSolveRobotYmm = rymm;
            state.lastSolveObsHash = obsHash;
          }
        }

        if (!validShootPose(out.getTranslation(), targetFieldPos, halfL, halfW, obs)) {
          out =
              findFastValidPose(robotPose, targetFieldPos, halfL, halfW, obs, HUB_SIMPLE_RADIUS_M);
        }
        if (!validShootPose(out.getTranslation(), targetFieldPos, halfL, halfW, obs)) {
          out = safeValidFallback(targetFieldPos, halfL, halfW, obs);
        }

        state.robotXmm = rxmm;
        state.robotYmm = rymm;
        state.obsHash = obsHash;
        state.tNs = now;
        state.pose = out;

        long lastNs = HUB_LAST_LOG_NS.get();
        if ((lastNs == 0L || (now - lastNs) >= 220_000_000L)
            && HUB_LAST_LOG_NS.compareAndSet(lastNs, now)) {
          // Logger.recordOutput("hubshot/distToTarget", distToTarget);
          // Logger.recordOutput("hubshot/useObs", useObs);
          // Logger.recordOutput("hubshot/lib_entries", lib != null ? lib.entries().size() : 0);
          // Logger.recordOutput("hubshot/lib_complete", lib != null && lib.complete());
          // Logger.recordOutput("hubshot/solved", sol != null);
          // Logger.recordOutput("hubshot/used_refine", refinedSol != null);
          // Logger.recordOutput(
          // "hubshot/pose_valid",
          // validShootPose(out.getTranslation(), targetFieldPos, halfL, halfW, obs));
        }

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

