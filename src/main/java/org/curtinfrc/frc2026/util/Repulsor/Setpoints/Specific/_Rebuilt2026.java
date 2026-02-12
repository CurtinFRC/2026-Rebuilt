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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific;

import static org.curtinfrc.frc2026.util.Repulsor.Constants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.OnlineSearchState;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotLibrary;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotLibraryBuilder;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;

public class _Rebuilt2026 {
  protected _Rebuilt2026() {}

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
      Translation2d target, double halfL, double halfW, List<? extends Obstacle> obs) {
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
          new Pose2d(Constants.FIELD_LENGTH / 2.0, Constants.FIELD_WIDTH / 2.0, Rotation2d.kZero));

  public static final GameSetpoint OUTPOST_COLLECT =
      new ApproachFromTagSetpoint(
          "OUTPOST_COLLECT", SetpointType.kHumanPlayer, BLUE_OUTPOST_ANCHOR_TAG_ID, 1.25);

  private static final ConcurrentHashMap<Integer, Translation2d> HUB_AIMPOINT_BLUE_CACHE =
      new ConcurrentHashMap<>();

  public static Translation2d hubAimpointBlue() {
    return HUB_AIMPOINT_BLUE_CACHE.computeIfAbsent(
        BLUE_HUB_ANCHOR_TAG_ID, _Rebuilt2026::hubAimpointFromAnchorTagBlue);
  }

  public static Translation2d hubAimpointForAlliance(Alliance alliance) {
    Translation2d blue = hubAimpointBlue();
    return alliance == Alliance.Red ? SetpointUtil.flipToRed(blue) : blue;
  }

  public static Optional<ShotSolution> getHubShotSolution(SetpointContext ctx) {
    if (ctx == null || ctx.robotPose().isEmpty()) {
      return Optional.empty();
    }

    Translation2d target = hubAimpointForAlliance(SetpointUtil.currentAllianceOrBlue());
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
        _Rebuilt2026::hubBackgroundTick, 0L, 20L, TimeUnit.MILLISECONDS);
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
            key, k -> new HubShotLibraryState(k, physics, targetFieldPos, releaseH, halfL, halfW));

    ShotLibrary pub = st.published;
    if (pub != null) return pub;
    return null;
  }

  private static OnlineSearchState getOrCreateOnlineState(HubShotCacheKey key, Translation2d seed) {
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
      Translation2d redTarget = SetpointUtil.flipToRed(blueTarget);
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
        return findValidAround(fakeRobot, targetFieldPos, 0.0, 0.0, List.of(), HUB_SIMPLE_RADIUS_M);
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
        out = findFastValidPose(robotPose, targetFieldPos, halfL, halfW, obs, HUB_SIMPLE_RADIUS_M);
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
          anchorTagId, _Rebuilt2026::hubAimpointFromAnchorTagBlue);
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
