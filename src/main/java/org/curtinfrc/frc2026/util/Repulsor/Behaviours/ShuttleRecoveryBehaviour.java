package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific._Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;

public final class ShuttleRecoveryBehaviour extends Behaviour {
  private static final double SHOOT_BEHIND_HUB_METERS = 2.95;
  private static final double SHOOT_LATERAL_STEP_METERS = 0.45;
  private static final double SHOOT_POS_TOL_METERS = 0.34;
  private static final double SHOOT_YAW_TOL_DEG = 13.0;
  private static final double FIELD_MARGIN_METERS = 0.28;

  private static final double MOTION_COMP_LATENCY_SEC = 0.08;
  private static final double MOTION_COMP_MIN_LEAD_SEC = 0.10;
  private static final double MOTION_COMP_MAX_LEAD_SEC = 0.45;
  private static final double MOTION_COMP_MAX_SPEED_MPS = 4.5;
  private static final double DEFAULT_TIME_TO_PLANE_SEC = 0.18;
  private static final long MAGAZINE_CAPACITY = 16L;
  private static final int RECOVERY_GOAL_UNITS = 2;

  private static final double[] SHOOT_LATERAL_OFFSETS =
      new double[] {0.0, SHOOT_LATERAL_STEP_METERS, -SHOOT_LATERAL_STEP_METERS, 0.9, -0.9};

  private static final GamePiecePhysics SHUTTLE_GAME_PIECE = loadShuttleGamePiece();

  private final int prio;
  private final Supplier<Boolean> hasPiece;
  private final Supplier<Double> ourSpeedCap;

  private final NetworkTablesValue<Double> shotAngle =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotAngle"), 0.0);
  private final NetworkTablesValue<Double> shotSpeed =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotSpeed"), 0.0);
  private final NetworkTablesValue<Boolean> shooterPassthrough =
      NetworkTablesValue.ofBoolean(
          NetworkTableInstance.getDefault(),
          NetworkTablesValue.toAdvantageKit("/ShooterPassthrough"),
          false);
  private final NetworkTablesValue<Long> pieceCount =
      NetworkTablesValue.ofInteger(NetworkTableInstance.getDefault(), "/PieceCount", 0L);

  public ShuttleRecoveryBehaviour(
      int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
    this.prio = priority;
    this.hasPiece = hasPiece;
    this.ourSpeedCap = ourSpeedCap;
  }

  @Override
  public String name() {
    return "ShuttleRecovery";
  }

  @Override
  public int priority() {
    return prio;
  }

  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.SHUTTLE_RECOVERY_MODE);
  }

  @Override
  public Command build(BehaviourContext ctx) {
    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            HeightSetpoint.NONE);

    AtomicReference<Pose2d> shuttleBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint shuttleShootRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "SHUTTLE_RECOVERY_SHOOT_ROUTE", SetpointType.kScore, shuttleBluePoseRef),
            HeightSetpoint.NET);

    AtomicReference<Pose2d> lastRobotPose = new AtomicReference<>(null);
    AtomicLong lastRobotPoseNs = new AtomicLong(0L);
    AtomicReference<Double> lastTimeToPlaneSec = new AtomicReference<>(DEFAULT_TIME_TO_PLANE_SEC);
    AtomicReference<ShotSolution> lastValidShot = new AtomicReference<>(null);

    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();
              long nowNs = System.nanoTime();
              SetpointContext spCtx = makeCtx(ctx, robotPose);
              Translation2d fieldVelocity =
                  estimateFieldVelocity(
                      lastRobotPose.get(), lastRobotPoseNs.get(), robotPose, nowNs);
              lastRobotPose.set(robotPose);
              lastRobotPoseNs.set(nowNs);

              long currentPieceCount = safePieceCount(pieceCount);
              boolean piece = Boolean.TRUE.equals(hasPiece.get()) || currentPieceCount > 0L;
              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

              ShuttleAim aim =
                  computeShuttleAim(
                      robotPose,
                      spCtx,
                      ctx.vision.getObstacles(),
                      fieldVelocity,
                      lastTimeToPlaneSec);

              if (aim.shotSolution().isPresent()) {
                ShotSolution solution = aim.shotSolution().get();
                lastValidShot.set(solution);
                shotSpeed.set(solution.launchSpeedMetersPerSecond());
                shotAngle.set(solution.launchAngle().getDegrees());
              } else {
                ShotSolution prior = lastValidShot.get();
                if (prior != null) {
                  shotSpeed.set(prior.launchSpeedMetersPerSecond());
                  shotAngle.set(prior.launchAngle().getDegrees());
                } else {
                  shotSpeed.set(0.0);
                  shotAngle.set(0.0);
                }
              }

              CategorySpec category;
              RepulsorSetpoint activeGoal;

              if (piece) {
                category = CategorySpec.kScore;
                Pose2d shootFieldPose = aim.shootPose();

                DriverStation.Alliance alliance =
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                Pose2d bluePose =
                    alliance == DriverStation.Alliance.Red
                        ? SetpointUtil.flipToBlue(shootFieldPose)
                        : shootFieldPose;
                shuttleBluePoseRef.set(bluePose);
                activeGoal = shuttleShootRoute;
              } else {
                category = CategorySpec.kCollect;
                Pose2d collectGoal = chooseRecoveryCollectGoalBlue(robotPose, cap);
                collectBluePoseRef.set(collectGoal);
                activeGoal = collectRoute;
              }

              ctx.repulsor.setCurrentGoal(activeGoal);
              Pose2d goalPose = activeGoal.get(spCtx);
              ctx.planner.setRequestedGoal(goalPose);

              boolean readyToShoot =
                  piece && isReadyToShoot(robotPose, goalPose) && aim.shotSolution().isPresent();
              boolean allowPassthrough = readyToShoot && currentPieceCount > 0L;
              shooterPassthrough.set(allowPassthrough);

              if (!piece && currentPieceCount >= MAGAZINE_CAPACITY) {
                shooterPassthrough.set(false);
              }

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      category,
                      false,
                      0.0);

              ctx.drive.runVelocity(
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation()));
            },
            ctx.drive)
        .finallyDo(
            interrupted -> {
              shooterPassthrough.set(false);
              ctx.drive.runVelocity(new ChassisSpeeds());
            });
  }

  private Pose2d chooseRecoveryCollectGoalBlue(Pose2d robotPose, double cap) {
    DriverStation.Alliance wpAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Pose2d robotPoseBlue =
        wpAlliance == DriverStation.Alliance.Red ? SetpointUtil.flipToRed(robotPose) : robotPose;

    Pose2d nextBlue =
        FieldTrackerCore.getInstance()
            .nextAllianceShuttleRecoveryGoalBlue(robotPoseBlue, cap, RECOVERY_GOAL_UNITS);
    if (nextBlue == null) {
      return new Pose2d(
          Constants.FIELD_LENGTH * 0.25, Constants.FIELD_WIDTH * 0.5, robotPoseBlue.getRotation());
    }
    return new Pose2d(nextBlue.getTranslation(), nextBlue.getRotation());
  }

  private ShuttleAim computeShuttleAim(
      Pose2d robotPose,
      SetpointContext spCtx,
      List<? extends Obstacle> obstacles,
      Translation2d fieldVelocity,
      AtomicReference<Double> lastTimeToPlaneSec) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Translation2d hubTarget = _Rebuilt2026.hubAimpointForAlliance(alliance);

    double releaseH = Math.max(0.0, spCtx.shooterReleaseHeightMeters());
    double halfL = Math.max(0.0, spCtx.robotLengthMeters()) / 2.0;
    double halfW = Math.max(0.0, spCtx.robotWidthMeters()) / 2.0;

    double prevFlight =
        lastTimeToPlaneSec.get() == null ? DEFAULT_TIME_TO_PLANE_SEC : lastTimeToPlaneSec.get();
    double leadSec =
        MathUtil.clamp(
            prevFlight + MOTION_COMP_LATENCY_SEC,
            MOTION_COMP_MIN_LEAD_SEC,
            MOTION_COMP_MAX_LEAD_SEC);

    Translation2d compensatedTarget =
        hubTarget.minus(
            new Translation2d(fieldVelocity.getX() * leadSec, fieldVelocity.getY() * leadSec));
    compensatedTarget = clampToField(compensatedTarget);

    Optional<ShotSolution> solved =
        solveShuttleShot(
            robotPose.getTranslation(),
            hubTarget,
            compensatedTarget,
            releaseH,
            halfL,
            halfW,
            obstacles,
            alliance);
    if (solved.isPresent()) {
      ShotSolution solution = solved.get();
      lastTimeToPlaneSec.set(
          MathUtil.clamp(
              solution.timeToPlaneSeconds(), MOTION_COMP_MIN_LEAD_SEC, MOTION_COMP_MAX_LEAD_SEC));
      return new ShuttleAim(new Pose2d(solution.shooterPosition(), solution.shooterYaw()), solved);
    }

    return new ShuttleAim(fallbackShuttlePose(hubTarget, alliance), Optional.empty());
  }

  private Optional<ShotSolution> solveShuttleShot(
      Translation2d robotPos,
      Translation2d hubTarget,
      Translation2d compensatedTarget,
      double shooterReleaseHeightMeters,
      double halfL,
      double halfW,
      List<? extends Obstacle> obstacles,
      DriverStation.Alliance alliance) {
    Translation2d behind = behindDirection(alliance);
    Translation2d lateral = new Translation2d(-behind.getY(), behind.getX());
    Translation2d base =
        clampToField(
            hubTarget.plus(
                new Translation2d(
                    behind.getX() * SHOOT_BEHIND_HUB_METERS,
                    behind.getY() * SHOOT_BEHIND_HUB_METERS)));

    ShotSolution best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (double lateralOffset : SHOOT_LATERAL_OFFSETS) {
      Translation2d shooterPos =
          clampToField(
              base.plus(
                  new Translation2d(
                      lateral.getX() * lateralOffset, lateral.getY() * lateralOffset)));

      if (!DragShotPlanner.isShooterPoseValid(shooterPos, hubTarget, halfL, halfW, obstacles)) {
        continue;
      }

      Optional<ShotSolution> solved =
          DragShotPlanner.calculateStaticShotAngleAndSpeed(
              SHUTTLE_GAME_PIECE,
              shooterPos,
              compensatedTarget,
              _Rebuilt2026.HUB_OPENING_FRONT_EDGE_HEIGHT_M,
              shooterReleaseHeightMeters,
              _Rebuilt2026.HUB_SHOT_CONSTRAINTS);
      if (solved.isEmpty()) {
        continue;
      }

      ShotSolution candidate = solved.get();
      double score =
          Math.abs(candidate.verticalErrorMeters())
              + 0.02 * robotPos.getDistance(candidate.shooterPosition());
      if (score < bestScore) {
        bestScore = score;
        best = candidate;
      }
    }

    return Optional.ofNullable(best);
  }

  private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
    double release;
    try {
      var ht = ctx.repulsor.getTargetHeight();
      var d = ht != null ? ht.getHeight() : null;
      release = d != null ? Math.max(0.0, d.in(Meters)) : 0.0;
    } catch (Exception ignored) {
      release = 0.0;
    }
    return new SetpointContext(
        Optional.of(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        release,
        ctx.vision.getObstacles());
  }

  private static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }

  private static boolean nearPose(Pose2d a, Pose2d b, double posTol, double degTol) {
    if (a.getTranslation().getDistance(b.getTranslation()) > posTol) {
      return false;
    }
    double e =
        Math.abs(shortestAngleRad(a.getRotation().getRadians(), b.getRotation().getRadians()));
    return e <= Math.toRadians(degTol);
  }

  private static boolean isReadyToShoot(Pose2d robotPose, Pose2d goalPose) {
    return nearPose(robotPose, goalPose, SHOOT_POS_TOL_METERS, SHOOT_YAW_TOL_DEG);
  }

  private static Translation2d estimateFieldVelocity(
      Pose2d prevPose, long prevNs, Pose2d nowPose, long nowNs) {
    if (prevPose == null || prevNs == 0L || nowNs <= prevNs) {
      return new Translation2d();
    }
    double dt = (nowNs - prevNs) * 1e-9;
    if (dt < 1e-4) {
      return new Translation2d();
    }
    double vx = (nowPose.getX() - prevPose.getX()) / dt;
    double vy = (nowPose.getY() - prevPose.getY()) / dt;

    double speed = Math.hypot(vx, vy);
    if (speed > MOTION_COMP_MAX_SPEED_MPS && speed > 1e-6) {
      double s = MOTION_COMP_MAX_SPEED_MPS / speed;
      vx *= s;
      vy *= s;
    }
    return new Translation2d(vx, vy);
  }

  private static Pose2d fallbackShuttlePose(
      Translation2d hubTarget, DriverStation.Alliance alliance) {
    Translation2d behind = behindDirection(alliance);
    Translation2d shooterPos =
        clampToField(
            hubTarget.plus(
                new Translation2d(
                    behind.getX() * SHOOT_BEHIND_HUB_METERS,
                    behind.getY() * SHOOT_BEHIND_HUB_METERS)));
    Rotation2d yaw = hubTarget.minus(shooterPos).getAngle();
    return new Pose2d(shooterPos, yaw);
  }

  private static Translation2d clampToField(Translation2d point) {
    double x =
        MathUtil.clamp(
            point.getX(), FIELD_MARGIN_METERS, Constants.FIELD_LENGTH - FIELD_MARGIN_METERS);
    double y =
        MathUtil.clamp(
            point.getY(), FIELD_MARGIN_METERS, Constants.FIELD_WIDTH - FIELD_MARGIN_METERS);
    return new Translation2d(x, y);
  }

  private static Translation2d behindDirection(DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      return new Translation2d(1.0, 0.0);
    }
    return new Translation2d(-1.0, 0.0);
  }

  private static GamePiecePhysics loadShuttleGamePiece() {
    try {
      return DragShotPlanner.loadGamePieceFromDeployYaml(_Rebuilt2026.GAME_PIECE_ID_FUEL);
    } catch (Throwable ignored) {
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

  private static long safePieceCount(NetworkTablesValue<Long> countValue) {
    Long value = countValue.get();
    if (value == null) {
      return 0L;
    }
    return Math.max(0L, value);
  }

  private record ShuttleAim(Pose2d shootPose, Optional<ShotSolution> shotSolution) {}
}
