package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Metrics.HPStationMetrics;
import org.curtinfrc.frc2026.util.Repulsor.Metrics.MetricRecorder;
import org.curtinfrc.frc2026.util.Repulsor.PredictiveFieldState;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.littletonrobotics.junction.Logger;

public class AutoPathBehaviour extends Behaviour {
  private final int prio;
  private final Supplier<Boolean> inScoring;
  private final Supplier<Boolean> inCollecting;
  private final Supplier<RepulsorSetpoint> nextScore;
  private final Supplier<List<RepulsorSetpoint>> hpOptions;
  private final Supplier<Boolean> atHPStation;
  private final Supplier<Boolean> hasPiece;
  private final Function<RepulsorSetpoint, String> stationKeyFn;
  private final Supplier<Double> ourSpeedCap;

  public AutoPathBehaviour(
      int priority,
      Supplier<Boolean> inScoring,
      Supplier<Boolean> inCollecting,
      Supplier<RepulsorSetpoint> nextScore,
      Supplier<List<RepulsorSetpoint>> hpOptions,
      Supplier<Boolean> atHPStation,
      Supplier<Boolean> hasPiece,
      Function<RepulsorSetpoint, String> stationKeyFn,
      Supplier<Double> ourSpeedCap) {
    this.prio = priority;
    this.inScoring = inScoring;
    this.inCollecting = inCollecting;
    this.nextScore = nextScore;
    this.hpOptions = hpOptions;
    this.atHPStation = atHPStation;
    this.hasPiece = hasPiece;
    this.stationKeyFn = stationKeyFn;
    this.ourSpeedCap = ourSpeedCap;
  }

  @Override
  public String name() {
    return "AutoPath";
  }

  @Override
  public int priority() {
    return prio;
  }

  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return !flags.contains(BehaviourFlag.DEFENSE_MODE)
        && !flags.contains(BehaviourFlag.SHOOTING_TEST);
  }

  private static final class ArcFollowState {
    boolean active;
    boolean returning;
    Pose2d startPose;
    Pose2d shootPose;
    Pose2d centerPose;
    Translation2d control;
    Translation2d targetPoint;
    double lookaheadT;
    long startNs;

    ArcFollowState() {
      reset();
    }

    void reset() {
      active = false;
      returning = false;
      startPose = Pose2d.kZero;
      shootPose = Pose2d.kZero;
      centerPose = Pose2d.kZero;
      control = new Translation2d();
      targetPoint = new Translation2d();
      lookaheadT = 0.12;
      startNs = 0L;
    }
  }

  private static Translation2d bezier2(
      Translation2d a, Translation2d b, Translation2d c, double t) {
    double u = 1.0 - t;
    Translation2d p0 = a.times(u * u);
    Translation2d p1 = b.times(2.0 * u * t);
    Translation2d p2 = c.times(t * t);
    return p0.plus(p1).plus(p2);
  }

  private static double clamp01(double v) {
    return MathUtil.clamp(v, 0.0, 1.0);
  }

  private static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
  }

  private static double closestTOnBezier(
      Translation2d a, Translation2d b, Translation2d c, Translation2d p) {
    double bestT = 0.0;
    double bestD2 = Double.POSITIVE_INFINITY;
    for (int i = 0; i <= 40; i++) {
      double t = i / 40.0;
      Translation2d q = bezier2(a, b, c, t);
      double dx = q.getX() - p.getX();
      double dy = q.getY() - p.getY();
      double d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        bestT = t;
      }
    }
    for (int i = 0; i < 6; i++) {
      double t0 = Math.max(0.0, bestT - 0.03);
      double t1 = Math.min(1.0, bestT + 0.03);
      double m0 = (t0 + bestT) * 0.5;
      double m1 = (bestT + t1) * 0.5;
      double d0 = bezier2(a, b, c, t0).getDistance(p);
      double dm0 = bezier2(a, b, c, m0).getDistance(p);
      double dm1 = bezier2(a, b, c, m1).getDistance(p);
      double d1 = bezier2(a, b, c, t1).getDistance(p);
      double min = d0;
      bestT = t0;
      if (dm0 < min) {
        min = dm0;
        bestT = m0;
      }
      if (dm1 < min) {
        min = dm1;
        bestT = m1;
      }
      if (d1 < min) {
        min = d1;
        bestT = t1;
      }
    }
    return bestT;
  }

  private static Translation2d computeControlPoint(
      Translation2d a, Translation2d c, double curvatureMeters) {
    Translation2d mid = new Translation2d((a.getX() + c.getX()) * 0.5, (a.getY() + c.getY()) * 0.5);
    Translation2d d = c.minus(a);
    double norm = d.getNorm();
    if (norm < 1e-6) return mid;
    Translation2d perp = new Translation2d(-d.getY() / norm, d.getX() / norm);
    return mid.plus(perp.times(curvatureMeters));
  }

  private static ChassisSpeeds driveToPoint(
      Pose2d robotPose,
      Translation2d point,
      Rotation2d desiredHeading,
      double maxSpeed,
      double maxOmega,
      double kPxy,
      double kPomega) {

    Translation2d err = point.minus(robotPose.getTranslation());
    double vx = MathUtil.clamp(err.getX() * kPxy, -maxSpeed, maxSpeed);
    double vy = MathUtil.clamp(err.getY() * kPxy, -maxSpeed, maxSpeed);

    double eTheta =
        shortestAngleRad(robotPose.getRotation().getRadians(), desiredHeading.getRadians());
    double omega = MathUtil.clamp(eTheta * kPomega, -maxOmega, maxOmega);

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation());
  }

  private static boolean nearPose(Pose2d a, Pose2d b, double posTol, double degTol) {
    if (a.getTranslation().getDistance(b.getTranslation()) > posTol) return false;
    double e =
        Math.abs(shortestAngleRad(a.getRotation().getRadians(), b.getRotation().getRadians()));
    return e <= Math.toRadians(degTol);
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
        ctx.coral_offset,
        ctx.algae_offset,
        release,
        ctx.vision.getObstacles());
  }

  @Override
  public Command build(BehaviourContext ctx) {
    AtomicReference<RepulsorSetpoint> active = new AtomicReference<>();
    AtomicBoolean init = new AtomicBoolean(false);
    AtomicReference<CategorySpec> lastCat = new AtomicReference<>(null);
    AtomicReference<String> timingStation = new AtomicReference<>(null);
    AtomicLong timingStartNs = new AtomicLong(0);
    AtomicReference<RepulsorSetpoint> lastEpisodeGoal = new AtomicReference<>(null);
    AtomicLong lastEpisodeFinalizeNs = new AtomicLong(0L);

    final long EP_COOLDOWN_NS = 1_000_000_000L;
    final long PINNED_FAIL_NS = 2_000_000_000L;
    final long STUCK_FAIL_NS = 3_000_000_000L;
    final double PROGRESS_EPS_METERS = 0.03;
    final double PINNED_PROGRESS_MIN_METERS = 0.15;
    final double STUCK_DIST_MIN_METERS = 0.5;
    final double SUCCESS_NEAR_DIST_METERS = 0.40;

    final int COLLECT_GOAL_UNITS = 3;

    AtomicLong episodeStartNs = new AtomicLong(0L);
    AtomicReference<Double> episodeBestDist = new AtomicReference<>(null);
    AtomicLong lastProgressNs = new AtomicLong(0L);
    AtomicBoolean episodeEverNearGoal = new AtomicBoolean(false);

    AtomicLong pinnedEnterNs = new AtomicLong(0L);
    AtomicReference<Double> pinnedStartBestDist = new AtomicReference<>(null);
    AtomicReference<Double> pinnedBestDist = new AtomicReference<>(null);
    AtomicBoolean pinnedFailedThisLatch = new AtomicBoolean(false);

    ArcFollowState arc = new ArcFollowState();

    ReactiveBypass byp = ctx.planner.bypass;

    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint("COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            HeightSetpoint.NONE);

    final RepulsorSetpoint scoreGoal =
        new RepulsorSetpoint(Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);
    final RepulsorSetpoint centerCollectGoal =
        new RepulsorSetpoint(Rebuilt2026.CENTER_COLLECT, HeightSetpoint.NONE);

    Consumer<Boolean> finalizeEpisode =
        forceSuccess -> {
          if (lastEpisodeGoal.get() == null) {
            return;
          }
          long now = System.nanoTime();
          long last = lastEpisodeFinalizeNs.get();
          if (last != 0L && now - last < EP_COOLDOWN_NS) {
            return;
          }
          boolean success = forceSuccess;
          if (episodeEverNearGoal.get()) {
            success = true;
          }
          byp.finalizeEpisode(success);
          lastEpisodeFinalizeNs.set(now);
          episodeStartNs.set(0L);
          episodeBestDist.set(null);
          lastProgressNs.set(0L);
          pinnedEnterNs.set(0L);
          pinnedStartBestDist.set(null);
          pinnedBestDist.set(null);
          pinnedFailedThisLatch.set(false);
          episodeEverNearGoal.set(false);
        };

    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();

              boolean scoringNow = inScoring != null && Boolean.TRUE.equals(inScoring.get());
              boolean piece = hasPiece.get();

              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

              if (scoringNow && piece) {
                if (!arc.active) {
                  arc.active = true;
                  arc.returning = false;
                  arc.startPose = robotPose;

                  SetpointContext spCtx = makeCtx(ctx, robotPose);

                  arc.shootPose = scoreGoal.get(spCtx);
                  arc.centerPose = centerCollectGoal.get(spCtx);

                  edu.wpi.first.wpilibj.DriverStation.Alliance wpilibAlliance =
                      DriverStation.getAlliance()
                          .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
                  arc.targetPoint = Rebuilt2026.hubAimpointForAlliance(wpilibAlliance);

                  double chord =
                      arc.startPose.getTranslation().getDistance(arc.shootPose.getTranslation());
                  double curvature = MathUtil.clamp(0.35 * chord, 0.6, 1.6);
                  arc.control =
                      computeControlPoint(
                          arc.startPose.getTranslation(),
                          arc.shootPose.getTranslation(),
                          curvature);

                  arc.lookaheadT = 0.14;
                  arc.startNs = System.nanoTime();

                  ctx.planner.clearCommitted();
                }

                if (!arc.returning) {
                  Translation2d a = arc.startPose.getTranslation();
                  Translation2d b = arc.control;
                  Translation2d c = arc.shootPose.getTranslation();

                  double tClose = closestTOnBezier(a, b, c, robotPose.getTranslation());
                  double tLook = clamp01(tClose + arc.lookaheadT);
                  Translation2d aimPoint = bezier2(a, b, c, tLook);

                  Rotation2d desiredYaw =
                      arc.targetPoint.minus(robotPose.getTranslation()).getAngle();

                  Rebuilt2026.onShootArcTick(robotPose, arc.shootPose, tClose);

                  ChassisSpeeds speeds =
                      driveToPoint(robotPose, aimPoint, desiredYaw, cap, 7.0, 2.2, 6.0);

                  ctx.drive.runVelocity(speeds);

                  Logger.recordOutput("shoot_arc_active", true);
                  Logger.recordOutput("shoot_arc_returning", false);
                  Logger.recordOutput("shoot_arc_t", tClose);

                  if (nearPose(robotPose, arc.shootPose, 0.22, 10.0) || tClose >= 0.985) {
                    arc.returning = true;
                  }
                  return;
                } else {
                  Translation2d goal = arc.centerPose.getTranslation();
                  Rotation2d desiredYaw = arc.centerPose.getRotation();

                  ChassisSpeeds speeds =
                      driveToPoint(robotPose, goal, desiredYaw, cap, 7.0, 2.0, 5.0);

                  ctx.drive.runVelocity(speeds);

                  Logger.recordOutput("shoot_arc_active", true);
                  Logger.recordOutput("shoot_arc_returning", true);

                  if (nearPose(robotPose, arc.centerPose, 0.28, 12.0)) {
                    arc.reset();
                    ctx.drive.runVelocity(new ChassisSpeeds());
                  }
                  return;
                }
              } else {
                if (arc.active) {
                  arc.reset();
                }
              }

              Logger.recordOutput("shoot_arc_active", false);

              CategorySpec cat = piece ? CategorySpec.kScore : CategorySpec.kCollect;

              if (!init.get()) {
                active.set(
                    cat == CategorySpec.kScore
                        ? scoreGoal
                        : chooseCollect(
                            ctx,
                            robotPose,
                            cap,
                            COLLECT_GOAL_UNITS,
                            collectBluePoseRef,
                            collectRoute));
                lastCat.set(cat);
                init.set(true);
              }

              if (lastCat.get() != cat) {
                if (lastCat.get() != null && lastEpisodeGoal.get() != null) {
                  finalizeEpisode.accept(false);
                }
                active.set(
                    cat == CategorySpec.kScore
                        ? scoreGoal
                        : chooseCollect(
                            ctx,
                            robotPose,
                            cap,
                            COLLECT_GOAL_UNITS,
                            collectBluePoseRef,
                            collectRoute));
                ctx.planner.clearCommitted();
                lastCat.set(cat);
                timingStartNs.set(0);
                timingStation.set(null);
              }

              if (cat == CategorySpec.kScore) {
                active.set(scoreGoal);
              } else {
                RepulsorSetpoint hp =
                    chooseCollect(
                        ctx, robotPose, cap, COLLECT_GOAL_UNITS, collectBluePoseRef, collectRoute);
                if (hp != null) active.set(hp);
              }

              ctx.planner.pollChosenSetpoint().ifPresent(active::set);

              RepulsorSetpoint sp = active.get();
              if (sp == null) {
                ctx.drive.runVelocity(new ChassisSpeeds());
                return;
              }

              RepulsorSetpoint prevGoal = lastEpisodeGoal.get();
              if (prevGoal == null) {
                lastEpisodeGoal.set(sp);
              } else if (prevGoal != sp) {
                finalizeEpisode.accept(false);
                lastEpisodeGoal.set(sp);
              }

              Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));

              ctx.repulsor.setCurrentGoal(sp);
              ctx.planner.setGoal(goalPose);

              double distToGoal = robotPose.getTranslation().getDistance(goalPose.getTranslation());
              long nowNs = System.nanoTime();

              if (episodeStartNs.get() == 0L) {
                episodeStartNs.set(nowNs);
                episodeBestDist.set(distToGoal);
                lastProgressNs.set(nowNs);
              } else {
                Double best = episodeBestDist.get();
                if (best == null || distToGoal < best - PROGRESS_EPS_METERS) {
                  episodeBestDist.set(distToGoal);
                  lastProgressNs.set(nowNs);
                }
              }

              if (distToGoal <= SUCCESS_NEAR_DIST_METERS) {
                episodeEverNearGoal.set(true);
              }

              if (cat == CategorySpec.kCollect) {
                if (atHPStation.get() && timingStartNs.get() == 0) {
                  timingStartNs.set(System.nanoTime());
                  timingStation.set(stationKeyFn.apply(sp));
                }
                if (hasPiece.get() && timingStartNs.get() != 0) {
                  double secs = (System.nanoTime() - timingStartNs.get()) / 1e9;
                  String key = timingStation.get();
                  if (key != null) {
                    MetricRecorder<Double> r = HPStationMetrics.recorder(key);
                    r.record(secs);
                  }
                  timingStartNs.set(0);
                  timingStation.set(null);
                }
              } else {
                timingStartNs.set(0);
                timingStation.set(null);
              }

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      ctx.coral_offset,
                      ctx.algae_offset,
                      cat,
                      false,
                      0.0);

              boolean pinned = byp.isPinnedMode();

              if (pinned) {
                if (pinnedEnterNs.get() == 0L) {
                  pinnedEnterNs.set(nowNs);
                  pinnedStartBestDist.set(episodeBestDist.get());
                  pinnedBestDist.set(episodeBestDist.get());
                  pinnedFailedThisLatch.set(false);
                } else {
                  Double currentBest = episodeBestDist.get();
                  if (currentBest != null) {
                    Double pinnedBest = pinnedBestDist.get();
                    if (pinnedBest == null || currentBest < pinnedBest) {
                      pinnedBestDist.set(currentBest);
                    }
                  }
                  long pinnedDur = nowNs - pinnedEnterNs.get();
                  Double startBest = pinnedStartBestDist.get();
                  Double bestSincePinned = pinnedBestDist.get();
                  if (!pinnedFailedThisLatch.get()
                      && startBest != null
                      && bestSincePinned != null) {
                    double improvement = startBest - bestSincePinned;
                    if (pinnedDur >= PINNED_FAIL_NS && improvement < PINNED_PROGRESS_MIN_METERS) {
                      episodeEverNearGoal.set(false);
                      finalizeEpisode.accept(false);
                      pinnedFailedThisLatch.set(true);
                    }
                  }
                }
              } else {
                pinnedEnterNs.set(0L);
                pinnedStartBestDist.set(null);
                pinnedBestDist.set(null);
                pinnedFailedThisLatch.set(false);
              }

              if (lastEpisodeGoal.get() != null && sp == lastEpisodeGoal.get()) {
                long lastProg = lastProgressNs.get();
                if (lastProg != 0L) {
                  long sinceProgressNs = nowNs - lastProg;
                  if (sinceProgressNs >= STUCK_FAIL_NS && distToGoal > STUCK_DIST_MIN_METERS) {
                    episodeEverNearGoal.set(false);
                    finalizeEpisode.accept(false);
                  }
                }
              }

              ctx.drive.runVelocity(
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation()));
            },
            ctx.drive)
        .finallyDo(
            interrupted -> {
              if (lastEpisodeGoal.get() != null) {
                finalizeEpisode.accept(!interrupted);
              }
              ctx.drive.runVelocity(new ChassisSpeeds());
            });
  }

  private RepulsorSetpoint initialScore(BehaviourContext ctx) {
    RepulsorSetpoint fromNT = nextScore.get();
    if (fromNT != null) return fromNT;
    RepulsorSetpoint pred = pickPredicted(ctx);
    return pred != null ? pred : fromNT;
  }

  private RepulsorSetpoint pickPredicted(BehaviourContext ctx) {
    Alliance alliance =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? Alliance.kBlue
            : Alliance.kRed;
    FieldTracker ft = FieldTracker.getInstance();
    ft.updatePredictorWorld(alliance);
    double cap = ourSpeedCap != null ? Math.max(0.1, ourSpeedCap.get()) : 3.5;
    List<PredictiveFieldState.Candidate> ranked =
        ft.getPredictedCandidates(
            alliance, ctx.robotPose.get().getTranslation(), cap, CategorySpec.kScore, 1);
    if (ranked == null || ranked.isEmpty()) return null;
    return ranked.get(0).setpoint;
  }

  private RepulsorSetpoint chooseCollect(
      BehaviourContext ctx,
      Pose2d robotPose,
      double cap,
      int goalUnits,
      AtomicReference<Pose2d> collectBluePoseRef,
      RepulsorSetpoint collectRoute) {

    edu.wpi.first.wpilibj.DriverStation.Alliance wpA =
        DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);

    Pose2d robotPoseBlue =
        wpA == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
            ? Setpoints.flipToRed(robotPose)
            : robotPose;

    Pose2d nextBlue =
        FieldTracker.getInstance().nextCollectionGoalBlue(robotPoseBlue, cap, goalUnits);

    if (nextBlue == null) {
      nextBlue =
          new Pose2d(
              Constants.FIELD_LENGTH * 0.5,
              Constants.FIELD_WIDTH * 0.5,
              robotPoseBlue.getRotation());
    }

    collectBluePoseRef.set(nextBlue);
    return collectRoute;
  }
}
