// File: src/main/java/org/curtinfrc/frc2025/util/Repulsor/Behaviours/AutoPathBehaviour.java
package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Metrics.HPStationMetrics;
import org.curtinfrc.frc2026.util.Repulsor.Metrics.MetricRecorder;
import org.curtinfrc.frc2026.util.Repulsor.PredictiveFieldState;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

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

    AtomicLong episodeStartNs = new AtomicLong(0L);
    AtomicReference<Double> episodeBestDist = new AtomicReference<>(null);
    AtomicLong lastProgressNs = new AtomicLong(0L);
    AtomicBoolean episodeEverNearGoal = new AtomicBoolean(false);

    AtomicLong pinnedEnterNs = new AtomicLong(0L);
    AtomicReference<Double> pinnedStartBestDist = new AtomicReference<>(null);
    AtomicReference<Double> pinnedBestDist = new AtomicReference<>(null);
    AtomicBoolean pinnedFailedThisLatch = new AtomicBoolean(false);

    ReactiveBypass byp = ctx.planner.bypass;

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
              boolean scorePhase = inScoring.get();
              boolean collectPhase = inCollecting.get();

              boolean piece = hasPiece.get();
              CategorySpec cat = piece ? CategorySpec.kScore : CategorySpec.kCollect;

              if (!init.get()) {
                active.set(cat == CategorySpec.kScore ? initialScore(ctx) : chooseHP());
                lastCat.set(cat);
                init.set(true);
              }

              if (lastCat.get() != cat) {
                if (lastCat.get() != null && lastEpisodeGoal.get() != null) {
                  finalizeEpisode.accept(false);
                }
                active.set(cat == CategorySpec.kScore ? initialScore(ctx) : chooseHP());
                ctx.planner.clearCommitted();
                lastCat.set(cat);
                timingStartNs.set(0);
                timingStation.set(null);
              }

              if (cat == CategorySpec.kScore) {
                RepulsorSetpoint pred = pickPredicted(ctx);
                if (pred != null) active.set(pred);
                RepulsorSetpoint forced = nextScore.get();
                if (forced != null) active.set(forced);
              } else {
                RepulsorSetpoint hp = chooseHP();
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

              Pose2d robotPose = ctx.robotPose.get();

              double release = 0.0;
              try {
                release =
                    Math.max(0.0, ctx.repulsor.getTargetHeight().getHeight() == null ? 0.0 : 0.0);
              } catch (Exception ignored) {
                release = 0.0;
              }

              Pose2d goalPose =
                  sp.get(
                      new org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext(
                          java.util.Optional.of(robotPose),
                          Math.max(0.0, ctx.robot_x) * 2.0,
                          Math.max(0.0, ctx.robot_y) * 2.0,
                          ctx.coral_offset,
                          ctx.algae_offset,
                          release,
                          ctx.vision.getObstacles()));

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

  private RepulsorSetpoint chooseHP() {
    List<RepulsorSetpoint> opts = hpOptions.get();
    if (opts == null || opts.isEmpty()) return null;
    RepulsorSetpoint best = null;
    double bestScore = Double.POSITIVE_INFINITY;
    for (RepulsorSetpoint sp : opts) {
      String key = stationKeyFn.apply(sp);
      double mean = Double.POSITIVE_INFINITY;
      var r = HPStationMetrics.recorder(key);
      Double o = r.getOverall();
      if (o != null) mean = o;
      if (mean < bestScore) {
        bestScore = mean;
        best = sp;
      }
    }
    return best != null ? best : opts.get(0);
  }
}
