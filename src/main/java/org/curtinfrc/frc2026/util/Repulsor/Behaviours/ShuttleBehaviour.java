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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.ReactiveBypass;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointUtil;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;

public class ShuttleBehaviour extends Behaviour {
  private final int prio;
  private final Supplier<Boolean> hasPiece;
  private final Supplier<Double> ourSpeedCap;
  private Pose2d lastCollectBluePose = null;

  private final NetworkTablesValue<Long> pieceCount =
      NetworkTablesValue.ofInteger(NetworkTableInstance.getDefault(), "/PieceCount", 0L);

  public ShuttleBehaviour(int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
    this.prio = priority;
    this.hasPiece = hasPiece;
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
    return !flags.contains(BehaviourFlag.DEFENCE_MODE);
  }

  private static double shortestAngleRad(double from, double to) {
    return MathUtil.angleModulus(to - from);
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
        release,
        ctx.vision.getObstacles());
  }

  private Command buildShootReadyCommand(BehaviourContext ctx) {
    return Commands.runOnce(
            () -> {
              FieldTrackerCore.getInstance().resetAll();
            })
        .andThen(
            Commands.waitSeconds(2)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          pieceCount.set(0L);
                        })));
  }

  private boolean isReadyToShoot(
      Pose2d robotPose, Pose2d goalPose, boolean piece, CategorySpec cat) {
    if (!piece) return false;
    if (cat != CategorySpec.kScore) return false;
    if (goalPose == null) return false;
    boolean near = nearPose(robotPose, goalPose, 0.28, 10.0);
    if (!near) return false;
    return true;
  }

  private Command buildResetCommand() {
    return Commands.runOnce(
        () -> {
          System.out.println("AutoPathBehaviour: Resetting FieldTracker");
          FieldTrackerCore.getInstance().resetAll();
        });
  }

  private Command resetCommand = null;

  @Override
  public Command build(BehaviourContext ctx) {
    AtomicReference<RepulsorSetpoint> lastActive = new AtomicReference<>(null);
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

    final int COLLECT_GOAL_UNITS = 2;

    final double SHOOT_LOCK_ENTER_M = 3.0;
    final double SHOOT_LOCK_EXIT_M = 3.6;
    final double SHOOT_LOCK_MIN_ROT_DEG = 8.0;

    AtomicLong episodeStartNs = new AtomicLong(0L);
    AtomicReference<Double> episodeBestDist = new AtomicReference<>(null);
    AtomicLong lastProgressNs = new AtomicLong(0L);
    AtomicBoolean episodeEverNearGoal = new AtomicBoolean(false);

    AtomicLong pinnedEnterNs = new AtomicLong(0L);
    AtomicReference<Double> pinnedStartBestDist = new AtomicReference<>(null);
    AtomicReference<Double> pinnedBestDist = new AtomicReference<>(null);
    AtomicBoolean pinnedFailedThisLatch = new AtomicBoolean(false);

    ReactiveBypass byp = ctx.planner.bypass;

    AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint("COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
            HeightSetpoint.NONE);

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
              boolean piece = hasPiece.get();
              double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;
              CategorySpec cat = CategorySpec.kCollect;

              CategorySpec prevCat = lastCat.get();
              if (prevCat != null && prevCat != cat) {
                if (lastEpisodeGoal.get() != null) {
                  finalizeEpisode.accept(false);
                }
                ctx.planner.clearCommitted();
                lastActive.set(null);
                timingStartNs.set(0);
                timingStation.set(null);
              }
              lastCat.set(cat);

              if (cat == CategorySpec.kScore) {
                if (resetCommand != null && resetCommand.isScheduled()) {
                  resetCommand.cancel();
                }

              } else {
                if (resetCommand == null || (resetCommand != null && !resetCommand.isScheduled())) {
                  resetCommand = Commands.waitSeconds(5).andThen(buildResetCommand());

                  CommandScheduler.getInstance().schedule(resetCommand);
                }
                // Logger.recordOutput("Repulsor/Goal1", hp.get(makeCtx(ctx, robotPose)));
              }

              RepulsorSetpoint sp =
                  chooseCollect(
                      ctx, robotPose, cap, COLLECT_GOAL_UNITS, collectBluePoseRef, collectRoute);

              lastActive.set(sp);

              Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));

              ctx.repulsor.setCurrentGoal(sp);
              ctx.planner.setRequestedGoal(goalPose);

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

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
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

              // Logger.recordOutput("autopath_goal_x", goalPose.getX());
              // Logger.recordOutput("autopath_goal_y", goalPose.getY());
              // Logger.recordOutput("autopath_goal_theta", goalPose.getRotation().getRadians());
              // Logger.recordOutput("autopath_dist_to_goal", distToGoal);
              // Logger.recordOutput("autopath_shoot_lock", shootGoalLocked.get());

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

  private RepulsorSetpoint pickPredicted(BehaviourContext ctx) {
    Alliance alliance =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? Alliance.kBlue
            : Alliance.kRed;
    FieldTrackerCore ft = FieldTrackerCore.getInstance();
    ft.updatePredictorWorld(alliance);
    double cap = ourSpeedCap != null ? Math.max(0.1, ourSpeedCap.get()) : 3.5;
    List<Candidate> ranked =
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
            ? SetpointUtil.flipToRed(robotPose)
            : robotPose;

    Pose2d nextBlue =
        FieldTrackerCore.getInstance().nextCollectionGoalBlue(robotPoseBlue, cap, goalUnits);

    if (nextBlue == null) {
      nextBlue =
          new Pose2d(
              Constants.FIELD_LENGTH * 0.5,
              Constants.FIELD_WIDTH * 0.5,
              robotPoseBlue.getRotation());
    }

    // Rotation2d locked =
    //     (lastCollectBluePose != null)
    //         ? lastCollectBluePose.getRotation()
    //         : robotPoseBlue.getRotation();
    nextBlue = new Pose2d(nextBlue.getTranslation(), nextBlue.getRotation());

    lastCollectBluePose = nextBlue;
    collectBluePoseRef.set(nextBlue);

    return collectRoute;
  }
}
