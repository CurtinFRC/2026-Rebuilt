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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

public class ReactiveBypassRuntime {
  private final ReactiveBypassConfig cfg;
  private final ReactiveBypassState state = new ReactiveBypassState();
  private final ReactiveBypassVibrationTracker vibration = new ReactiveBypassVibrationTracker();
  private final ReactiveBypassTelemetry telemetry = new ReactiveBypassTelemetry();

  public ReactiveBypassRuntime(ReactiveBypassConfig cfg) {
    this.cfg = cfg;
    ReactiveBypassConfigLoader.loadConfigFromYaml(this.cfg, ReactiveBypassConfig.class);
  }

  public void setConfig(java.util.function.Consumer<ReactiveBypassConfig> c) {
    c.accept(cfg);
  }

  public void enableLogging(String filePath) {
    telemetry.enableLogging(filePath);
  }

  public void disableLogging() {
    telemetry.disableLogging();
  }

  public void finalizeEpisode(boolean success) {
    telemetry.finalizeEpisode(success);
  }

  public void resetEpisodeMetrics() {
    telemetry.resetEpisodeMetrics();
  }

  public void reset() {
    state.reset();
    vibration.reset();
    telemetry.resetEpisodeMetrics();
  }

  public Optional<Pose2d> update(
      Pose2d pose,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double dtSeconds,
      double robotX,
      double robotY,
      List<? extends Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly,
      Function<String, Boolean> canRejoinOriginal) {

    state.timeSinceLatchS += dtSeconds;
    state.timeSinceEvalS += dtSeconds;
    state.timeSinceSideSwitchS += dtSeconds;
    state.sideConfidence = Math.max(0.0, state.sideConfidence - 0.5 * dtSeconds);
    state.pinnedCooldownS = Math.max(0.0, state.pinnedCooldownS - dtSeconds);
    if (state.pinnedMode) state.pinnedTimeS += dtSeconds;
    vibration.feedWindow(pose, headingTowardGoal, dtSeconds, cfg);

    boolean robotTouchDynamic =
        ReactiveBypassProbing.robotTouchDynamic(
            cfg, pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);

    if (state.latchedSubgoal != null) {
      boolean timeOk = state.timeSinceLatchS >= cfg.holdMinSeconds;
      boolean aheadClear =
          ReactiveBypassProbing.lookAheadClear(
              cfg,
              pose,
              headingTowardGoal,
              robotX,
              robotY,
              intersectsDynamicOnly,
              cfg.releaseAheadMeters,
              cfg.triggerWidthMeters);
      boolean rejoinOk = canRejoinOriginal.apply("rejoin");
      boolean progressOk =
          pose.getTranslation().getDistance(state.latchedAtPosition) >= cfg.minProgressMeters
              || state.timeSinceLatchS < 0.6;
      boolean localStuck =
          vibration.isVibrating(cfg)
              || vibration.isForwardStuck(cfg, state.lastOcc)
              || robotTouchDynamic;
      boolean allowRelatch = state.timeSinceLatchS >= cfg.holdMinSeconds * 1.5;
      if ((timeOk && aheadClear && rejoinOk) || state.timeSinceLatchS >= cfg.subgoalMaxSeconds) {
        state.latchedSubgoal = null;
        state.preferredSide = 0;
        state.lastChosenCost = null;
        state.consecutiveBypassFailures = 0;
      } else if ((!progressOk || localStuck) && allowRelatch) {
        int side;
        if (!dynamicObstacles.isEmpty()) {
          int freer =
              ReactiveBypassProbing.chooseFreerSide(
                  cfg,
                  state.sideConfidence,
                  state.preferredSide,
                  pose,
                  headingTowardGoal,
                  robotX,
                  robotY,
                  intersectsDynamicOnly);
          if (state.preferredSide != 0
              && freer != state.preferredSide
              && state.timeSinceSideSwitchS < cfg.sideStickSeconds) {
            side = state.preferredSide;
          } else {
            side = freer;
          }
        } else {
          side = Math.max(+1, state.preferredSide == 0 ? +1 : state.preferredSide);
        }
        double lat =
            ReactiveBypassMath.clamp(
                cfg.lateralMeters * cfg.escapeLateral, cfg.minLateralMeters, cfg.lateralMaxMeters);
        double fwd =
            ReactiveBypassMath.clamp(
                cfg.forwardMeters * cfg.escapeForward, cfg.forwardMeters, cfg.forwardMaxMeters);
        state.latchedSubgoal =
            ReactiveBypassWaypointPlanner.makeWaypoint(
                cfg, pose, headingTowardGoal, side, lat, fwd);
        state.lastChosenCost = null;
        state.timeSinceLatchS = Math.max(state.timeSinceLatchS, cfg.escapeHoldS);
        state.consecutiveBypassFailures++;
      } else {
        state.latchedSubgoal =
            ReactiveBypassWaypointPlanner.slewSubgoal(
                cfg, state.latchedSubgoal, state.latchedSubgoal, pose, dtSeconds);
        return Optional.of(
            ReactiveBypassWaypointPlanner.alignedPose(state.latchedSubgoal, headingTowardGoal));
      }
    }

    if (state.timeSinceEvalS < cfg.recalcSeconds) return Optional.empty();

    double headingErrDeg =
        Math.abs(
                ReactiveBypassMath.radDiff(
                    headingTowardGoal.getRadians(),
                    pose.getTranslation()
                        .minus(goal.getTranslation())
                        .unaryMinus()
                        .getAngle()
                        .getRadians()))
            * 180.0
            / Math.PI;

    double occ =
        ReactiveBypassProbing.corridorOcc(
            cfg, pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);
    state.lastOcc = occ;

    boolean forwardStuck = vibration.isForwardStuck(cfg, state.lastOcc);
    boolean vibStuck = vibration.isVibratingWithOccBoost(cfg, state.lastOcc) || forwardStuck;
    boolean blocked =
        ReactiveBypassProbing.hysteresisBlocked(cfg, state.latchedSubgoal != null, occ)
            || vibStuck
            || robotTouchDynamic;
    boolean headingOk =
        headingErrDeg >= cfg.minHeadingErrToTriggerDeg || vibStuck || robotTouchDynamic;

    if (state.latchedSubgoal == null && blocked) {
      state.blockedAccumS += dtSeconds;
    } else if (!blocked) {
      state.blockedAccumS = 0.0;
    }

    Rotation2d directGoalHeading = goal.getTranslation().minus(pose.getTranslation()).getAngle();
    telemetry.logStep(pose, goal, dtSeconds, blocked, robotTouchDynamic, state.pinnedMode);

    if (state.pinnedMode) {
      boolean canCheckExit = state.pinnedTimeS >= cfg.pinnedMinTimeSeconds;
      boolean exitPinned =
          (canCheckExit
                  && (!blocked || state.lastOcc < cfg.pinnedOccMin * 0.6 || !robotTouchDynamic))
              || state.pinnedTimeS >= cfg.pinnedMaxTimeSeconds;
      if (exitPinned) {
        state.pinnedMode = false;
        state.pinnedCooldownS = cfg.pinnedCooldownSeconds;
        state.pinnedTimeS = 0.0;
      } else {
        state.timeSinceEvalS = 0.0;
        state.latchedSubgoal = null;
        return Optional.of(buildPinnedPushPose(pose, headingTowardGoal, directGoalHeading, true));
      }
    } else {
      boolean pinnedStill =
          vibStuck
              || vibration.isVibrating(cfg)
              || vibration.isForwardStuck(cfg, state.lastOcc)
              || robotTouchDynamic;
      boolean occHighEnough = state.lastOcc >= cfg.pinnedOccMin || robotTouchDynamic;
      boolean canEnterPinned =
          state.latchedSubgoal == null
              && blocked
              && pinnedStill
              && occHighEnough
              && state.blockedAccumS >= cfg.pinnedMinTimeSeconds
              && state.pinnedCooldownS <= 0.0;
      if (canEnterPinned) {
        state.pinnedMode = true;
        state.pinnedTimeS = 0.0;
        state.pinnedHeading =
            Double.isNaN(directGoalHeading.getRadians()) ? headingTowardGoal : directGoalHeading;
        state.timeSinceEvalS = 0.0;
        state.latchedSubgoal = null;
        return Optional.of(buildPinnedPushPose(pose, headingTowardGoal, directGoalHeading, false));
      }
    }

    if (!blocked || !headingOk) {
      state.timeSinceEvalS = 0.0;
      return Optional.empty();
    }

    int biasSide =
        ReactiveBypassProbing.chooseFreerSide(
            cfg,
            state.sideConfidence,
            state.preferredSide,
            pose,
            headingTowardGoal,
            robotX,
            robotY,
            intersectsDynamicOnly);

    List<Pose2d> candidates =
        ReactiveBypassWaypointPlanner.generateCandidates(
            cfg, state.lastOcc, state.preferredSide, pose, headingTowardGoal, biasSide, goal);

    ReactiveBypassScore best = null;
    boolean stuckNowForSwitchPenalty =
        vibration.isVibrating(cfg) || vibration.isForwardStuck(cfg, state.lastOcc);
    for (Pose2d wp : candidates) {
      ReactiveBypassScore s =
          ReactiveBypassScorer.scoreCandidate(
              cfg,
              state.preferredSide,
              state.timeSinceSideSwitchS,
              stuckNowForSwitchPenalty,
              pose,
              wp,
              goal,
              headingTowardGoal,
              robotX,
              robotY,
              dynamicObstacles,
              intersectsDynamicOnly);
      if (!(s.okLeg1 && s.okLeg2)) continue;
      if (best == null || s.totalCost < best.totalCost) best = s;
    }

    if (best == null) {
      state.consecutiveBypassFailures++;
      state.timeSinceEvalS = 0.0;
      if (state.consecutiveBypassFailures >= 2) {
        int side =
            ReactiveBypassProbing.chooseFreerSide(
                cfg,
                state.sideConfidence,
                state.preferredSide,
                pose,
                headingTowardGoal,
                robotX,
                robotY,
                intersectsDynamicOnly);
        double lat =
            ReactiveBypassMath.clamp(
                cfg.lateralMeters * cfg.escapeLateral * 1.4,
                cfg.minLateralMeters,
                cfg.lateralMaxMeters);
        double fwd =
            ReactiveBypassMath.clamp(
                cfg.forwardMeters * cfg.escapeForward * 1.2,
                cfg.forwardMeters,
                cfg.forwardMaxMeters);
        state.latchedSubgoal =
            ReactiveBypassWaypointPlanner.makeWaypoint(
                cfg, pose, headingTowardGoal, side, lat, fwd);
        state.latchedAtPosition = pose.getTranslation();
        state.timeSinceLatchS = 0.0;
        state.lastChosenCost = null;
        return Optional.of(
            ReactiveBypassWaypointPlanner.alignedPose(state.latchedSubgoal, headingTowardGoal));
      }
      return Optional.empty();
    }

    if (state.lastChosenCost != null) {
      double improve =
          (state.lastChosenCost - best.totalCost) / Math.max(1e-6, state.lastChosenCost);
      if (improve < cfg.relatchImproveFrac && state.latchedSubgoal != null) {
        state.timeSinceEvalS = 0.0;
        state.latchedSubgoal =
            ReactiveBypassWaypointPlanner.slewSubgoal(
                cfg, state.latchedSubgoal, state.latchedSubgoal, pose, dtSeconds);
        return Optional.of(
            ReactiveBypassWaypointPlanner.alignedPose(state.latchedSubgoal, headingTowardGoal));
      }
    }

    state.latchedSubgoal = best.wp;
    state.lastChosenCost = best.totalCost;
    state.latchedAtPosition = pose.getTranslation();
    state.timeSinceLatchS = 0.0;
    state.timeSinceEvalS = 0.0;
    state.consecutiveBypassFailures = 0;
    int chosenSide = ReactiveBypassMath.sideOf(pose, headingTowardGoal, best.wp.getTranslation());
    if (state.preferredSide != chosenSide && state.timeSinceSideSwitchS >= cfg.sideStickSeconds) {
      state.preferredSide = chosenSide;
      state.timeSinceSideSwitchS = 0.0;
      state.sideConfidence = 1.0;
    }
    state.latchedSubgoal =
        ReactiveBypassWaypointPlanner.slewSubgoal(
            cfg, state.latchedSubgoal, state.latchedSubgoal, pose, dtSeconds);
    return Optional.of(
        ReactiveBypassWaypointPlanner.alignedPose(state.latchedSubgoal, headingTowardGoal));
  }

  public boolean isPinnedMode() {
    return state.pinnedMode;
  }

  private Pose2d buildPinnedPushPose(
      Pose2d pose,
      Rotation2d headingTowardGoal,
      Rotation2d directGoalHeading,
      boolean useDirectFallback) {
    double fwd =
        ReactiveBypassMath.clamp(
            cfg.forwardMeters * cfg.escapeForward * 1.4,
            cfg.forwardMeters,
            cfg.forwardMaxMeters * 1.3);
    Rotation2d pushHeading = state.pinnedHeading;
    if (Double.isNaN(pushHeading.getRadians())) {
      if (useDirectFallback) {
        pushHeading =
            Double.isNaN(directGoalHeading.getRadians()) ? headingTowardGoal : directGoalHeading;
      } else {
        pushHeading = headingTowardGoal;
      }
    }
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, pushHeading);
    Translation2d tgt = ReactiveBypassWaypointPlanner.clampToField(cfg, p.plus(dir.times(fwd)));
    Pose2d wp = new Pose2d(tgt, pushHeading);
    return ReactiveBypassWaypointPlanner.alignedPose(wp, pushHeading);
  }
}
