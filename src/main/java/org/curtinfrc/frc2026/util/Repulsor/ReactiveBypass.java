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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Function;

public class ReactiveBypass {
  public static class Config {
    public double fieldLen = Constants.FIELD_LENGTH;
    public double fieldWid = Constants.FIELD_WIDTH;
    public double inflationMeters = 0.10;
    public double triggerAheadMeters = 1.2;
    public double triggerWidthMeters = 0.7;
    public int corridorSamples = 15;
    public double occHigh = 0.2079111216823982;
    public double occLow = 0.08402471544016843;
    public double lateralMeters = 0.50;
    public double lateralMaxMeters = 1.90;
    public double minLateralMeters = 0.55;
    public double forwardMeters = 1.4219386889821075;
    public double forwardMaxMeters = 2.50;
    public double holdMinSeconds = 0.45;
    public double recalcSeconds = 0.18;
    public double subgoalMaxSeconds = 2.0;
    public double minHeadingErrToTriggerDeg = 6.0;
    public double releaseAheadMeters = 1.35;
    public double angleCostWeight = 0.6538189117352257;
    public double curvatureWeight = 0.5598688771152425;
    public double wallPenaltyGain = 0.55;
    public double occCostGain = 2;
    public double sideSwitchPenalty = 0.35;
    public double headingDeadbandDeg = 4.0;
    public double minProgressMeters = 0.08;
    public double probeOccLegStep = 0.25;
    public double sideBiasProbeAngleDeg = 10.0;
    public double sideStickSeconds = 0.70;
    public double subgoalSlewRateMps = 2.0;
    public double subgoalJitterMeters = 0.04;
    public double arcAngleDeg = 18.0;
    public double arcRMin = 0.9;
    public double arcRMax = 1.8;
    public int arcRadialSteps = 2;
    public int arcAngularStepsPerSide = 2;
    public double relatchImproveFrac = 0.12;
    public double zzzMaxYawDeltaDeg = 22.0;
    public double zzzPenalty = 0.40;
    public double vibWindowS = 0.6;
    public double vibMinDisp = 0.10;
    public int vibMaxDirFlips = 8;
    public double escapeForward = 1.60;
    public double escapeLateral = 1.30;
    public double escapeHoldS = 0.60;
    public double escapeOccBoost = 0.25;
    public double progressCostWeight = 0.05;
    public double minForwardProgressMeters = 0.50;
    public double nearGoalDistMeters = 3.0;
    public double nearGoalForwardScale = 0.60;
    public double nearGoalLateralScale = 0.70;
    public double stuckMinForwardProgress = 0.15;
    public double stuckOccMin = 0.30;
    public double stuckLookbackFrac = 0.7;
    public double sideSwitchStuckPenaltyScale = 0.25;
    public double pinnedOccMin = 0.40;
    public double pinnedMinTimeSeconds = 0.35;
    public double pinnedMaxTimeSeconds = 3.0;
    public double pinnedCooldownSeconds = 0.70;
    public double cornerWallThresh = 3.0;
    public double cornerEscapeLatBoost = 1.8;
    public double cornerEscapeFwdBoost = 1.4;
    public double cornerEscapeBlend = 0.65;
    public double cornerRewardGain = 0.6;
  }

  private final Config cfg = new Config();
  private Pose2d latchedSubgoal = null;
  private Translation2d latchedAtPosition = Translation2d.kZero;
  private double timeSinceLatchS = 1e9;
  private double timeSinceEvalS = 1e9;
  private double timeSinceSideSwitchS = 1e9;
  private int preferredSide = 0;
  private double lastOcc = 0.0;
  private Double lastChosenCost = null;
  private double sideConfidence = 0.0;
  private final ArrayDeque<Sample> vib = new ArrayDeque<>();
  private double vibAccumTime = 0.0;
  private boolean pinnedMode = false;
  private double pinnedTimeS = 0.0;
  private double blockedAccumS = 0.0;
  private double pinnedCooldownS = 0.0;
  private Rotation2d pinnedHeading = Rotation2d.kZero;
  private boolean loggingEnabled = false;
  private String logFilePath = "ReactiveBypassLog.csv";
  private boolean logHeaderWritten = false;
  private double logTimeS = 0.0;
  private double logForwardProgressM = 0.0;
  private double logBlockedTimeS = 0.0;
  private double logPinnedTimeS = 0.0;
  private int logCollisions = 0;
  private Translation2d logStartPos = Translation2d.kZero;
  private Translation2d logGoalPos = Translation2d.kZero;
  private boolean logHasStart = false;
  private boolean lastTouchDynamic = false;
  private int consecutiveBypassFailures = 0;

  public ReactiveBypass() {
    loadConfigFromYaml();
  }

  public void setConfig(java.util.function.Consumer<Config> c) {
    c.accept(cfg);
  }

  public void enableLogging(String filePath) {
    this.logFilePath = filePath;
    this.loggingEnabled = true;
  }

  public void disableLogging() {
    this.loggingEnabled = false;
  }

  public void finalizeEpisode(boolean success) {
    if (!loggingEnabled || !logHasStart) return;
    double startToGoalMeters = logStartPos.getDistance(logGoalPos);
    String line =
        String.format(
            Locale.US,
            "%.3f,%.3f,%.3f,%.3f,%d,%b,%.3f",
            logTimeS,
            logForwardProgressM,
            logBlockedTimeS,
            logPinnedTimeS,
            logCollisions,
            success,
            startToGoalMeters);
    appendLogLine(line);
    resetEpisodeMetrics();
  }

  public void resetEpisodeMetrics() {
    logTimeS = 0.0;
    logForwardProgressM = 0.0;
    logBlockedTimeS = 0.0;
    logPinnedTimeS = 0.0;
    logCollisions = 0;
    logStartPos = Translation2d.kZero;
    logGoalPos = Translation2d.kZero;
    logHasStart = false;
    lastTouchDynamic = false;
  }

  private void appendLogLine(String line) {
    // Logger.recordOutput("Repulsor/Bypass/LoggingEnabled", loggingEnabled);
    if (!loggingEnabled) return;
    try {
      Path path = Paths.get(logFilePath);
      if (!logHeaderWritten) {
        String header =
            "duration_s,forward_progress_m,blocked_time_s,pinned_time_s,collisions,success,start_to_goal_m";
        if (!Files.exists(path) || Files.size(path) == 0L) {
          Files.write(
              path,
              (header + System.lineSeparator()).getBytes(StandardCharsets.UTF_8),
              StandardOpenOption.CREATE,
              StandardOpenOption.APPEND);
        }
        logHeaderWritten = true;
      }
      Files.write(
          path,
          (line + System.lineSeparator()).getBytes(StandardCharsets.UTF_8),
          StandardOpenOption.CREATE,
          StandardOpenOption.APPEND);
    } catch (IOException ignored) {
      System.err.println("ReactiveBypass: Failed to write log line: " + ignored);
    }
  }

  private void logStep(
      Pose2d pose,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double dtSeconds,
      boolean blockedNow,
      boolean touchDynamic) {
    if (!loggingEnabled) return;
    logTimeS += dtSeconds;
    if (!logHasStart) {
      logStartPos = pose.getTranslation();
      logGoalPos = goal.getTranslation();
      logForwardProgressM = 0.0;
      logHasStart = true;
    }
    Translation2d startToGoal = logGoalPos.minus(logStartPos);
    double denom = startToGoal.getNorm();
    if (denom > 1e-6) {
      Translation2d startToCur = pose.getTranslation().minus(logStartPos);
      double proj =
          (startToCur.getX() * startToGoal.getX() + startToCur.getY() * startToGoal.getY()) / denom;
      if (proj > logForwardProgressM) {
        logForwardProgressM = proj;
      }
    }
    if (blockedNow) {
      logBlockedTimeS += dtSeconds;
    }
    if (pinnedMode) {
      logPinnedTimeS += dtSeconds;
    }
    if (touchDynamic && !lastTouchDynamic) {
      logCollisions++;
    }
    lastTouchDynamic = touchDynamic;
  }

  public void reset() {
    latchedSubgoal = null;
    latchedAtPosition = Translation2d.kZero;
    timeSinceLatchS = 1e9;
    timeSinceEvalS = 1e9;
    timeSinceSideSwitchS = 1e9;
    preferredSide = 0;
    lastOcc = 0.0;
    lastChosenCost = null;
    sideConfidence = 0.0;
    vib.clear();
    vibAccumTime = 0.0;
    pinnedMode = false;
    pinnedTimeS = 0.0;
    blockedAccumS = 0.0;
    pinnedCooldownS = 0.0;
    pinnedHeading = Rotation2d.kZero;
    consecutiveBypassFailures = 0;
    resetEpisodeMetrics();
  }

  public Optional<Pose2d> update(
      Pose2d pose,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double dtSeconds,
      double robotX,
      double robotY,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly,
      Function<String, Boolean> canRejoinOriginal) {

    timeSinceLatchS += dtSeconds;
    timeSinceEvalS += dtSeconds;
    timeSinceSideSwitchS += dtSeconds;
    sideConfidence = Math.max(0.0, sideConfidence - 0.5 * dtSeconds);
    pinnedCooldownS = Math.max(0.0, pinnedCooldownS - dtSeconds);
    if (pinnedMode) pinnedTimeS += dtSeconds;
    feedVibrationWindow(pose, headingTowardGoal, dtSeconds);
    boolean robotTouchDynamic = false;
    {
      double rx = robotX + 2.0 * cfg.inflationMeters;
      double ry = robotY + 2.0 * cfg.inflationMeters;
      Translation2d[] rect =
          FieldPlanner.robotRect(pose.getTranslation(), headingTowardGoal, rx, ry);
      robotTouchDynamic = intersectsDynamicOnly.apply(rect);
    }
    if (latchedSubgoal != null) {
      boolean timeOk = timeSinceLatchS >= cfg.holdMinSeconds;
      boolean aheadClear =
          lookAheadClear(
              pose,
              headingTowardGoal,
              robotX,
              robotY,
              intersectsDynamicOnly,
              cfg.releaseAheadMeters,
              cfg.triggerWidthMeters);
      boolean rejoinOk = canRejoinOriginal.apply("rejoin");
      boolean progressOk =
          pose.getTranslation().getDistance(latchedAtPosition) >= cfg.minProgressMeters
              || timeSinceLatchS < 0.6;
      boolean localStuck = isVibrating() || isForwardStuck() || robotTouchDynamic;
      boolean allowRelatch = timeSinceLatchS >= cfg.holdMinSeconds * 1.5;
      if ((timeOk && aheadClear && rejoinOk) || timeSinceLatchS >= cfg.subgoalMaxSeconds) {
        latchedSubgoal = null;
        preferredSide = 0;
        lastChosenCost = null;
        consecutiveBypassFailures = 0;
      } else if ((!progressOk || localStuck) && allowRelatch) {
        int side;
        if (!dynamicObstacles.isEmpty()) {
          int freer =
              chooseFreerSide(pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);
          if (preferredSide != 0
              && freer != preferredSide
              && timeSinceSideSwitchS < cfg.sideStickSeconds) {
            side = preferredSide;
          } else {
            side = freer;
          }
        } else {
          side = Math.max(+1, preferredSide == 0 ? +1 : preferredSide);
        }
        double lat =
            clamp(
                cfg.lateralMeters * cfg.escapeLateral, cfg.minLateralMeters, cfg.lateralMaxMeters);
        double fwd =
            clamp(cfg.forwardMeters * cfg.escapeForward, cfg.forwardMeters, cfg.forwardMaxMeters);
        latchedSubgoal = makeWaypoint(pose, headingTowardGoal, side, lat, fwd);
        lastChosenCost = null;
        timeSinceLatchS = Math.max(timeSinceLatchS, cfg.escapeHoldS);
        consecutiveBypassFailures++;
      } else {
        latchedSubgoal = slewSubgoal(latchedSubgoal, pose, dtSeconds);
        return Optional.of(alignedPose(latchedSubgoal, headingTowardGoal));
      }
    }
    if (timeSinceEvalS < cfg.recalcSeconds) return Optional.empty();
    double headingErrDeg =
        Math.abs(
                radDiff(
                    headingTowardGoal.getRadians(),
                    pose.getTranslation()
                        .minus(goal.getTranslation())
                        .unaryMinus()
                        .getAngle()
                        .getRadians()))
            * 180.0
            / Math.PI;
    double occ = corridorOcc(pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);
    lastOcc = occ;
    boolean forwardStuck = isForwardStuck();
    boolean vibStuck = isVibratingWithOccBoost() || forwardStuck;
    boolean blocked = hysteresisBlocked(occ) || vibStuck || robotTouchDynamic;
    boolean headingOk =
        headingErrDeg >= cfg.minHeadingErrToTriggerDeg || vibStuck || robotTouchDynamic;
    if (latchedSubgoal == null && blocked) {
      blockedAccumS += dtSeconds;
    } else if (!blocked) {
      blockedAccumS = 0.0;
    }
    Rotation2d directGoalHeading = goal.getTranslation().minus(pose.getTranslation()).getAngle();
    logStep(pose, goal, headingTowardGoal, dtSeconds, blocked, robotTouchDynamic);
    if (pinnedMode) {
      boolean canCheckExit = pinnedTimeS >= cfg.pinnedMinTimeSeconds;
      boolean exitPinned =
          (canCheckExit && (!blocked || lastOcc < cfg.pinnedOccMin * 0.6 || !robotTouchDynamic))
              || pinnedTimeS >= cfg.pinnedMaxTimeSeconds;
      if (exitPinned) {
        pinnedMode = false;
        pinnedCooldownS = cfg.pinnedCooldownSeconds;
        pinnedTimeS = 0.0;
      } else {
        timeSinceEvalS = 0.0;
        double fwd =
            clamp(
                cfg.forwardMeters * cfg.escapeForward * 1.4,
                cfg.forwardMeters,
                cfg.forwardMaxMeters * 1.3);
        Rotation2d pushHeading = pinnedHeading;
        if (Double.isNaN(pushHeading.getRadians()))
          pushHeading =
              Double.isNaN(directGoalHeading.getRadians()) ? headingTowardGoal : directGoalHeading;
        Translation2d p = pose.getTranslation();
        Translation2d dir = new Translation2d(1.0, pushHeading);
        Translation2d tgt = clampToField(p.plus(dir.times(fwd)));
        Pose2d wp = new Pose2d(tgt, pushHeading);
        latchedSubgoal = null;
        return Optional.of(alignedPose(wp, pushHeading));
      }
    } else {
      boolean pinnedStill = vibStuck || isVibrating() || isForwardStuck() || robotTouchDynamic;
      boolean occHighEnough = lastOcc >= cfg.pinnedOccMin || robotTouchDynamic;
      boolean canEnterPinned =
          latchedSubgoal == null
              && blocked
              && pinnedStill
              && occHighEnough
              && blockedAccumS >= cfg.pinnedMinTimeSeconds
              && pinnedCooldownS <= 0.0;
      if (canEnterPinned) {
        pinnedMode = true;
        pinnedTimeS = 0.0;
        pinnedHeading =
            Double.isNaN(directGoalHeading.getRadians()) ? headingTowardGoal : directGoalHeading;
        timeSinceEvalS = 0.0;
        double fwd =
            clamp(
                cfg.forwardMeters * cfg.escapeForward * 1.4,
                cfg.forwardMeters,
                cfg.forwardMaxMeters * 1.3);
        Rotation2d pushHeading = pinnedHeading;
        if (Double.isNaN(pushHeading.getRadians())) pushHeading = headingTowardGoal;
        Translation2d p = pose.getTranslation();
        Translation2d dir = new Translation2d(1.0, pushHeading);
        Translation2d tgt = clampToField(p.plus(dir.times(fwd)));
        Pose2d wp = new Pose2d(tgt, pushHeading);
        latchedSubgoal = null;
        return Optional.of(alignedPose(wp, pushHeading));
      }
    }
    if (!blocked || !headingOk) {
      timeSinceEvalS = 0.0;
      return Optional.empty();
    }
    int biasSide = chooseFreerSide(pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);
    List<Pose2d> candidates = generateCandidates(pose, headingTowardGoal, biasSide, goal);
    BypassScore best = null;
    for (Pose2d wp : candidates) {
      BypassScore s =
          scoreCandidate(
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
      consecutiveBypassFailures++;
      timeSinceEvalS = 0.0;
      if (consecutiveBypassFailures >= 2) {
        int side = chooseFreerSide(pose, headingTowardGoal, robotX, robotY, intersectsDynamicOnly);
        double lat =
            clamp(
                cfg.lateralMeters * cfg.escapeLateral * 1.4,
                cfg.minLateralMeters,
                cfg.lateralMaxMeters);
        double fwd =
            clamp(
                cfg.forwardMeters * cfg.escapeForward * 1.2,
                cfg.forwardMeters,
                cfg.forwardMaxMeters);
        latchedSubgoal = makeWaypoint(pose, headingTowardGoal, side, lat, fwd);
        latchedAtPosition = pose.getTranslation();
        timeSinceLatchS = 0.0;
        lastChosenCost = null;
        return Optional.of(alignedPose(latchedSubgoal, headingTowardGoal));
      }
      return Optional.empty();
    }
    if (lastChosenCost != null) {
      double improve = (lastChosenCost - best.totalCost) / Math.max(1e-6, lastChosenCost);
      if (improve < cfg.relatchImproveFrac && latchedSubgoal != null) {
        timeSinceEvalS = 0.0;
        latchedSubgoal = slewSubgoal(latchedSubgoal, pose, dtSeconds);
        return Optional.of(alignedPose(latchedSubgoal, headingTowardGoal));
      }
    }
    latchedSubgoal = best.wp;
    lastChosenCost = best.totalCost;
    latchedAtPosition = pose.getTranslation();
    timeSinceLatchS = 0.0;
    timeSinceEvalS = 0.0;
    consecutiveBypassFailures = 0;
    int chosenSide = sideOf(pose, headingTowardGoal, best.wp.getTranslation());
    if (preferredSide != chosenSide && timeSinceSideSwitchS >= cfg.sideStickSeconds) {
      preferredSide = chosenSide;
      timeSinceSideSwitchS = 0.0;
      sideConfidence = 1.0;
    }
    latchedSubgoal = slewSubgoal(latchedSubgoal, pose, dtSeconds);
    return Optional.of(alignedPose(latchedSubgoal, headingTowardGoal));
  }

  private static final class Sample {
    final Translation2d pos;
    final double sPara;
    final double sPerp;
    final int signPara;
    final int signPerp;
    final double dt;

    Sample(Translation2d pos, double sPara, double sPerp, int signPara, int signPerp, double dt) {
      this.pos = pos;
      this.sPara = sPara;
      this.sPerp = sPerp;
      this.signPara = signPara;
      this.signPerp = signPerp;
      this.dt = dt;
    }
  }

  private void feedVibrationWindow(Pose2d pose, Rotation2d heading, double dt) {
    Translation2d p = pose.getTranslation();
    double cos = Math.cos(heading.getRadians());
    double sin = Math.sin(heading.getRadians());
    double sPara = p.getX() * cos + p.getY() * sin;
    double sPerp = -p.getX() * sin + p.getY() * cos;
    int signPara = 0;
    int signPerp = 0;
    if (!vib.isEmpty()) {
      Sample last = vib.getLast();
      double dPara = sPara - last.sPara;
      double dPerp = sPerp - last.sPerp;
      if (Math.abs(dPara) > 1e-4) signPara = dPara > 0 ? +1 : -1;
      if (Math.abs(dPerp) > 1e-4) signPerp = dPerp > 0 ? +1 : -1;
    }
    vib.addLast(new Sample(p, sPara, sPerp, signPara, signPerp, dt));
    vibAccumTime += dt;
    while (!vib.isEmpty() && vibAccumTime > cfg.vibWindowS) {
      vibAccumTime -= vib.removeFirst().dt;
    }
  }

  private boolean isVibrating() {
    if (vib.isEmpty()) return false;
    double pMin = vib.getFirst().sPara, pMax = pMin;
    double lMin = vib.getFirst().sPerp, lMax = lMin;
    int flipsPara = 0, flipsPerp = 0;
    int lastPara = 0, lastPerp = 0;
    for (Sample s : vib) {
      pMin = Math.min(pMin, s.sPara);
      pMax = Math.max(pMax, s.sPara);
      lMin = Math.min(lMin, s.sPerp);
      lMax = Math.max(lMax, s.sPerp);
      if (s.signPara != 0 && lastPara != 0 && s.signPara != lastPara) flipsPara++;
      if (s.signPerp != 0 && lastPerp != 0 && s.signPerp != lastPerp) flipsPerp++;
      if (s.signPara != 0) lastPara = s.signPara;
      if (s.signPerp != 0) lastPerp = s.signPerp;
    }
    double dispPara = pMax - pMin;
    double dispPerp = lMax - lMin;
    double disp = Math.hypot(dispPara, dispPerp);
    int flips = Math.max(flipsPara, flipsPerp);
    return disp < cfg.vibMinDisp && flips >= cfg.vibMaxDirFlips;
  }

  private boolean isForwardStuck() {
    if (vib.isEmpty()) return false;
    double requiredWindow = cfg.vibWindowS * cfg.stuckLookbackFrac;
    if (vibAccumTime < requiredWindow) return false;
    Sample first = vib.getFirst();
    Sample last = vib.getLast();
    double forwardDisp = Math.abs(last.sPara - first.sPara);
    return forwardDisp < cfg.stuckMinForwardProgress && lastOcc >= cfg.stuckOccMin;
  }

  private boolean isVibratingWithOccBoost() {
    if (!isVibrating()) return false;
    return lastOcc >= Math.max(0.0, cfg.occHigh - cfg.escapeOccBoost);
  }

  private boolean hysteresisBlocked(double occNow) {
    if (latchedSubgoal != null) return occNow >= cfg.occLow;
    return occNow >= cfg.occHigh;
  }

  private double corridorOcc(
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d side = new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg));
    int hit = 0, tot = 0;
    for (int i = 1; i <= cfg.corridorSamples; i++) {
      double f = (cfg.triggerAheadMeters * i) / cfg.corridorSamples;
      for (int s = -1; s <= 1; s += 2) {
        double w = s * cfg.triggerWidthMeters * 0.5;
        Translation2d probe = p.plus(dir.times(f)).plus(side.times(w));
        Translation2d[] rect = FieldPlanner.robotRect(probe, heading, rx, ry);
        if (intersectsDynamicOnly.apply(rect)) hit++;
        tot++;
      }
    }
    return (tot == 0) ? 0.0 : ((double) hit / (double) tot);
  }

  private boolean lookAheadClear(
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly,
      double aheadMeters,
      double widthMeters) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d side = new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg));
    int samples = Math.max(4, cfg.corridorSamples / 2);
    for (int i = 1; i <= samples; i++) {
      double f = (aheadMeters * i) / samples;
      for (int s = -1; s <= 1; s += 2) {
        double w = s * widthMeters * 0.5;
        Translation2d probe = p.plus(dir.times(f)).plus(side.times(w));
        Translation2d[] rect = FieldPlanner.robotRect(probe, heading, rx, ry);
        if (intersectsDynamicOnly.apply(rect)) return false;
      }
    }
    return true;
  }

  private double localOccAt(
      Translation2d center,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d[] rect = FieldPlanner.robotRect(center, heading, rx, ry);
    return intersectsDynamicOnly.apply(rect) ? 1.0 : 0.0;
  }

  private int chooseFreerSide(
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double ang = Math.toRadians(cfg.sideBiasProbeAngleDeg);
    Rotation2d leftHead = Rotation2d.fromRadians(heading.getRadians() + ang);
    Rotation2d rightHead = Rotation2d.fromRadians(heading.getRadians() - ang);
    double leftOcc = corridorOcc(pose, leftHead, robotX, robotY, intersectsDynamicOnly);
    double rightOcc = corridorOcc(pose, rightHead, robotX, robotY, intersectsDynamicOnly);
    double diff = leftOcc - rightOcc;
    if (Math.abs(diff) < 0.04)
      return (sideConfidence > 0.2 && preferredSide != 0) ? preferredSide : +1;
    return (diff < 0) ? +1 : -1;
  }

  private List<Pose2d> generateCandidates(
      Pose2d pose, Rotation2d heading, int biasSide, Pose2d goal) {
    double distToGoal = pose.getTranslation().getDistance(goal.getTranslation());
    double nearNorm = clamp01(distToGoal / Math.max(1e-6, cfg.nearGoalDistMeters));
    double fwdScaleGoal = lerp(cfg.nearGoalForwardScale, 1.0, nearNorm);
    double latScaleGoal = lerp(cfg.nearGoalLateralScale, 1.0, nearNorm);
    double baseLat = Math.max(cfg.minLateralMeters, cfg.lateralMeters * latScaleGoal);
    double baseFwd = cfg.forwardMeters * fwdScaleGoal;
    double occScale = clamp01((lastOcc - cfg.occLow) / Math.max(1e-6, (cfg.occHigh - cfg.occLow)));
    double latBoost = 1.0 + 0.6 * occScale;
    double fwdBoost = 1.0 + 0.4 * occScale;
    double robotWall = wallPenalty(pose.getTranslation());
    boolean nearWall = robotWall > cfg.cornerWallThresh;
    if (nearWall) {
      latBoost *= cfg.cornerEscapeLatBoost;
      fwdBoost *= cfg.cornerEscapeFwdBoost;
    }
    double[] latScales = new double[] {1.0, 1.25, 1.5, latBoost};
    double[] fwdScales = new double[] {1.0, 1.25, fwdBoost};
    ArrayList<Pose2d> out = new ArrayList<>();
    Rotation2d[] headings;
    if (nearWall) {
      Rotation2d wallNormal = nearestWallNormal(pose.getTranslation());
      double blend = clamp01(cfg.cornerEscapeBlend);
      double ax = Math.cos(heading.getRadians());
      double ay = Math.sin(heading.getRadians());
      double bx = Math.cos(wallNormal.getRadians());
      double by = Math.sin(wallNormal.getRadians());
      double cx = blend * ax + (1.0 - blend) * bx;
      double cy = blend * ay + (1.0 - blend) * by;
      Rotation2d escapeHeading = new Rotation2d(cx, cy);
      if (Math.abs(radDiff(heading.getRadians(), escapeHeading.getRadians()))
          < Math.toRadians(2.0)) {
        headings = new Rotation2d[] {heading};
      } else {
        headings = new Rotation2d[] {heading, escapeHeading};
      }
    } else {
      headings = new Rotation2d[] {heading};
    }
    for (Rotation2d basis : headings) {
      int[] sides =
          (preferredSide != 0)
              ? new int[] {preferredSide, -preferredSide}
              : new int[] {biasSide, -biasSide};
      for (int sideSign : sides) {
        for (double ls : latScales) {
          double lat = clamp(baseLat * ls, cfg.minLateralMeters, cfg.lateralMaxMeters);
          for (double fs : fwdScales) {
            double fwd = clamp(baseFwd * fs, cfg.forwardMeters, cfg.forwardMaxMeters);
            Pose2d wp = makeWaypoint(pose, basis, sideSign, lat, fwd);
            if (!goesBehindGoal(wp, goal, basis)) out.add(wp);
          }
        }
      }
      double arcDeg = cfg.arcAngleDeg;
      int steps = cfg.arcAngularStepsPerSide;
      int rSteps = Math.max(1, cfg.arcRadialSteps);
      for (int sideSign : sides) {
        for (int ai = 1; ai <= steps; ai++) {
          double a = Math.toRadians((arcDeg * ai) / steps) * sideSign;
          Rotation2d dir = Rotation2d.fromRadians(basis.getRadians() + a);
          for (int ri = 0; ri < rSteps; ri++) {
            double r =
                cfg.arcRMin + (cfg.arcRMax - cfg.arcRMin) * (ri / Math.max(1.0, rSteps - 1.0));
            Translation2d tgt = pose.getTranslation().plus(new Translation2d(r, dir));
            Pose2d wp = new Pose2d(clampToField(tgt), basis);
            if (!goesBehindGoal(wp, goal, basis)) out.add(wp);
          }
        }
      }
    }
    return out;
  }

  private boolean goesBehindGoal(Pose2d wp, Pose2d goal, Rotation2d heading) {
    Translation2d toGoal = goal.getTranslation().minus(wp.getTranslation());
    double cos =
        Math.cos(heading.getRadians()) * toGoal.getX()
            + Math.sin(heading.getRadians()) * toGoal.getY();
    return cos < -0.15;
  }

  private Pose2d makeWaypoint(
      Pose2d pose, Rotation2d heading, int sideSign, double lateral, double forward) {
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d lat =
        new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg)).times(sideSign);
    Translation2d tgt = p.plus(dir.times(forward)).plus(lat.times(lateral));
    tgt = clampToField(tgt);
    return new Pose2d(tgt, heading);
  }

  private Pose2d slewSubgoal(Pose2d target, Pose2d pose, double dt) {
    if (target == null) return null;
    if (latchedSubgoal == null) return target;
    Translation2d cur = latchedSubgoal.getTranslation();
    Translation2d des = target.getTranslation();
    Translation2d delta = des.minus(cur);
    double maxStep = Math.max(0.02, cfg.subgoalSlewRateMps * dt);
    double d = delta.getNorm();
    Translation2d next = (d <= maxStep) ? des : cur.plus(delta.times(maxStep / d));
    if (next.getDistance(des) <= cfg.subgoalJitterMeters) next = des;
    double projCur = projectAlong(pose.getTranslation(), headingOf(pose), cur);
    double projNext = projectAlong(pose.getTranslation(), headingOf(pose), next);
    if (projNext < projCur && projCur - projNext < 0.12) next = cur;
    return new Pose2d(next, target.getRotation());
  }

  private static class BypassScore {
    final Pose2d wp;
    final boolean okLeg1;
    final boolean okLeg2;
    final double pathLen;
    final double angleCost;
    final double curvatureCost;
    final double wallPenalty;
    final double occPathCost;
    final double switchPenalty;
    final double zzzPenalty;
    final double progressCost;
    final double localOccCost;
    final double cornerReward;
    final double totalCost;

    BypassScore(
        Pose2d wp,
        boolean a,
        boolean b,
        double len,
        double ang,
        double curv,
        double wall,
        double occPath,
        double sw,
        double zzz,
        double prog,
        double localOcc,
        double cornerReward,
        double total) {
      this.wp = wp;
      this.okLeg1 = a;
      this.okLeg2 = b;
      this.pathLen = len;
      this.angleCost = ang;
      this.curvatureCost = curv;
      this.wallPenalty = wall;
      this.occPathCost = occPath;
      this.switchPenalty = sw;
      this.zzzPenalty = zzz;
      this.progressCost = prog;
      this.localOccCost = localOcc;
      this.cornerReward = cornerReward;
      this.totalCost = total;
    }
  }

  private BypassScore scoreCandidate(
      Pose2d pose,
      Pose2d waypoint,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double robotX,
      double robotY,
      List<? extends FieldPlanner.Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {

    boolean ok1 =
        ExtraPathing.isClearPath(
            "Repulsor/Bypass/Leg1",
            pose.getTranslation(),
            waypoint.getTranslation(),
            dynamicObstacles,
            robotX,
            robotY,
            true);

    boolean ok2 =
        ok1
            && ExtraPathing.isClearPath(
                "Repulsor/Bypass/Leg2",
                waypoint.getTranslation(),
                goal.getTranslation(),
                dynamicObstacles,
                robotX,
                robotY,
                true);

    double len =
        pose.getTranslation().getDistance(waypoint.getTranslation())
            + waypoint.getTranslation().getDistance(goal.getTranslation());

    double angErr =
        Math.abs(
            radDiff(
                headingTowardGoal.getRadians(),
                waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians()));
    double ang =
        (angErr <= Math.toRadians(cfg.headingDeadbandDeg))
            ? 0.0
            : cfg.angleCostWeight * (angErr - Math.toRadians(cfg.headingDeadbandDeg));

    double leg1 = waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians();
    double leg2 = goal.getTranslation().minus(waypoint.getTranslation()).getAngle().getRadians();
    double curv = cfg.curvatureWeight * Math.abs(radDiff(leg1, leg2));

    double wall = cfg.wallPenaltyGain * wallPenalty(waypoint.getTranslation());

    double occLeg =
        cfg.occCostGain
            * (legOcc(
                    pose.getTranslation(),
                    waypoint.getTranslation(),
                    robotX,
                    robotY,
                    intersectsDynamicOnly)
                + legOcc(
                    waypoint.getTranslation(),
                    goal.getTranslation(),
                    robotX,
                    robotY,
                    intersectsDynamicOnly));

    double localOcc =
        cfg.occCostGain
            * 0.5
            * localOccAt(
                waypoint.getTranslation(),
                headingTowardGoal,
                robotX,
                robotY,
                intersectsDynamicOnly);

    int side = sideOf(pose, headingTowardGoal, waypoint.getTranslation());
    double sw = sideSwitchPenalty(side);

    double yawDeltaDeg = Math.abs(Math.toDegrees(radDiff(headingTowardGoal.getRadians(), leg1)));
    double zzz = yawDeltaDeg > cfg.zzzMaxYawDeltaDeg ? cfg.zzzPenalty : 0.0;

    Translation2d delta = waypoint.getTranslation().minus(pose.getTranslation());
    double forwardProgress =
        delta.getX() * Math.cos(headingTowardGoal.getRadians())
            + delta.getY() * Math.sin(headingTowardGoal.getRadians());
    double requiredForward =
        cfg.minForwardProgressMeters
            * (pose.getTranslation().getDistance(goal.getTranslation()) > cfg.nearGoalDistMeters
                ? 1.0
                : 0.5);
    double progressDeficit = Math.max(0.0, requiredForward - forwardProgress);
    double prog = cfg.progressCostWeight * progressDeficit;

    double wallHere = wallPenalty(pose.getTranslation());
    double wallWp = wallPenalty(waypoint.getTranslation());
    double cornerReward = 0.0;
    if (wallHere > cfg.cornerWallThresh) {
      double improve = wallHere - wallWp;
      if (improve > 0.0) cornerReward = -cfg.cornerRewardGain * improve;
    }

    double total =
        (ok1 && ok2 ? len : 1e9)
            + ang
            + curv
            + wall
            + occLeg
            + localOcc
            + sw
            + zzz
            + prog
            + cornerReward;

    return new BypassScore(
        waypoint,
        ok1,
        ok2,
        len,
        ang,
        curv,
        wall,
        occLeg,
        sw,
        zzz,
        prog,
        localOcc,
        cornerReward,
        total);
  }

  private double legOcc(
      Translation2d a,
      Translation2d b,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d delta = b.minus(a);
    double L = Math.max(1e-6, delta.getNorm());
    Rotation2d dir = delta.getAngle();
    Translation2d step = new Translation2d(cfg.probeOccLegStep, dir);
    int steps = Math.max(1, (int) Math.ceil(L / cfg.probeOccLegStep));
    int hit = 0;
    for (int i = 1; i <= steps; i++) {
      Translation2d p = a.plus(step.times(i));
      Translation2d[] rect = FieldPlanner.robotRect(p, dir, rx, ry);
      if (intersectsDynamicOnly.apply(rect)) hit++;
    }
    return (double) hit / (double) steps;
  }

  private Rotation2d nearestWallNormal(Translation2d p) {
    double left = p.getX();
    double right = cfg.fieldLen - p.getX();
    double bottom = p.getY();
    double top = cfg.fieldWid - p.getY();
    double min = left;
    int idx = 0;
    if (right < min) {
      min = right;
      idx = 1;
    }
    if (bottom < min) {
      min = bottom;
      idx = 2;
    }
    if (top < min) {
      min = top;
      idx = 3;
    }
    switch (idx) {
      case 0:
        return new Rotation2d();
      case 1:
        return Rotation2d.fromRadians(Math.PI);
      case 2:
        return Rotation2d.kCCW_90deg;
      default:
        return Rotation2d.kCW_90deg;
    }
  }

  private double wallPenalty(Translation2d p) {
    double dx = Math.min(p.getX(), cfg.fieldLen - p.getX());
    double dy = Math.min(p.getY(), cfg.fieldWid - p.getY());
    double d = Math.min(dx, dy);
    return 1.0 / (0.18 + d);
  }

  private double sideSwitchPenalty(int candidateSide) {
    if (preferredSide == 0) return 0.0;
    if (candidateSide == preferredSide) return 0.0;
    boolean stuckNow = isVibrating() || isForwardStuck();
    if (stuckNow) {
      return cfg.sideSwitchPenalty * cfg.sideSwitchStuckPenaltyScale;
    }
    if (timeSinceSideSwitchS < cfg.sideStickSeconds) return 1e9;
    return cfg.sideSwitchPenalty;
  }

  private static int sideOf(Pose2d pose, Rotation2d heading, Translation2d target) {
    Translation2d to = target.minus(pose.getTranslation());
    double d = radDiff(heading.getRadians(), to.getAngle().getRadians());
    if (Math.abs(d) < 1e-6) return 0;
    return d > 0 ? +1 : -1;
  }

  private static double radDiff(double a, double b) {
    double d = a - b;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    return d;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static double clamp01(double v) {
    return clamp(v, 0.0, 1.0);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * clamp01(t);
  }

  private Translation2d clampToField(Translation2d t) {
    double x = clamp(t.getX(), 0.05, cfg.fieldLen - 0.05);
    double y = clamp(t.getY(), 0.05, cfg.fieldWid - 0.05);
    return new Translation2d(x, y);
  }

  private Pose2d alignedPose(Pose2d p, Rotation2d desiredHeading) {
    return new Pose2d(p.getTranslation(), desiredHeading);
  }

  private static Rotation2d headingOf(Pose2d pose) {
    return pose.getRotation();
  }

  private static double projectAlong(Translation2d origin, Rotation2d dir, Translation2d point) {
    Translation2d rel = point.minus(origin);
    return rel.getX() * Math.cos(dir.getRadians()) + rel.getY() * Math.sin(dir.getRadians());
  }

  public boolean isPinnedMode() {
    return pinnedMode;
  }

  private void loadConfigFromYaml() {
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      Path yamlPath = deployDir.resolve("ReactiveBypassConfig.yaml");
      if (!Files.exists(yamlPath)) return;
      List<String> lines = Files.readAllLines(yamlPath, StandardCharsets.UTF_8);
      for (String raw : lines) {
        String line = raw.trim();
        if (line.isEmpty()) continue;
        if (line.startsWith("#")) continue;
        int idx = line.indexOf(':');
        if (idx <= 0) continue;
        String key = line.substring(0, idx).trim();
        String valueStr = line.substring(idx + 1).trim();
        if (!valueStr.isEmpty()) applyConfigField(key, valueStr);
      }
    } catch (IOException e) {
      System.err.println("ReactiveBypass: Failed to load config from YAML: " + e);
    }
  }

  private void applyConfigField(String key, String valueStr) {
    try {
      Field f = Config.class.getField(key);
      Class<?> t = f.getType();
      if (t == double.class) {
        double v = Double.parseDouble(valueStr);
        f.setDouble(cfg, v);
      } else if (t == int.class) {
        int v = Integer.parseInt(valueStr);
        f.setInt(cfg, v);
      } else if (t == boolean.class) {
        boolean v = Boolean.parseBoolean(valueStr);
        f.setBoolean(cfg, v);
      }
    } catch (NoSuchFieldException | IllegalAccessException | IllegalArgumentException ignored) {
    }
  }
}

