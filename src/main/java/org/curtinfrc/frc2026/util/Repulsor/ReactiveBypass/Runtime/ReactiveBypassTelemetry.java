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
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Locale;

final class ReactiveBypassTelemetry {
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

  void enableLogging(String filePath) {
    this.logFilePath = filePath;
    this.loggingEnabled = true;
  }

  void disableLogging() {
    this.loggingEnabled = false;
  }

  void finalizeEpisode(boolean success) {
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

  void resetEpisodeMetrics() {
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

  void logStep(
      Pose2d pose,
      Pose2d goal,
      double dtSeconds,
      boolean blockedNow,
      boolean touchDynamic,
      boolean pinnedMode) {
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

  private void appendLogLine(String line) {
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
}
