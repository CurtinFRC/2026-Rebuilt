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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassConfig;
import org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime.ReactiveBypassRuntime;

public class ReactiveBypass {
  public static class Config extends ReactiveBypassConfig {}

  private final Config cfg = new Config();
  private final ReactiveBypassRuntime runtime = new ReactiveBypassRuntime(cfg);

  public void setConfig(Consumer<Config> c) {
    c.accept(cfg);
  }

  public void enableLogging(String filePath) {
    runtime.enableLogging(filePath);
  }

  public void disableLogging() {
    runtime.disableLogging();
  }

  public void finalizeEpisode(boolean success) {
    runtime.finalizeEpisode(success);
  }

  public void resetEpisodeMetrics() {
    runtime.resetEpisodeMetrics();
  }

  public void reset() {
    runtime.reset();
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
    return runtime.update(
        pose,
        goal,
        headingTowardGoal,
        dtSeconds,
        robotX,
        robotY,
        dynamicObstacles,
        intersectsDynamicOnly,
        canRejoinOriginal);
  }

  public boolean isPinnedMode() {
    return runtime.isPinnedMode();
  }
}
