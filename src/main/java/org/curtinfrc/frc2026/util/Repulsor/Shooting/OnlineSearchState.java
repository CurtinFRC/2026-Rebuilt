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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

public final class OnlineSearchState {
  private Translation2d seed;
  private double stepMeters;
  private long lastUpdateNs;
  private ShotSolution lastSolution;
  private int lastRobotXmm;
  private int lastRobotYmm;
  private int lastTargetXmm;
  private int lastTargetYmm;
  private int lastObsHash;
  private long lastSolutionNs;

  public OnlineSearchState(Translation2d seed, double stepMeters) {
    this.seed = seed;
    this.stepMeters = stepMeters;
    this.lastUpdateNs = System.nanoTime();
    Profiler.ensureInit();
  }

  public Translation2d seed() {
    return seed;
  }

  public void seed(Translation2d seed) {
    this.seed = seed;
  }

  public double stepMeters() {
    return stepMeters;
  }

  public void stepMeters(double stepMeters) {
    this.stepMeters = stepMeters;
  }

  public long lastUpdateNs() {
    return lastUpdateNs;
  }

  public void touch() {
    this.lastUpdateNs = System.nanoTime();
  }

  ShotSolution lastSolution() {
    return lastSolution;
  }

  int lastRobotXmm() {
    return lastRobotXmm;
  }

  int lastRobotYmm() {
    return lastRobotYmm;
  }

  int lastTargetXmm() {
    return lastTargetXmm;
  }

  int lastTargetYmm() {
    return lastTargetYmm;
  }

  int lastObsHash() {
    return lastObsHash;
  }

  long lastSolutionNs() {
    return lastSolutionNs;
  }

  void setLastSolution(
      ShotSolution solution,
      int robotXmm,
      int robotYmm,
      int targetXmm,
      int targetYmm,
      int obsHash) {
    this.lastSolution = solution;
    this.lastRobotXmm = robotXmm;
    this.lastRobotYmm = robotYmm;
    this.lastTargetXmm = targetXmm;
    this.lastTargetYmm = targetYmm;
    this.lastObsHash = obsHash;
    this.lastSolutionNs = System.nanoTime();
  }
}
