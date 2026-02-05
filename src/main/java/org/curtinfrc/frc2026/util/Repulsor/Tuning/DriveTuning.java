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


package org.curtinfrc.frc2026.util.Repulsor.Tuning;

public abstract class DriveTuning extends Tuning {
  protected DriveTuning(String key) {
    super(key);
  }

  public abstract double maxLinearSpeedMps();

  public abstract double minStepMeters();

  public abstract double baseStepMeters(double distanceMeters, boolean slowDown);

  public abstract double nearGoalScale(double distanceMeters);

  public abstract double scaleForTurning(double yawDeltaRad, boolean isScoring);

  public double stepSizeMeters(
      double distanceMeters, double obstacleMag, boolean isScoring, boolean slowDown) {
    double base = baseStepMeters(Math.max(0.0, distanceMeters), slowDown);
    return base;
    // double near = nearGoalScale(distanceMeters);
    // double obs = 1.0 / (1.0 + obstacleMag);
    // return Math.max(
    //     minStepMeters(), Math.min(maxLinearSpeedMps() * dtSeconds(), base * near * obs));
  }
}

