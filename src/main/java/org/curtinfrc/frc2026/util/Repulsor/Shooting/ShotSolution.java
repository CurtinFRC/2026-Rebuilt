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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ShotSolution {
  private final Translation2d shooterPosition;
  private final Rotation2d shooterYaw;
  private final double launchSpeedMetersPerSecond;
  private final Rotation2d launchAngle;
  private final double timeToPlaneSeconds;
  private final Translation2d impactFieldPosition;
  private final double verticalErrorMeters;

  public ShotSolution(
      Translation2d shooterPosition,
      Rotation2d shooterYaw,
      double launchSpeedMetersPerSecond,
      Rotation2d launchAngle,
      double timeToPlaneSeconds,
      Translation2d impactFieldPosition,
      double verticalErrorMeters) {
    this.shooterPosition = shooterPosition;
    this.shooterYaw = shooterYaw == null ? Rotation2d.kZero : shooterYaw;
    this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
    this.launchAngle = launchAngle == null ? Rotation2d.kZero : launchAngle;
    this.timeToPlaneSeconds = timeToPlaneSeconds;
    this.impactFieldPosition =
        impactFieldPosition == null ? new Translation2d() : impactFieldPosition;
    this.verticalErrorMeters = verticalErrorMeters;
  }

  public Translation2d shooterPosition() {
    return shooterPosition;
  }

  public Rotation2d shooterYaw() {
    return shooterYaw;
  }

  public double launchSpeedMetersPerSecond() {
    return launchSpeedMetersPerSecond;
  }

  public Rotation2d launchAngle() {
    return launchAngle;
  }

  public double timeToPlaneSeconds() {
    return timeToPlaneSeconds;
  }

  public Translation2d impactFieldPosition() {
    return impactFieldPosition;
  }

  public double verticalErrorMeters() {
    return verticalErrorMeters;
  }
}
