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

public final class ShotLibraryEntry {
  private final Translation2d shooterPosition;
  private final double shooterYawRad;
  private final double launchSpeedMetersPerSecond;
  private final double launchAngleRad;
  private final double timeToPlaneSeconds;
  private final double verticalErrorMeters;

  public ShotLibraryEntry(
      Translation2d shooterPosition,
      double shooterYawRad,
      double launchSpeedMetersPerSecond,
      double launchAngleRad,
      double timeToPlaneSeconds,
      double verticalErrorMeters) {
    this.shooterPosition = shooterPosition;
    this.shooterYawRad = shooterYawRad;
    this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
    this.launchAngleRad = launchAngleRad;
    this.timeToPlaneSeconds = timeToPlaneSeconds;
    this.verticalErrorMeters = verticalErrorMeters;
  }

  public Translation2d shooterPosition() {
    return shooterPosition;
  }

  public double shooterYawRad() {
    return shooterYawRad;
  }

  public double launchSpeedMetersPerSecond() {
    return launchSpeedMetersPerSecond;
  }

  public double launchAngleRad() {
    return launchAngleRad;
  }

  public double timeToPlaneSeconds() {
    return timeToPlaneSeconds;
  }

  public double verticalErrorMeters() {
    return verticalErrorMeters;
  }
}
