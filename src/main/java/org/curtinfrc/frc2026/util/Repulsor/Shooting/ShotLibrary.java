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
import java.util.List;

public final class ShotLibrary {
  private final Translation2d targetFieldPosition;
  private final double targetHeightMeters;
  private final double shooterReleaseHeightMeters;
  private final double robotHalfLengthMeters;
  private final double robotHalfWidthMeters;
  private final Constraints constraints;
  private final List<ShotLibraryEntry> entries;
  private final boolean complete;

  public ShotLibrary(
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      Constraints constraints,
      List<ShotLibraryEntry> entries,
      boolean complete) {
    this.targetFieldPosition = targetFieldPosition;
    this.targetHeightMeters = targetHeightMeters;
    this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
    this.robotHalfLengthMeters = robotHalfLengthMeters;
    this.robotHalfWidthMeters = robotHalfWidthMeters;
    this.constraints = constraints;
    this.entries = entries;
    this.complete = complete;
  }

  public Translation2d targetFieldPosition() {
    return targetFieldPosition;
  }

  public double targetHeightMeters() {
    return targetHeightMeters;
  }

  public double shooterReleaseHeightMeters() {
    return shooterReleaseHeightMeters;
  }

  public double robotHalfLengthMeters() {
    return robotHalfLengthMeters;
  }

  public double robotHalfWidthMeters() {
    return robotHalfWidthMeters;
  }

  public Constraints constraints() {
    return constraints;
  }

  public List<ShotLibraryEntry> entries() {
    return entries;
  }

  public boolean complete() {
    return complete;
  }
}
