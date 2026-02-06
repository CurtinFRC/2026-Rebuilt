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

public final class Constraints {

  public enum ShotStyle {
    ANY,
    DIRECT,
    ARC
  }

  private final double minLaunchSpeedMetersPerSecond;
  private final double maxLaunchSpeedMetersPerSecond;
  private final double minLaunchAngleDeg;
  private final double maxLaunchAngleDeg;
  private final ShotStyle shotStyle;

  public Constraints(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg) {
    this(
        minLaunchSpeedMetersPerSecond,
        maxLaunchSpeedMetersPerSecond,
        minLaunchAngleDeg,
        maxLaunchAngleDeg,
        ShotStyle.ANY);
  }

  public Constraints(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg,
      ShotStyle shotStyle) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
    this.minLaunchAngleDeg = minLaunchAngleDeg;
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
    this.shotStyle = shotStyle == null ? ShotStyle.ANY : shotStyle;
  }

  public double minLaunchSpeedMetersPerSecond() {
    return minLaunchSpeedMetersPerSecond;
  }

  public double maxLaunchSpeedMetersPerSecond() {
    return maxLaunchSpeedMetersPerSecond;
  }

  public double minLaunchAngleDeg() {
    return minLaunchAngleDeg;
  }

  public double maxLaunchAngleDeg() {
    return maxLaunchAngleDeg;
  }

  public ShotStyle shotStyle() {
    return shotStyle;
  }
}
