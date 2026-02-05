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


package org.curtinfrc.frc2026.util.Repulsor.Units;

import edu.wpi.first.math.util.Units;

/**
 * Shooter / flywheel unit conversions built on top of WPILib Units.
 *
 * <p>Conventions: - Distances in meters - Linear velocities in meters per second (m/s) - Angular
 * speeds in rotations per minute (RPM)
 *
 * <p>Motor -> (gear ratio) -> Flywheel -> (radius) -> Exit velocity
 */
public final class ShooterUnits {
  private ShooterUnits() {
    // Utility class
  }

  /**
   * Converts flywheel RPM directly to tangential exit velocity (m/s) at the rim.
   *
   * @param flywheelRpm flywheel speed in RPM (at the wheel itself)
   * @param wheelRadiusMeters wheel radius in meters
   * @return linear velocity at the rim in m/s
   */
  public static double flywheelRpmToMetersPerSecond(double flywheelRpm, double wheelRadiusMeters) {
    double omegaRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelRpm);
    return omegaRadPerSec * wheelRadiusMeters;
  }

  /**
   * Converts tangential exit velocity (m/s) at the rim to flywheel RPM.
   *
   * @param exitVelocityMps desired exit velocity in m/s
   * @param wheelRadiusMeters wheel radius in meters
   * @return flywheel speed in RPM
   */
  public static double metersPerSecondToFlywheelRpm(
      double exitVelocityMps, double wheelRadiusMeters) {
    double omegaRadPerSec = exitVelocityMps / wheelRadiusMeters;
    return Units.radiansPerSecondToRotationsPerMinute(omegaRadPerSec);
  }

  /**
   * Converts motor RPM to exit velocity (m/s), given a motor-to-flywheel gear ratio.
   *
   * @param motorRpm motor speed in RPM
   * @param wheelRadiusMeters wheel radius in meters
   * @param motorToWheelGearRatio motor rotations / one wheel rotation (e.g. 3.0 means motor spins
   *     3x per wheel rev)
   * @return exit velocity in m/s
   */
  public static double motorRpmToExitVelocity(
      double motorRpm, double wheelRadiusMeters, double motorToWheelGearRatio) {
    double wheelRpm = motorRpm / motorToWheelGearRatio;
    return flywheelRpmToMetersPerSecond(wheelRpm, wheelRadiusMeters);
  }

  /**
   * Converts desired exit velocity (m/s) to required motor RPM, given a motor-to-flywheel gear
   * ratio.
   *
   * @param exitVelocityMps desired exit velocity in m/s
   * @param wheelRadiusMeters wheel radius in meters
   * @param motorToWheelGearRatio motor rotations / one wheel rotation
   * @return required motor RPM
   */
  public static double exitVelocityToMotorRpm(
      double exitVelocityMps, double wheelRadiusMeters, double motorToWheelGearRatio) {
    double wheelRpm = metersPerSecondToFlywheelRpm(exitVelocityMps, wheelRadiusMeters);
    return wheelRpm * motorToWheelGearRatio;
  }

  /**
   * Convenience overload using diameter instead of radius.
   *
   * @param flywheelRpm flywheel speed in RPM
   * @param wheelDiameterMeters wheel diameter in meters
   * @return exit velocity in m/s
   */
  public static double flywheelRpmToMetersPerSecondFromDiameter(
      double flywheelRpm, double wheelDiameterMeters) {
    return flywheelRpmToMetersPerSecond(flywheelRpm, wheelDiameterMeters / 2.0);
  }

  /**
   * Convenience overload using diameter instead of radius.
   *
   * @param exitVelocityMps desired exit velocity in m/s
   * @param wheelDiameterMeters wheel diameter in meters
   * @return flywheel RPM
   */
  public static double metersPerSecondToFlywheelRpmFromDiameter(
      double exitVelocityMps, double wheelDiameterMeters) {
    return metersPerSecondToFlywheelRpm(exitVelocityMps, wheelDiameterMeters / 2.0);
  }
}

