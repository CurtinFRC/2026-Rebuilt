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

package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Linear mechanism simulation for a single motor with a drum or pulley. */
public class SingleMotorLinearSim extends SimpleMechanismSim {
  private final double drumRadiusMeters;

  /**
   * Creates a linear simulation.
   *
   * @param params mechanism parameters
   */
  public SingleMotorLinearSim(MechanismSimParams params) {
    super(
        params,
        params.getGearRatio() / Math.max(1e-6, params.getDrumRadiusMeters()),
        params.getGearRatio()
            * params.getEfficiency()
            / Math.max(1e-6, params.getDrumRadiusMeters()));
    if (params.getDrumRadiusMeters() <= 0.0) {
      throw new IllegalArgumentException("drumRadiusMeters");
    }
    drumRadiusMeters = params.getDrumRadiusMeters();
  }

  /**
   * Returns the drum radius.
   *
   * @return drum radius in meters
   */
  public double getDrumRadiusMeters() {
    return drumRadiusMeters;
  }

  /**
   * Returns the position in meters.
   *
   * @return position in meters
   */
  public double getPositionMeters() {
    return getPosition();
  }

  /**
   * Returns the velocity in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond() {
    return getVelocity();
  }
}
