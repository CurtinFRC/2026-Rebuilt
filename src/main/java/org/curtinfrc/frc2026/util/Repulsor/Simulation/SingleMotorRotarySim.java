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

import edu.wpi.first.math.util.Units;

/** Rotary mechanism simulation for a single motor. */
public class SingleMotorRotarySim extends SimpleMechanismSim {
  /**
   * Creates a rotary simulation.
   *
   * @param params mechanism parameters
   */
  public SingleMotorRotarySim(MechanismSimParams params) {
    super(params, params.getGearRatio(), params.getGearRatio() * params.getEfficiency());
  }

  /**
   * Returns the position in radians.
   *
   * @return position in radians
   */
  public double getPositionRad() {
    return getPosition();
  }

  /**
   * Returns the position in rotations.
   *
   * @return position in rotations
   */
  public double getPositionRotations() {
    return Units.radiansToRotations(getPosition());
  }

  /**
   * Returns the velocity in radians per second.
   *
   * @return velocity in radians per second
   */
  public double getVelocityRadPerSec() {
    return getVelocity();
  }

  /**
   * Returns the velocity in rotations per second.
   *
   * @return velocity in rotations per second
   */
  public double getVelocityRotationsPerSecond() {
    return Units.radiansToRotations(getVelocity());
  }
}
