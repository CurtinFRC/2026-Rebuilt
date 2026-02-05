/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
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
