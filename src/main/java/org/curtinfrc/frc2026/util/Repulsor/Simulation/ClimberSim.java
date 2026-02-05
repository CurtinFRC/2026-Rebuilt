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

/** Climber simulation based on a linear mechanism with gravity load. */
public final class ClimberSim extends ElevatorSim {
  /**
   * Creates a climber simulation.
   *
   * @param params mechanism parameters
   * @param gravityAccel gravity acceleration in meters per second squared
   */
  public ClimberSim(MechanismSimParams params, double gravityAccel) {
    super(params, gravityAccel);
  }
}
