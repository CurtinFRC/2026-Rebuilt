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

