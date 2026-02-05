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

/** Interface for subsystem IO layers that use a mechanism simulation. */
public interface MechanismSimIO {
  /**
   * Sets the input voltage.
   *
   * @param volts input voltage in volts
   */
  void setInputVoltage(double volts);

  /**
   * Sets the motor command as a percent output.
   *
   * @param percent percent output in the range [-1, 1]
   */
  void setMotorCommand(double percent);

  /**
   * Advances the simulation.
   *
   * @param dtSeconds timestep in seconds
   */
  void update(double dtSeconds);

  /**
   * Returns the current simulation state.
   *
   * @return simulation state
   */
  MechanismSimState getState();
}
