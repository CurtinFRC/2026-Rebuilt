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

/** Interface for pressure or pneumatic effect models. */
public interface PressureModel {
  /**
   * Updates the pressure model and returns the effort applied to the mechanism.
   *
   * @param dtSeconds timestep in seconds
   * @param command command value, typically in the range [-1, 1]
   * @param state current simulation state
   * @return effort applied by the pressure model
   */
  double update(double dtSeconds, double command, MechanismSimState state);

  /**
   * Returns the current pressure estimate in kilopascals.
   *
   * @return pressure in kilopascals
   */
  double getPressureKpa();

  /**
   * Returns the current flow rate estimate in unitless normalized terms.
   *
   * @return flow rate estimate
   */
  double getFlowRate();

  /** Resets any internal state to its default values. */
  void reset();
}

