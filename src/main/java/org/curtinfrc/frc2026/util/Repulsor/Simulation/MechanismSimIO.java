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
