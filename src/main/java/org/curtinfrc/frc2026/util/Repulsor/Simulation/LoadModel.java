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

/** Functional interface for external load models. */
@FunctionalInterface
public interface LoadModel {
  /**
   * Computes external effort acting on the mechanism.
   *
   * @param state current simulation state
   * @param appliedVoltage applied motor voltage in volts
   * @return external effort in torque or force units
   */
  double getExternalEffort(MechanismSimState state, double appliedVoltage);
}
