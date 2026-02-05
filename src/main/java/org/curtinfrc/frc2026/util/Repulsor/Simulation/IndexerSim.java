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

/** Indexer or conveyor simulation with a constant load torque. */
public final class IndexerSim extends SingleMotorRotarySim {
  private double loadTorque;

  /**
   * Creates an indexer simulation.
   *
   * @param params mechanism parameters
   * @param loadTorque constant load torque
   */
  public IndexerSim(MechanismSimParams params, double loadTorque) {
    super(params);
    this.loadTorque = loadTorque;
    setLoadModel(this::loadModel);
  }

  /**
   * Sets the constant load torque.
   *
   * @param loadTorque constant load torque
   */
  public void setLoadTorque(double loadTorque) {
    this.loadTorque = loadTorque;
  }

  private double loadModel(MechanismSimState state, double appliedVoltage) {
    double sign = Math.signum(state.getVelocity());
    return -sign * Math.abs(loadTorque);
  }
}
