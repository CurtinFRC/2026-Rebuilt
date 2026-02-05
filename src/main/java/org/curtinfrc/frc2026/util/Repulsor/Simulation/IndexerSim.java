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
