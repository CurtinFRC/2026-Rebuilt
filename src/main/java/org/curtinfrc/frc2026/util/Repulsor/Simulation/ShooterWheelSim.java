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

/** Shooter wheel simulation with aerodynamic drag. */
public final class ShooterWheelSim extends SingleMotorRotarySim {
  private double dragCoefficient;

  /**
   * Creates a shooter wheel simulation.
   *
   * @param params mechanism parameters
   * @param dragCoefficient drag coefficient in torque per speed squared
   */
  public ShooterWheelSim(MechanismSimParams params, double dragCoefficient) {
    super(params);
    this.dragCoefficient = Math.max(0.0, dragCoefficient);
    setLoadModel(this::dragLoad);
  }

  /**
   * Sets the drag coefficient.
   *
   * @param dragCoefficient drag coefficient in torque per speed squared
   */
  public void setDragCoefficient(double dragCoefficient) {
    this.dragCoefficient = Math.max(0.0, dragCoefficient);
  }

  private double dragLoad(MechanismSimState state, double appliedVoltage) {
    double velocity = state.getVelocity();
    double drag = dragCoefficient * velocity * velocity;
    return -Math.signum(velocity) * drag;
  }
}
