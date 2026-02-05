/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
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

