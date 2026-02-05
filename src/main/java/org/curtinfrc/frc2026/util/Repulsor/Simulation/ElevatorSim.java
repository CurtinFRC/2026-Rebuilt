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

/** Elevator simulation with gravity and optional constant load. */
public class ElevatorSim extends SingleMotorLinearSim {
  private double massKg;
  private double gravityAccel;
  private double gravitySign;
  private double constantLoadNewtons;

  /**
   * Creates an elevator simulation.
   *
   * @param params mechanism parameters
   * @param gravityAccel gravity acceleration in meters per second squared
   */
  public ElevatorSim(MechanismSimParams params, double gravityAccel) {
    super(params);
    this.massKg = Math.max(0.0, params.getMassKg());
    this.gravityAccel = Math.abs(gravityAccel);
    this.gravitySign = -1.0;
    setLoadModel(this::gravityLoad);
  }

  /**
   * Sets the carriage mass.
   *
   * @param massKg mass in kilograms
   */
  public void setMassKg(double massKg) {
    this.massKg = Math.max(0.0, massKg);
    setEffectiveInertia(Math.max(1e-6, this.massKg));
  }

  /**
   * Sets the gravity acceleration.
   *
   * @param gravityAccel gravity acceleration in meters per second squared
   */
  public void setGravityAccel(double gravityAccel) {
    this.gravityAccel = Math.abs(gravityAccel);
  }

  /**
   * Sets the gravity direction sign.
   *
   * @param gravitySign positive for upward gravity, negative for downward gravity
   */
  public void setGravitySign(double gravitySign) {
    this.gravitySign = gravitySign >= 0.0 ? 1.0 : -1.0;
  }

  /**
   * Sets an additional constant load.
   *
   * @param constantLoadNewtons constant load in newtons
   */
  public void setConstantLoadNewtons(double constantLoadNewtons) {
    this.constantLoadNewtons = constantLoadNewtons;
  }

  private double gravityLoad(MechanismSimState state, double appliedVoltage) {
    double gravity = massKg * gravityAccel * gravitySign;
    return gravity + constantLoadNewtons;
  }
}
