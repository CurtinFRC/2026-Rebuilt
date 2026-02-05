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

/** Arm simulation with gravity torque based on angle. */
public class ArmSim extends SingleMotorRotarySim {
  private double massKg;
  private double cgRadiusMeters;
  private double gravityAccel;
  private double gravityOffsetRad;
  private double gravitySign;

  /**
   * Creates an arm simulation.
   *
   * @param params mechanism parameters
   * @param massKg arm mass in kilograms
   * @param cgRadiusMeters center of gravity distance in meters
   * @param gravityAccel gravity acceleration in meters per second squared
   * @param gravityOffsetRad offset added to the arm angle for gravity torque
   */
  public ArmSim(
      MechanismSimParams params,
      double massKg,
      double cgRadiusMeters,
      double gravityAccel,
      double gravityOffsetRad) {
    super(params);
    this.massKg = Math.max(0.0, massKg);
    this.cgRadiusMeters = Math.max(0.0, cgRadiusMeters);
    this.gravityAccel = Math.abs(gravityAccel);
    this.gravityOffsetRad = gravityOffsetRad;
    this.gravitySign = 1.0;
    setLoadModel(this::gravityLoad);
  }

  /**
   * Sets the arm mass.
   *
   * @param massKg mass in kilograms
   */
  public void setMassKg(double massKg) {
    this.massKg = Math.max(0.0, massKg);
  }

  /**
   * Sets the center of gravity radius.
   *
   * @param cgRadiusMeters center of gravity radius in meters
   */
  public void setCgRadiusMeters(double cgRadiusMeters) {
    this.cgRadiusMeters = Math.max(0.0, cgRadiusMeters);
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
   * Sets the gravity offset angle.
   *
   * @param gravityOffsetRad gravity offset in radians
   */
  public void setGravityOffsetRad(double gravityOffsetRad) {
    this.gravityOffsetRad = gravityOffsetRad;
  }

  /**
   * Sets the gravity sign.
   *
   * @param gravitySign positive for default direction, negative to invert
   */
  public void setGravitySign(double gravitySign) {
    this.gravitySign = gravitySign >= 0.0 ? 1.0 : -1.0;
  }

  private double gravityLoad(MechanismSimState state, double appliedVoltage) {
    double torque =
        massKg * gravityAccel * cgRadiusMeters * Math.sin(state.getPosition() + gravityOffsetRad);
    return -gravitySign * torque;
  }
}

