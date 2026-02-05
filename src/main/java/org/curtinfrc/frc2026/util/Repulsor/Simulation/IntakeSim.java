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

import edu.wpi.first.math.MathUtil;

/** Roller intake simulation with a configurable contact load. */
public final class IntakeSim extends SingleMotorRotarySim {
  private double contactTorque;
  private double contactVelocityScale;
  private double contactViscous;
  private double contactFactor;

  /**
   * Creates an intake simulation.
   *
   * @param params mechanism parameters
   * @param contactTorque torque applied at full contact
   * @param contactVelocityScale velocity scale for contact engagement
   * @param contactViscous viscous contact coefficient
   */
  public IntakeSim(
      MechanismSimParams params,
      double contactTorque,
      double contactVelocityScale,
      double contactViscous) {
    super(params);
    this.contactTorque = Math.max(0.0, contactTorque);
    this.contactVelocityScale = Math.max(1e-6, contactVelocityScale);
    this.contactViscous = Math.max(0.0, contactViscous);
    setLoadModel(this::contactLoad);
  }

  /**
   * Sets whether the intake is in contact with a game piece.
   *
   * @param hasGamePiece true when a game piece is present
   */
  public void setHasGamePiece(boolean hasGamePiece) {
    contactFactor = hasGamePiece ? 1.0 : 0.0;
  }

  /**
   * Sets the contact factor from 0 to 1.
   *
   * @param factor contact factor
   */
  public void setContactFactor(double factor) {
    contactFactor = MathUtil.clamp(factor, 0.0, 1.0);
  }

  /**
   * Sets the contact torque at full engagement.
   *
   * @param contactTorque torque at full engagement
   */
  public void setContactTorque(double contactTorque) {
    this.contactTorque = Math.max(0.0, contactTorque);
  }

  /**
   * Sets the contact velocity scale.
   *
   * @param contactVelocityScale velocity scale
   */
  public void setContactVelocityScale(double contactVelocityScale) {
    this.contactVelocityScale = Math.max(1e-6, contactVelocityScale);
  }

  /**
   * Sets the viscous contact coefficient.
   *
   * @param contactViscous viscous contact coefficient
   */
  public void setContactViscous(double contactViscous) {
    this.contactViscous = Math.max(0.0, contactViscous);
  }

  private double contactLoad(MechanismSimState state, double appliedVoltage) {
    if (contactFactor <= 0.0) {
      return 0.0;
    }
    double velocity = state.getVelocity();
    double speed = Math.abs(velocity);
    double engagement = 1.0 - Math.exp(-speed / contactVelocityScale);
    double torque = contactFactor * (contactTorque * engagement + contactViscous * speed);
    return -Math.signum(velocity) * torque;
  }
}
