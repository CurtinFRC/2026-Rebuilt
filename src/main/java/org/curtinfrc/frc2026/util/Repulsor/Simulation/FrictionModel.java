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

/**
 * Friction model supporting static, kinetic, and viscous friction with a smooth transition near
 * zero velocity.
 */
public final class FrictionModel {
  private final double staticFriction;
  private final double kineticFriction;
  private final double viscousFriction;
  private final double stribeckVelocity;
  private final double stictionVelocity;
  private final double signEpsilon;

  /**
   * Creates a friction model.
   *
   * @param staticFriction effort required to break static friction
   * @param kineticFriction kinetic friction effort at nonzero velocity
   * @param viscousFriction viscous friction coefficient
   * @param stribeckVelocity velocity scale for the static to kinetic transition
   * @param stictionVelocity velocity below which static friction may hold motion
   */
  public FrictionModel(
      double staticFriction,
      double kineticFriction,
      double viscousFriction,
      double stribeckVelocity,
      double stictionVelocity) {
    this.staticFriction = Math.max(0.0, staticFriction);
    this.kineticFriction = Math.max(0.0, kineticFriction);
    this.viscousFriction = Math.max(0.0, viscousFriction);
    this.stribeckVelocity = Math.max(0.0, stribeckVelocity);
    this.stictionVelocity = Math.max(0.0, stictionVelocity);
    this.signEpsilon = 1e-6;
  }

  /**
   * Returns a friction model with no friction.
   *
   * @return friction model with zero friction
   */
  public static FrictionModel none() {
    return new FrictionModel(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Returns the static friction effort.
   *
   * @return static friction effort
   */
  public double getStaticFriction() {
    return staticFriction;
  }

  /**
   * Returns the kinetic friction effort.
   *
   * @return kinetic friction effort
   */
  public double getKineticFriction() {
    return kineticFriction;
  }

  /**
   * Returns the viscous friction coefficient.
   *
   * @return viscous friction coefficient
   */
  public double getViscousFriction() {
    return viscousFriction;
  }

  /**
   * Returns the Stribeck velocity scale.
   *
   * @return Stribeck velocity scale
   */
  public double getStribeckVelocity() {
    return stribeckVelocity;
  }

  /**
   * Returns the stiction velocity threshold.
   *
   * @return stiction velocity threshold
   */
  public double getStictionVelocity() {
    return stictionVelocity;
  }

  FrictionResult compute(double velocity, double effortWithoutFriction) {
    double absVelocity = Math.abs(velocity);
    if (absVelocity < stictionVelocity && Math.abs(effortWithoutFriction) < staticFriction) {
      return new FrictionResult(effortWithoutFriction, true);
    }
    double sign =
        absVelocity > signEpsilon ? Math.signum(velocity) : Math.signum(effortWithoutFriction);
    double stribeck = kineticFriction;
    if (stribeckVelocity > signEpsilon) {
      double ratio = absVelocity / stribeckVelocity;
      stribeck = kineticFriction + (staticFriction - kineticFriction) * Math.exp(-(ratio * ratio));
    } else {
      stribeck = Math.max(kineticFriction, staticFriction);
    }
    double coulomb = sign * stribeck;
    double viscous = viscousFriction * velocity;
    return new FrictionResult(coulomb + viscous, false);
  }

  static final class FrictionResult {
    private final double frictionEffort;
    private final boolean stuck;

    FrictionResult(double frictionEffort, boolean stuck) {
      this.frictionEffort = frictionEffort;
      this.stuck = stuck;
    }

    double getFrictionEffort() {
      return frictionEffort;
    }

    boolean isStuck() {
      return stuck;
    }
  }
}
