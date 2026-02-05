/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Linear mechanism simulation for a single motor with a drum or pulley. */
public class SingleMotorLinearSim extends SimpleMechanismSim {
  private final double drumRadiusMeters;

  /**
   * Creates a linear simulation.
   *
   * @param params mechanism parameters
   */
  public SingleMotorLinearSim(MechanismSimParams params) {
    super(
        params,
        params.getGearRatio() / Math.max(1e-6, params.getDrumRadiusMeters()),
        params.getGearRatio()
            * params.getEfficiency()
            / Math.max(1e-6, params.getDrumRadiusMeters()));
    if (params.getDrumRadiusMeters() <= 0.0) {
      throw new IllegalArgumentException("drumRadiusMeters");
    }
    drumRadiusMeters = params.getDrumRadiusMeters();
  }

  /**
   * Returns the drum radius.
   *
   * @return drum radius in meters
   */
  public double getDrumRadiusMeters() {
    return drumRadiusMeters;
  }

  /**
   * Returns the position in meters.
   *
   * @return position in meters
   */
  public double getPositionMeters() {
    return getPosition();
  }

  /**
   * Returns the velocity in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond() {
    return getVelocity();
  }
}

