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

import java.util.Random;

/** Simulated incremental encoder with optional quantization and noise. */
public class SimEncoder {
  private final double countsPerUnit;
  private final double positionNoiseStdDev;
  private final double velocityNoiseStdDev;
  private final Random rng;
  private boolean inverted;
  private double position;
  private double velocity;

  /**
   * Creates an encoder with a counts-per-unit scale.
   *
   * @param countsPerUnit counts per output unit
   */
  public SimEncoder(double countsPerUnit) {
    this(countsPerUnit, 0.0, 0.0, new Random());
  }

  /**
   * Creates an encoder with quantization and noise.
   *
   * @param countsPerUnit counts per output unit
   * @param positionNoiseStdDev position noise standard deviation
   * @param velocityNoiseStdDev velocity noise standard deviation
   * @param seed random seed
   */
  public SimEncoder(
      double countsPerUnit, double positionNoiseStdDev, double velocityNoiseStdDev, long seed) {
    this(countsPerUnit, positionNoiseStdDev, velocityNoiseStdDev, new Random(seed));
  }

  private SimEncoder(
      double countsPerUnit, double positionNoiseStdDev, double velocityNoiseStdDev, Random rng) {
    this.countsPerUnit = countsPerUnit;
    this.positionNoiseStdDev = Math.max(0.0, positionNoiseStdDev);
    this.velocityNoiseStdDev = Math.max(0.0, velocityNoiseStdDev);
    this.rng = rng != null ? rng : new Random();
  }

  /**
   * Sets whether the encoder is inverted.
   *
   * @param inverted true to invert position and velocity
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  /**
   * Updates the encoder with mechanism position and velocity.
   *
   * @param positionUnits position in output units
   * @param velocityUnitsPerSec velocity in output units per second
   */
  public void update(double positionUnits, double velocityUnitsPerSec) {
    double pos = inverted ? -positionUnits : positionUnits;
    double vel = inverted ? -velocityUnitsPerSec : velocityUnitsPerSec;
    pos = quantize(pos);
    vel = quantize(vel);
    position = pos + noise(positionNoiseStdDev);
    velocity = vel + noise(velocityNoiseStdDev);
  }

  /**
   * Returns the measured position in output units.
   *
   * @return position in output units
   */
  public double getPosition() {
    return position;
  }

  /**
   * Returns the measured velocity in output units per second.
   *
   * @return velocity in output units per second
   */
  public double getVelocity() {
    return velocity;
  }

  /**
   * Returns the measured position in counts.
   *
   * @return position in counts
   */
  public double getPositionCounts() {
    return countsPerUnit != 0.0 ? position * countsPerUnit : 0.0;
  }

  /**
   * Returns the measured velocity in counts per second.
   *
   * @return velocity in counts per second
   */
  public double getVelocityCountsPerSecond() {
    return countsPerUnit != 0.0 ? velocity * countsPerUnit : 0.0;
  }

  /**
   * Returns the counts per unit scale.
   *
   * @return counts per output unit
   */
  public double getCountsPerUnit() {
    return countsPerUnit;
  }

  private double quantize(double value) {
    if (countsPerUnit <= 0.0) {
      return value;
    }
    return Math.round(value * countsPerUnit) / countsPerUnit;
  }

  private double noise(double stdDev) {
    if (stdDev <= 0.0) {
      return 0.0;
    }
    return rng.nextGaussian() * stdDev;
  }
}
