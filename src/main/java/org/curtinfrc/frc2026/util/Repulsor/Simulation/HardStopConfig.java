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

/** Configuration for hard travel limits with spring contact and impact restitution. */
public final class HardStopConfig {
  private final double minPosition;
  private final double maxPosition;
  private final double stiffness;
  private final double damping;
  private final double restitution;
  private final double impactDamping;

  /**
   * Creates a hard stop configuration.
   *
   * @param minPosition minimum position in output units
   * @param maxPosition maximum position in output units
   * @param stiffness contact stiffness in effort per unit
   * @param damping contact damping in effort per unit per second
   * @param restitution coefficient of restitution for impacts
   * @param impactDamping damping applied after impact, from 0 to 1
   */
  public HardStopConfig(
      double minPosition,
      double maxPosition,
      double stiffness,
      double damping,
      double restitution,
      double impactDamping) {
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
    this.stiffness = Math.max(0.0, stiffness);
    this.damping = Math.max(0.0, damping);
    this.restitution = Math.max(0.0, Math.min(1.0, restitution));
    this.impactDamping = Math.max(0.0, Math.min(1.0, impactDamping));
  }

  /**
   * Returns a disabled hard stop configuration.
   *
   * @return disabled hard stop configuration
   */
  public static HardStopConfig disabled() {
    return new HardStopConfig(
        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Returns the minimum position.
   *
   * @return minimum position in output units
   */
  public double getMinPosition() {
    return minPosition;
  }

  /**
   * Returns the maximum position.
   *
   * @return maximum position in output units
   */
  public double getMaxPosition() {
    return maxPosition;
  }

  /**
   * Returns the contact stiffness.
   *
   * @return contact stiffness in effort per unit
   */
  public double getStiffness() {
    return stiffness;
  }

  /**
   * Returns the contact damping.
   *
   * @return contact damping in effort per unit per second
   */
  public double getDamping() {
    return damping;
  }

  /**
   * Returns the coefficient of restitution.
   *
   * @return restitution coefficient
   */
  public double getRestitution() {
    return restitution;
  }

  /**
   * Returns the impact damping factor applied after restitution.
   *
   * @return impact damping factor from 0 to 1
   */
  public double getImpactDamping() {
    return impactDamping;
  }

  /**
   * Returns whether the configuration represents an enabled hard stop.
   *
   * @return true if limits are finite or stiffness is nonzero
   */
  public boolean isEnabled() {
    return Double.isFinite(minPosition)
        || Double.isFinite(maxPosition)
        || stiffness > 0.0
        || damping > 0.0;
  }
}
