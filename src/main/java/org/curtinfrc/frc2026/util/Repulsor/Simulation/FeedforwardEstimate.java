/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Estimated feedforward gains from simulation-based identification. */
public final class FeedforwardEstimate {
  private final double kS;
  private final double kV;
  private final double kA;
  private final double kG;
  private final boolean hasKg;
  private final String notes;

  /**
   * Creates a feedforward estimate.
   *
   * @param kS static gain
   * @param kV velocity gain
   * @param kA acceleration gain
   * @param kG gravity gain
   * @param hasKg whether kG is estimated
   * @param notes optional notes
   */
  public FeedforwardEstimate(
      double kS, double kV, double kA, double kG, boolean hasKg, String notes) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
    this.hasKg = hasKg;
    this.notes = notes != null ? notes : "";
  }

  /**
   * Returns the static gain.
   *
   * @return kS gain
   */
  public double getKS() {
    return kS;
  }

  /**
   * Returns the velocity gain.
   *
   * @return kV gain
   */
  public double getKV() {
    return kV;
  }

  /**
   * Returns the acceleration gain.
   *
   * @return kA gain
   */
  public double getKA() {
    return kA;
  }

  /**
   * Returns the gravity gain.
   *
   * @return kG gain
   */
  public double getKG() {
    return kG;
  }

  /**
   * Returns whether kG was estimated.
   *
   * @return true if kG was estimated
   */
  public boolean hasKG() {
    return hasKg;
  }

  /**
   * Returns optional notes about the estimate.
   *
   * @return notes string
   */
  public String getNotes() {
    return notes;
  }
}

