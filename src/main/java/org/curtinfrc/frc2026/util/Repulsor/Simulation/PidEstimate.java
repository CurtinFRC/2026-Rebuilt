package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Estimated PID gains from a simplified model. */
public final class PidEstimate {
  private final double kP;
  private final double kI;
  private final double kD;
  private final String notes;

  /**
   * Creates a PID estimate.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param notes optional notes
   */
  public PidEstimate(double kP, double kI, double kD, String notes) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.notes = notes != null ? notes : "";
  }

  /**
   * Returns the proportional gain.
   *
   * @return kP gain
   */
  public double getKP() {
    return kP;
  }

  /**
   * Returns the integral gain.
   *
   * @return kI gain
   */
  public double getKI() {
    return kI;
  }

  /**
   * Returns the derivative gain.
   *
   * @return kD gain
   */
  public double getKD() {
    return kD;
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
