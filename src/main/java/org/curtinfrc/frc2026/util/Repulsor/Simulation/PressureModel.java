package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/**
 * Interface for pressure or pneumatic effect models.
 */
public interface PressureModel {
  /**
   * Updates the pressure model and returns the effort applied to the mechanism.
   *
   * @param dtSeconds timestep in seconds
   * @param command command value, typically in the range [-1, 1]
   * @param state current simulation state
   * @return effort applied by the pressure model
   */
  double update(double dtSeconds, double command, MechanismSimState state);

  /**
   * Returns the current pressure estimate in kilopascals.
   *
   * @return pressure in kilopascals
   */
  double getPressureKpa();

  /**
   * Returns the current flow rate estimate in unitless normalized terms.
   *
   * @return flow rate estimate
   */
  double getFlowRate();

  /** Resets any internal state to its default values. */
  void reset();
}
