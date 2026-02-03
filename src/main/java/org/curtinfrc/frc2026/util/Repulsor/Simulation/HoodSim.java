package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Hood simulation as a specialized arm model. */
public final class HoodSim extends ArmSim {
  /**
   * Creates a hood simulation.
   *
   * @param params mechanism parameters
   * @param massKg hood mass in kilograms
   * @param cgRadiusMeters center of gravity distance in meters
   * @param gravityAccel gravity acceleration in meters per second squared
   * @param gravityOffsetRad gravity offset in radians
   */
  public HoodSim(
      MechanismSimParams params,
      double massKg,
      double cgRadiusMeters,
      double gravityAccel,
      double gravityOffsetRad) {
    super(params, massKg, cgRadiusMeters, gravityAccel, gravityOffsetRad);
  }
}
