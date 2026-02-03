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
