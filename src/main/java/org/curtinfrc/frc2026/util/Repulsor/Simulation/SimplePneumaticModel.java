package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import edu.wpi.first.math.MathUtil;

/**
 * Simple first-order pneumatic model that produces a bounded force or torque based on a command.
 */
public final class SimplePneumaticModel implements PressureModel {
  private final double pistonAreaSqM;
  private double supplyPressureKpa;
  private double timeConstantSeconds;
  private double effort;
  private double pressureKpa;
  private double flowRate;

  /**
   * Creates a pneumatic model.
   *
   * @param supplyPressureKpa supply pressure in kilopascals
   * @param pistonAreaSqM piston area in square meters
   * @param timeConstantSeconds response time constant in seconds
   */
  public SimplePneumaticModel(
      double supplyPressureKpa, double pistonAreaSqM, double timeConstantSeconds) {
    this.pistonAreaSqM = Math.max(0.0, pistonAreaSqM);
    this.supplyPressureKpa = Math.max(0.0, supplyPressureKpa);
    this.timeConstantSeconds = Math.max(1e-3, timeConstantSeconds);
  }

  /**
   * Sets the supply pressure.
   *
   * @param supplyPressureKpa supply pressure in kilopascals
   */
  public void setSupplyPressureKpa(double supplyPressureKpa) {
    this.supplyPressureKpa = Math.max(0.0, supplyPressureKpa);
  }

  /**
   * Sets the pneumatic time constant.
   *
   * @param timeConstantSeconds response time constant in seconds
   */
  public void setTimeConstantSeconds(double timeConstantSeconds) {
    this.timeConstantSeconds = Math.max(1e-3, timeConstantSeconds);
  }

  /**
   * Returns the piston area.
   *
   * @return piston area in square meters
   */
  public double getPistonAreaSqM() {
    return pistonAreaSqM;
  }

  /**
   * Returns the supply pressure.
   *
   * @return supply pressure in kilopascals
   */
  public double getSupplyPressureKpa() {
    return supplyPressureKpa;
  }

  /**
   * Returns the time constant.
   *
   * @return response time constant in seconds
   */
  public double getTimeConstantSeconds() {
    return timeConstantSeconds;
  }

  @Override
  public double update(double dtSeconds, double command, MechanismSimState state) {
    double clamped = MathUtil.clamp(command, -1.0, 1.0);
    double maxEffort = supplyPressureKpa * 1000.0 * pistonAreaSqM;
    double targetEffort = clamped * maxEffort;
    double alpha = dtSeconds / (timeConstantSeconds + dtSeconds);
    effort += (targetEffort - effort) * alpha;
    pressureKpa = maxEffort > 0.0 ? Math.abs(effort) / maxEffort * supplyPressureKpa : 0.0;
    flowRate = (targetEffort - effort) / timeConstantSeconds;
    return effort;
  }

  @Override
  public double getPressureKpa() {
    return pressureKpa;
  }

  @Override
  public double getFlowRate() {
    return flowRate;
  }

  @Override
  public void reset() {
    effort = 0.0;
    pressureKpa = 0.0;
    flowRate = 0.0;
  }
}
