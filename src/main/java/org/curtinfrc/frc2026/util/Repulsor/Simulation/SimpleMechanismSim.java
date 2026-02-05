/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

/** Base class for single degree of freedom mechanisms driven by a DC motor. */
public abstract class SimpleMechanismSim extends MechanismSimBase implements MechanismSimIO {
  private final DCMotor motor;
  private final double gearRatio;
  private final double efficiency;
  private final double motorRadPerUnit;
  private final double outputEffortPerMotorTorque;

  /**
   * Creates a new motor-driven mechanism simulation.
   *
   * @param params mechanism parameters
   * @param motorRadPerUnit motor radians per output unit
   * @param outputEffortPerMotorTorque output effort per motor torque
   */
  protected SimpleMechanismSim(
      MechanismSimParams params, double motorRadPerUnit, double outputEffortPerMotorTorque) {
    super(
        params.getEffectiveInertia(),
        params.getFrictionModel(),
        params.getHardStopConfig(),
        params.getMaxVoltage(),
        params.getCurrentLimitAmps());
    motor = params.getMotor();
    gearRatio = params.getGearRatio();
    efficiency = params.getEfficiency();
    this.motorRadPerUnit = motorRadPerUnit;
    this.outputEffortPerMotorTorque = outputEffortPerMotorTorque;
  }

  /**
   * Returns the motor model.
   *
   * @return motor model
   */
  public DCMotor getMotorModel() {
    return motor;
  }

  /**
   * Returns the gear ratio as motor rotations per mechanism rotation.
   *
   * @return gear ratio
   */
  public double getGearRatio() {
    return gearRatio;
  }

  /**
   * Returns the gear train efficiency.
   *
   * @return efficiency from 0 to 1
   */
  public double getEfficiency() {
    return efficiency;
  }

  /**
   * Sets a motor command as a percent output.
   *
   * @param percent percent output in the range [-1, 1]
   */
  @Override
  public void setMotorCommand(double percent) {
    setInputVoltage(MathUtil.clamp(percent, -1.0, 1.0) * getMaxVoltage());
  }

  /**
   * Returns the position in output units.
   *
   * @return position in output units
   */
  public double getPositionRadOrMeters() {
    return getPosition();
  }

  /**
   * Returns the velocity in output units per second.
   *
   * @return velocity in output units per second
   */
  public double getVelocityRadPerSecOrMps() {
    return getVelocity();
  }

  /**
   * Returns the simulated motor current.
   *
   * @return motor current in amps
   */
  public double getCurrentAmps() {
    return getMotorCurrentAmps();
  }

  @Override
  protected ActuatorOutput calculateActuatorOutput(double outputVelocity, double inputVoltage) {
    double motorSpeed = outputVelocity * motorRadPerUnit;
    double motorCurrent = motor.getCurrent(motorSpeed, inputVoltage);
    double limit = getCurrentLimitAmps();
    if (Double.isFinite(limit)) {
      motorCurrent = MathUtil.clamp(motorCurrent, -limit, limit);
    }
    double motorTorque = motor.getTorque(motorCurrent);
    double outputEffort = motorTorque * outputEffortPerMotorTorque;
    return new ActuatorOutput(outputEffort, motorCurrent, motorTorque);
  }
}

