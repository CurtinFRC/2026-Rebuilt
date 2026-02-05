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

import edu.wpi.first.math.MathUtil;

/** Abstract base for mechanism simulations with friction, loads, and sensor support. */
public abstract class MechanismSimBase {
  private final MechanismSimState state = new MechanismSimState();
  private double effectiveInertia;
  private double maxVoltage;
  private double currentLimitAmps;
  private double inputVoltage;
  private FrictionModel frictionModel;
  private HardStopConfig hardStopConfig;
  private LoadModel loadModel;
  private PressureModel pressureModel;
  private double pressureCommand;

  /**
   * Creates a new mechanism simulation base.
   *
   * @param effectiveInertia effective inertia or mass
   * @param frictionModel friction model
   * @param hardStopConfig hard stop configuration
   * @param maxVoltage maximum voltage
   * @param currentLimitAmps current limit in amps
   */
  protected MechanismSimBase(
      double effectiveInertia,
      FrictionModel frictionModel,
      HardStopConfig hardStopConfig,
      double maxVoltage,
      double currentLimitAmps) {
    if (effectiveInertia <= 0.0) {
      throw new IllegalArgumentException("effectiveInertia");
    }
    this.effectiveInertia = effectiveInertia;
    this.frictionModel = frictionModel != null ? frictionModel : FrictionModel.none();
    this.hardStopConfig = hardStopConfig != null ? hardStopConfig : HardStopConfig.disabled();
    this.maxVoltage = Math.max(0.0, maxVoltage);
    this.currentLimitAmps = Math.max(0.0, currentLimitAmps);
  }

  /**
   * Sets the input voltage command.
   *
   * @param voltage voltage in volts
   */
  public void setInputVoltage(double voltage) {
    this.inputVoltage = voltage;
  }

  /**
   * Returns the input voltage command.
   *
   * @return input voltage in volts
   */
  public double getInputVoltage() {
    return inputVoltage;
  }

  /**
   * Sets the maximum voltage used for saturation.
   *
   * @param maxVoltage maximum voltage in volts
   */
  public void setMaxVoltage(double maxVoltage) {
    this.maxVoltage = Math.max(0.0, maxVoltage);
  }

  /**
   * Returns the maximum voltage used for saturation.
   *
   * @return maximum voltage in volts
   */
  public double getMaxVoltage() {
    return maxVoltage;
  }

  /**
   * Sets the current limit used by motor simulations.
   *
   * @param currentLimitAmps current limit in amps
   */
  public void setCurrentLimitAmps(double currentLimitAmps) {
    this.currentLimitAmps = Math.max(0.0, currentLimitAmps);
  }

  /**
   * Returns the current limit used by motor simulations.
   *
   * @return current limit in amps
   */
  public double getCurrentLimitAmps() {
    return currentLimitAmps;
  }

  /**
   * Sets the friction model.
   *
   * @param frictionModel friction model
   */
  public void setFrictionModel(FrictionModel frictionModel) {
    this.frictionModel = frictionModel != null ? frictionModel : FrictionModel.none();
  }

  /**
   * Sets the hard stop configuration.
   *
   * @param hardStopConfig hard stop configuration
   */
  public void setHardStopConfig(HardStopConfig hardStopConfig) {
    this.hardStopConfig = hardStopConfig != null ? hardStopConfig : HardStopConfig.disabled();
  }

  /**
   * Sets the load model.
   *
   * @param loadModel load model
   */
  public void setLoadModel(LoadModel loadModel) {
    this.loadModel = loadModel;
  }

  /**
   * Sets the pressure model.
   *
   * @param pressureModel pressure model
   */
  public void setPressureModel(PressureModel pressureModel) {
    this.pressureModel = pressureModel;
    if (this.pressureModel != null) {
      this.pressureModel.reset();
    }
  }

  /**
   * Sets the pressure command value.
   *
   * @param command command value in the range [-1, 1]
   */
  public void setPressureCommand(double command) {
    this.pressureCommand = MathUtil.clamp(command, -1.0, 1.0);
  }

  /**
   * Returns the current simulation state.
   *
   * @return simulation state
   */
  public MechanismSimState getState() {
    return state;
  }

  /**
   * Returns the current position in output units.
   *
   * @return position in output units
   */
  public double getPosition() {
    return state.getPosition();
  }

  /**
   * Returns the current velocity in output units per second.
   *
   * @return velocity in output units per second
   */
  public double getVelocity() {
    return state.getVelocity();
  }

  /**
   * Returns the current acceleration in output units per second squared.
   *
   * @return acceleration in output units per second squared
   */
  public double getAcceleration() {
    return state.getAcceleration();
  }

  /**
   * Returns the applied voltage after saturation.
   *
   * @return applied voltage in volts
   */
  public double getAppliedVoltage() {
    return state.getAppliedVoltage();
  }

  /**
   * Returns the simulated motor current.
   *
   * @return motor current in amps
   */
  public double getMotorCurrentAmps() {
    return state.getMotorCurrentAmps();
  }

  /**
   * Returns the simulated motor torque.
   *
   * @return motor torque in newton meters
   */
  public double getMotorTorqueNm() {
    return state.getMotorTorqueNm();
  }

  /**
   * Returns the net output effort.
   *
   * @return output effort in torque or force units
   */
  public double getOutputEffort() {
    return state.getOutputEffort();
  }

  /**
   * Returns whether the lower limit is triggered.
   *
   * @return true if the lower limit is triggered
   */
  public boolean isLowerLimitTriggered() {
    return state.isLowerLimitTriggered();
  }

  /**
   * Returns whether the upper limit is triggered.
   *
   * @return true if the upper limit is triggered
   */
  public boolean isUpperLimitTriggered() {
    return state.isUpperLimitTriggered();
  }

  /**
   * Resets the simulation state.
   *
   * @param position position in output units
   * @param velocity velocity in output units per second
   */
  public void reset(double position, double velocity) {
    state.setPosition(position);
    state.setVelocity(velocity);
    state.setAcceleration(0.0);
  }

  /**
   * Returns a snapshot of the current simulation state.
   *
   * @return snapshot state
   */
  public MechanismSimState snapshotState() {
    return state.copy();
  }

  /**
   * Restores a simulation state snapshot.
   *
   * @param snapshot snapshot state
   */
  public void restoreState(MechanismSimState snapshot) {
    if (snapshot != null) {
      state.copyFrom(snapshot);
    }
  }

  /**
   * Advances the simulation by a timestep.
   *
   * @param dtSeconds timestep in seconds
   */
  public void update(double dtSeconds) {
    if (dtSeconds <= 0.0) {
      return;
    }
    double appliedVoltage = MathUtil.clamp(inputVoltage, -maxVoltage, maxVoltage);
    state.setAppliedVoltage(appliedVoltage);

    ActuatorOutput actuator = calculateActuatorOutput(state.getVelocity(), appliedVoltage);
    state.setMotorCurrentAmps(actuator.motorCurrentAmps);
    state.setMotorTorqueNm(actuator.motorTorqueNm);

    double pressureEffort = 0.0;
    if (pressureModel != null) {
      pressureEffort = pressureModel.update(dtSeconds, pressureCommand, state);
    }

    double externalEffort = 0.0;
    if (loadModel != null) {
      externalEffort = loadModel.getExternalEffort(state, appliedVoltage);
    }

    double contactEffort = computeContactEffort(state.getPosition(), state.getVelocity());
    double effortWithoutFriction =
        actuator.outputEffort + externalEffort + pressureEffort + contactEffort;
    FrictionModel.FrictionResult friction =
        frictionModel.compute(state.getVelocity(), effortWithoutFriction);

    double netEffort = effortWithoutFriction - friction.getFrictionEffort();
    double acceleration = 0.0;
    if (friction.isStuck()) {
      state.setVelocity(0.0);
    } else {
      acceleration = netEffort / effectiveInertia;
      state.setVelocity(state.getVelocity() + acceleration * dtSeconds);
    }

    state.setPosition(state.getPosition() + state.getVelocity() * dtSeconds);
    applyHardStops();

    state.setAcceleration(acceleration);
    state.setOutputEffort(netEffort);
    state.setExternalEffort(externalEffort + contactEffort);
    state.setFrictionEffort(friction.getFrictionEffort());
    state.setPressureEffort(pressureEffort);
    state.setLowerLimitTriggered(
        hardStopConfig.isEnabled() && state.getPosition() <= hardStopConfig.getMinPosition());
    state.setUpperLimitTriggered(
        hardStopConfig.isEnabled() && state.getPosition() >= hardStopConfig.getMaxPosition());
  }

  /**
   * Estimates feedforward constants by running tests in simulation.
   *
   * @param config feedforward test configuration
   * @return feedforward estimate
   */
  public FeedforwardEstimate estimateFeedforward(FeedforwardTestConfig config) {
    if (config == null) {
      throw new IllegalArgumentException("config");
    }

    MechanismSimState snapshot = snapshotState();
    double savedVoltage = inputVoltage;
    double savedPressure = pressureCommand;
    double startPosition = snapshot.getPosition();

    LinearFit positiveFit = new LinearFit();
    LinearFit negativeFit = new LinearFit();

    for (double voltage : config.getQuasistaticVoltages()) {
      double absVoltage = Math.abs(voltage);
      if (absVoltage <= 0.0) {
        continue;
      }
      double posVelocity = runQuasistatic(startPosition, absVoltage, config);
      if (Math.abs(posVelocity) >= config.getMinVelocityForFit()) {
        positiveFit.add(Math.abs(posVelocity), absVoltage);
      }
      double negVelocity = runQuasistatic(startPosition, -absVoltage, config);
      if (Math.abs(negVelocity) >= config.getMinVelocityForFit()) {
        negativeFit.add(Math.abs(negVelocity), absVoltage);
      }
    }

    double kS = 0.0;
    double kV = 0.0;
    int fitCount = 0;
    if (positiveFit.count() >= 2) {
      kS += positiveFit.intercept();
      kV += positiveFit.slope();
      fitCount++;
    }
    if (negativeFit.count() >= 2) {
      kS += negativeFit.intercept();
      kV += negativeFit.slope();
      fitCount++;
    }
    if (fitCount > 0) {
      kS /= fitCount;
      kV /= fitCount;
    }

    double kA = estimateKa(startPosition, config, kS, kV);
    double kG = 0.0;
    boolean hasKg = false;
    if (config.isEstimateKg()) {
      kG = estimateKg(config.getGravityPosition(), config.getDtSeconds());
      hasKg = true;
    }

    restoreState(snapshot);
    setInputVoltage(savedVoltage);
    setPressureCommand(savedPressure);

    return new FeedforwardEstimate(
        kS, kV, kA, kG, hasKg, "Estimated from simulation, validate on hardware");
  }

  /**
   * Returns a recommended velocity PID estimate based on a target bandwidth.
   *
   * @param feedforward feedforward estimate
   * @param bandwidthHz desired bandwidth in hertz
   * @return PID estimate
   */
  public PidEstimate recommendVelocityPid(FeedforwardEstimate feedforward, double bandwidthHz) {
    if (feedforward == null) {
      throw new IllegalArgumentException("feedforward");
    }
    double omega = 2.0 * Math.PI * Math.max(0.0, bandwidthHz);
    double kP = Math.max(0.0, feedforward.getKA() * omega - feedforward.getKV());
    return new PidEstimate(kP, 0.0, 0.0, "Velocity loop estimate");
  }

  /**
   * Returns a recommended position PID estimate based on a target bandwidth and damping ratio.
   *
   * @param feedforward feedforward estimate
   * @param bandwidthHz desired bandwidth in hertz
   * @param dampingRatio desired damping ratio
   * @return PID estimate
   */
  public PidEstimate recommendPositionPid(
      FeedforwardEstimate feedforward, double bandwidthHz, double dampingRatio) {
    if (feedforward == null) {
      throw new IllegalArgumentException("feedforward");
    }
    double omega = 2.0 * Math.PI * Math.max(0.0, bandwidthHz);
    double zeta = Math.max(0.0, dampingRatio);
    double kP = Math.max(0.0, feedforward.getKA() * omega * omega);
    double kD = Math.max(0.0, 2.0 * zeta * omega * feedforward.getKA() - feedforward.getKV());
    return new PidEstimate(kP, 0.0, kD, "Position loop estimate");
  }

  /**
   * Computes actuator output for the mechanism.
   *
   * @param outputVelocity output velocity in mechanism units per second
   * @param inputVoltage applied voltage in volts
   * @return actuator output
   */
  protected abstract ActuatorOutput calculateActuatorOutput(
      double outputVelocity, double inputVoltage);

  /**
   * Sets the effective inertia used by the simulation.
   *
   * @param effectiveInertia effective inertia or mass
   */
  protected void setEffectiveInertia(double effectiveInertia) {
    if (effectiveInertia <= 0.0) {
      throw new IllegalArgumentException("effectiveInertia");
    }
    this.effectiveInertia = effectiveInertia;
  }

  /**
   * Returns the effective inertia used by the simulation.
   *
   * @return effective inertia or mass
   */
  protected double getEffectiveInertia() {
    return effectiveInertia;
  }

  private double computeContactEffort(double position, double velocity) {
    if (hardStopConfig == null || !hardStopConfig.isEnabled()) {
      return 0.0;
    }
    double effort = 0.0;
    if (position < hardStopConfig.getMinPosition()) {
      double penetration = hardStopConfig.getMinPosition() - position;
      effort += hardStopConfig.getStiffness() * penetration;
      effort += -hardStopConfig.getDamping() * Math.min(velocity, 0.0);
    }
    if (position > hardStopConfig.getMaxPosition()) {
      double penetration = position - hardStopConfig.getMaxPosition();
      effort -= hardStopConfig.getStiffness() * penetration;
      effort -= hardStopConfig.getDamping() * Math.max(velocity, 0.0);
    }
    return effort;
  }

  private void applyHardStops() {
    if (hardStopConfig == null || !hardStopConfig.isEnabled()) {
      return;
    }
    double position = state.getPosition();
    double velocity = state.getVelocity();

    if (position < hardStopConfig.getMinPosition()) {
      position = hardStopConfig.getMinPosition();
      if (velocity < 0.0) {
        velocity = -velocity * hardStopConfig.getRestitution();
        velocity *= 1.0 - hardStopConfig.getImpactDamping();
      }
    }

    if (position > hardStopConfig.getMaxPosition()) {
      position = hardStopConfig.getMaxPosition();
      if (velocity > 0.0) {
        velocity = -velocity * hardStopConfig.getRestitution();
        velocity *= 1.0 - hardStopConfig.getImpactDamping();
      }
    }

    state.setPosition(position);
    state.setVelocity(velocity);
  }

  private double runQuasistatic(
      double startPosition, double voltage, FeedforwardTestConfig config) {
    reset(startPosition, 0.0);
    setInputVoltage(voltage);
    advanceFor(config.getSettleTimeSeconds(), config.getDtSeconds());
    return averageVelocity(config.getSampleTimeSeconds(), config.getDtSeconds());
  }

  private double estimateKa(
      double startPosition, FeedforwardTestConfig config, double kS, double kV) {
    reset(startPosition, 0.0);
    setInputVoltage(config.getStepVoltage());
    double dt = config.getDtSeconds();
    int steps = (int) Math.ceil(config.getStepDurationSeconds() / dt);
    int accelSteps = (int) Math.ceil(config.getAccelSampleTimeSeconds() / dt);
    accelSteps = Math.max(1, accelSteps);

    double accelSum = 0.0;
    double velSum = 0.0;
    int count = 0;

    for (int i = 0; i < steps; i++) {
      update(dt);
      if (i < accelSteps) {
        accelSum += state.getAcceleration();
        velSum += state.getVelocity();
        count++;
      }
    }

    if (count <= 0) {
      return 0.0;
    }
    double avgAccel = accelSum / count;
    double avgVel = velSum / count;
    if (Math.abs(avgAccel) < 1e-6) {
      return 0.0;
    }
    double sign = Math.signum(config.getStepVoltage());
    return (config.getStepVoltage() - kS * sign - kV * avgVel) / avgAccel;
  }

  private double estimateKg(double position, double dtSeconds) {
    double low = -maxVoltage;
    double high = maxVoltage;
    double mid = 0.0;

    for (int i = 0; i < 24; i++) {
      mid = (low + high) * 0.5;
      reset(position, 0.0);
      setInputVoltage(mid);
      update(dtSeconds);
      double accel = state.getAcceleration();
      if (accel > 0.0) {
        high = mid;
      } else {
        low = mid;
      }
    }

    return mid;
  }

  private void advanceFor(double durationSeconds, double dtSeconds) {
    int steps = (int) Math.ceil(durationSeconds / dtSeconds);
    for (int i = 0; i < steps; i++) {
      update(dtSeconds);
    }
  }

  private double averageVelocity(double durationSeconds, double dtSeconds) {
    int steps = (int) Math.ceil(durationSeconds / dtSeconds);
    steps = Math.max(1, steps);
    double sum = 0.0;
    for (int i = 0; i < steps; i++) {
      update(dtSeconds);
      sum += state.getVelocity();
    }
    return sum / steps;
  }

  static final class ActuatorOutput {
    final double outputEffort;
    final double motorCurrentAmps;
    final double motorTorqueNm;

    ActuatorOutput(double outputEffort, double motorCurrentAmps, double motorTorqueNm) {
      this.outputEffort = outputEffort;
      this.motorCurrentAmps = motorCurrentAmps;
      this.motorTorqueNm = motorTorqueNm;
    }
  }

  private static final class LinearFit {
    private double sumX;
    private double sumY;
    private double sumXX;
    private double sumXY;
    private int count;

    void add(double x, double y) {
      sumX += x;
      sumY += y;
      sumXX += x * x;
      sumXY += x * y;
      count++;
    }

    int count() {
      return count;
    }

    double slope() {
      double denom = count * sumXX - sumX * sumX;
      if (Math.abs(denom) < 1e-9) {
        return 0.0;
      }
      return (count * sumXY - sumX * sumY) / denom;
    }

    double intercept() {
      if (count <= 0) {
        return 0.0;
      }
      return (sumY - slope() * sumX) / count;
    }
  }
}
