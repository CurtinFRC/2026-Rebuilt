package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import java.util.Arrays;

/** Configuration for feedforward estimation tests. */
public final class FeedforwardTestConfig {
  private final double dtSeconds;
  private final double settleTimeSeconds;
  private final double sampleTimeSeconds;
  private final double[] quasistaticVoltages;
  private final double stepVoltage;
  private final double stepDurationSeconds;
  private final double accelSampleTimeSeconds;
  private final double minVelocityForFit;
  private final double gravityPosition;
  private final boolean estimateKg;

  private FeedforwardTestConfig(Builder builder) {
    dtSeconds = builder.dtSeconds;
    settleTimeSeconds = builder.settleTimeSeconds;
    sampleTimeSeconds = builder.sampleTimeSeconds;
    quasistaticVoltages =
        Arrays.copyOf(builder.quasistaticVoltages, builder.quasistaticVoltages.length);
    stepVoltage = builder.stepVoltage;
    stepDurationSeconds = builder.stepDurationSeconds;
    accelSampleTimeSeconds = builder.accelSampleTimeSeconds;
    minVelocityForFit = builder.minVelocityForFit;
    gravityPosition = builder.gravityPosition;
    estimateKg = builder.estimateKg;
  }

  /**
   * Returns the simulation timestep used for tests.
   *
   * @return timestep in seconds
   */
  public double getDtSeconds() {
    return dtSeconds;
  }

  /**
   * Returns the settle time for quasistatic tests.
   *
   * @return settle time in seconds
   */
  public double getSettleTimeSeconds() {
    return settleTimeSeconds;
  }

  /**
   * Returns the sample time for quasistatic tests.
   *
   * @return sample time in seconds
   */
  public double getSampleTimeSeconds() {
    return sampleTimeSeconds;
  }

  /**
   * Returns the voltage list for quasistatic tests.
   *
   * @return array of voltages
   */
  public double[] getQuasistaticVoltages() {
    return Arrays.copyOf(quasistaticVoltages, quasistaticVoltages.length);
  }

  /**
   * Returns the step voltage for dynamic tests.
   *
   * @return step voltage
   */
  public double getStepVoltage() {
    return stepVoltage;
  }

  /**
   * Returns the duration of the step test.
   *
   * @return step duration in seconds
   */
  public double getStepDurationSeconds() {
    return stepDurationSeconds;
  }

  /**
   * Returns the duration used to sample acceleration.
   *
   * @return acceleration sample time in seconds
   */
  public double getAccelSampleTimeSeconds() {
    return accelSampleTimeSeconds;
  }

  /**
   * Returns the minimum velocity used for the fit.
   *
   * @return minimum velocity
   */
  public double getMinVelocityForFit() {
    return minVelocityForFit;
  }

  /**
   * Returns the position used for gravity estimation.
   *
   * @return gravity position in output units
   */
  public double getGravityPosition() {
    return gravityPosition;
  }

  /**
   * Returns whether gravity estimation is enabled.
   *
   * @return true if gravity estimation is enabled
   */
  public boolean isEstimateKg() {
    return estimateKg;
  }

  /**
   * Creates a new builder with default values.
   *
   * @return builder instance
   */
  public static Builder builder() {
    return new Builder();
  }

  /** Builder for {@link FeedforwardTestConfig}. */
  public static final class Builder {
    private double dtSeconds = 0.005;
    private double settleTimeSeconds = 0.75;
    private double sampleTimeSeconds = 0.2;
    private double[] quasistaticVoltages = new double[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    private double stepVoltage = 6.0;
    private double stepDurationSeconds = 0.3;
    private double accelSampleTimeSeconds = 0.05;
    private double minVelocityForFit = 1e-3;
    private double gravityPosition = 0.0;
    private boolean estimateKg = false;

    /**
     * Sets the timestep.
     *
     * @param dtSeconds timestep in seconds
     * @return builder instance
     */
    public Builder withDtSeconds(double dtSeconds) {
      this.dtSeconds = Math.max(1e-4, dtSeconds);
      return this;
    }

    /**
     * Sets the settle time for quasistatic tests.
     *
     * @param settleTimeSeconds settle time in seconds
     * @return builder instance
     */
    public Builder withSettleTimeSeconds(double settleTimeSeconds) {
      this.settleTimeSeconds = Math.max(0.0, settleTimeSeconds);
      return this;
    }

    /**
     * Sets the sample time for quasistatic tests.
     *
     * @param sampleTimeSeconds sample time in seconds
     * @return builder instance
     */
    public Builder withSampleTimeSeconds(double sampleTimeSeconds) {
      this.sampleTimeSeconds = Math.max(0.0, sampleTimeSeconds);
      return this;
    }

    /**
     * Sets the voltage list for quasistatic tests.
     *
     * @param quasistaticVoltages array of voltages
     * @return builder instance
     */
    public Builder withQuasistaticVoltages(double[] quasistaticVoltages) {
      this.quasistaticVoltages =
          quasistaticVoltages != null
              ? Arrays.copyOf(quasistaticVoltages, quasistaticVoltages.length)
              : new double[0];
      return this;
    }

    /**
     * Sets the step voltage used to estimate kA.
     *
     * @param stepVoltage step voltage
     * @return builder instance
     */
    public Builder withStepVoltage(double stepVoltage) {
      this.stepVoltage = stepVoltage;
      return this;
    }

    /**
     * Sets the step duration used to estimate kA.
     *
     * @param stepDurationSeconds step duration in seconds
     * @return builder instance
     */
    public Builder withStepDurationSeconds(double stepDurationSeconds) {
      this.stepDurationSeconds = Math.max(0.0, stepDurationSeconds);
      return this;
    }

    /**
     * Sets the acceleration sample window.
     *
     * @param accelSampleTimeSeconds acceleration sample time in seconds
     * @return builder instance
     */
    public Builder withAccelSampleTimeSeconds(double accelSampleTimeSeconds) {
      this.accelSampleTimeSeconds = Math.max(0.0, accelSampleTimeSeconds);
      return this;
    }

    /**
     * Sets the minimum velocity used for the linear fit.
     *
     * @param minVelocityForFit minimum velocity
     * @return builder instance
     */
    public Builder withMinVelocityForFit(double minVelocityForFit) {
      this.minVelocityForFit = Math.max(0.0, minVelocityForFit);
      return this;
    }

    /**
     * Enables gravity estimation at the specified position.
     *
     * @param gravityPosition position to use for gravity estimation
     * @return builder instance
     */
    public Builder withGravityEstimateAt(double gravityPosition) {
      this.gravityPosition = gravityPosition;
      this.estimateKg = true;
      return this;
    }

    /**
     * Disables gravity estimation.
     *
     * @return builder instance
     */
    public Builder withoutGravityEstimate() {
      this.estimateKg = false;
      return this;
    }

    /**
     * Builds the configuration.
     *
     * @return feedforward test configuration
     */
    public FeedforwardTestConfig build() {
      return new FeedforwardTestConfig(this);
    }
  }
}
