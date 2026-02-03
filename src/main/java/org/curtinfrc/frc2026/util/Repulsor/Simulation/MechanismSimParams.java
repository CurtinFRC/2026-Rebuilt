package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Objects;

/**
 * Parameter set for mechanism simulations.
 */
public final class MechanismSimParams {
  private final DCMotor motor;
  private final double gearRatio;
  private final double efficiency;
  private final double effectiveInertia;
  private final double massKg;
  private final double drumRadiusMeters;
  private final FrictionModel frictionModel;
  private final HardStopConfig hardStopConfig;
  private final double maxVoltage;
  private final double currentLimitAmps;

  private MechanismSimParams(Builder builder) {
    motor = Objects.requireNonNull(builder.motor, "motor");
    gearRatio = builder.gearRatio;
    efficiency = builder.efficiency;
    effectiveInertia = builder.effectiveInertia;
    massKg = builder.massKg;
    drumRadiusMeters = builder.drumRadiusMeters;
    frictionModel = builder.frictionModel;
    hardStopConfig = builder.hardStopConfig;
    maxVoltage = builder.maxVoltage;
    currentLimitAmps = builder.currentLimitAmps;
  }

  /**
   * Creates a builder with a required motor model.
   *
   * @param motor motor model
   * @return builder instance
   */
  public static Builder builder(DCMotor motor) {
    return new Builder(motor);
  }

  /**
   * Creates a builder configured for a rotary mechanism.
   *
   * @param motor motor model
   * @param gearRatio motor rotations per mechanism rotation
   * @param inertiaKgM2 effective inertia in kilogram meters squared
   * @return builder instance
   */
  public static Builder rotary(DCMotor motor, double gearRatio, double inertiaKgM2) {
    return new Builder(motor).withGearRatio(gearRatio).withEffectiveInertia(inertiaKgM2);
  }

  /**
   * Creates a builder configured for a linear mechanism.
   *
   * @param motor motor model
   * @param gearRatio motor rotations per drum rotation
   * @param massKg effective linear mass in kilograms
   * @param drumRadiusMeters drum radius in meters
   * @return builder instance
   */
  public static Builder linear(
      DCMotor motor, double gearRatio, double massKg, double drumRadiusMeters) {
    return new Builder(motor)
        .withGearRatio(gearRatio)
        .withMassKg(massKg)
        .withDrumRadiusMeters(drumRadiusMeters);
  }

  /**
   * Returns the motor model.
   *
   * @return motor model
   */
  public DCMotor getMotor() {
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
   * Returns the effective inertia for the simulation.
   *
   * <p>For rotary mechanisms this is moment of inertia in kilogram meters squared. For linear
   * mechanisms this is the mass in kilograms.
   *
   * @return effective inertia or mass
   */
  public double getEffectiveInertia() {
    return effectiveInertia;
  }

  /**
   * Returns the configured mass in kilograms.
   *
   * @return mass in kilograms
   */
  public double getMassKg() {
    return massKg;
  }

  /**
   * Returns the configured drum radius.
   *
   * @return drum radius in meters
   */
  public double getDrumRadiusMeters() {
    return drumRadiusMeters;
  }

  /**
   * Returns the friction model.
   *
   * @return friction model
   */
  public FrictionModel getFrictionModel() {
    return frictionModel;
  }

  /**
   * Returns the hard stop configuration.
   *
   * @return hard stop configuration
   */
  public HardStopConfig getHardStopConfig() {
    return hardStopConfig;
  }

  /**
   * Returns the maximum voltage.
   *
   * @return maximum voltage
   */
  public double getMaxVoltage() {
    return maxVoltage;
  }

  /**
   * Returns the current limit.
   *
   * @return current limit in amps
   */
  public double getCurrentLimitAmps() {
    return currentLimitAmps;
  }

  /** Builder for {@link MechanismSimParams}. */
  public static final class Builder {
    private final DCMotor motor;
    private double gearRatio = 1.0;
    private double efficiency = 1.0;
    private double effectiveInertia = 0.01;
    private double massKg = 1.0;
    private double drumRadiusMeters = 0.05;
    private FrictionModel frictionModel = FrictionModel.none();
    private HardStopConfig hardStopConfig = HardStopConfig.disabled();
    private double maxVoltage = 12.0;
    private double currentLimitAmps = Double.POSITIVE_INFINITY;

    private Builder(DCMotor motor) {
      this.motor = Objects.requireNonNull(motor, "motor");
    }

    /**
     * Sets the gear ratio as motor rotations per mechanism rotation.
     *
     * @param gearRatio gear ratio
     * @return builder instance
     */
    public Builder withGearRatio(double gearRatio) {
      this.gearRatio = gearRatio;
      return this;
    }

    /**
     * Sets the gear train efficiency.
     *
     * @param efficiency efficiency from 0 to 1
     * @return builder instance
     */
    public Builder withEfficiency(double efficiency) {
      this.efficiency = Math.max(0.0, Math.min(1.0, efficiency));
      return this;
    }

    /**
     * Sets the effective inertia for the mechanism.
     *
     * @param effectiveInertia effective inertia or mass
     * @return builder instance
     */
    public Builder withEffectiveInertia(double effectiveInertia) {
      this.effectiveInertia = effectiveInertia;
      return this;
    }

    /**
     * Sets the mass for a linear mechanism.
     *
     * @param massKg mass in kilograms
     * @return builder instance
     */
    public Builder withMassKg(double massKg) {
      this.massKg = massKg;
      this.effectiveInertia = massKg;
      return this;
    }

    /**
     * Sets the drum radius for a linear mechanism.
     *
     * @param drumRadiusMeters drum radius in meters
     * @return builder instance
     */
    public Builder withDrumRadiusMeters(double drumRadiusMeters) {
      this.drumRadiusMeters = drumRadiusMeters;
      return this;
    }

    /**
     * Sets the friction model.
     *
     * @param frictionModel friction model
     * @return builder instance
     */
    public Builder withFrictionModel(FrictionModel frictionModel) {
      this.frictionModel = frictionModel != null ? frictionModel : FrictionModel.none();
      return this;
    }

    /**
     * Sets the hard stop configuration.
     *
     * @param hardStopConfig hard stop configuration
     * @return builder instance
     */
    public Builder withHardStopConfig(HardStopConfig hardStopConfig) {
      this.hardStopConfig = hardStopConfig != null ? hardStopConfig : HardStopConfig.disabled();
      return this;
    }

    /**
     * Sets the maximum voltage.
     *
     * @param maxVoltage maximum voltage
     * @return builder instance
     */
    public Builder withMaxVoltage(double maxVoltage) {
      this.maxVoltage = Math.max(0.0, maxVoltage);
      return this;
    }

    /**
     * Sets the current limit in amps.
     *
     * @param currentLimitAmps current limit in amps
     * @return builder instance
     */
    public Builder withCurrentLimitAmps(double currentLimitAmps) {
      this.currentLimitAmps = Math.max(0.0, currentLimitAmps);
      return this;
    }

    /**
     * Builds the parameter object.
     *
     * @return parameter object
     */
    public MechanismSimParams build() {
      if (effectiveInertia <= 0.0) {
        throw new IllegalArgumentException("effectiveInertia");
      }
      return new MechanismSimParams(this);
    }
  }
}
