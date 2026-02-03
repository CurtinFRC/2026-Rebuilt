package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Simulated absolute encoder with wrapping. */
public final class SimAbsoluteEncoder {
  private final SimEncoder encoder;
  private final double rangeMin;
  private final double rangeMax;
  private final double range;
  private double offset;

  /**
   * Creates an absolute encoder with a range and quantization.
   *
   * @param rangeMin minimum absolute value
   * @param rangeMax maximum absolute value
   * @param countsPerUnit counts per output unit
   */
  public SimAbsoluteEncoder(double rangeMin, double rangeMax, double countsPerUnit) {
    this(rangeMin, rangeMax, countsPerUnit, 0.0, 0.0, 0L);
  }

  /**
   * Creates an absolute encoder with noise and quantization.
   *
   * @param rangeMin minimum absolute value
   * @param rangeMax maximum absolute value
   * @param countsPerUnit counts per output unit
   * @param positionNoiseStdDev position noise standard deviation
   * @param velocityNoiseStdDev velocity noise standard deviation
   * @param seed random seed
   */
  public SimAbsoluteEncoder(
      double rangeMin,
      double rangeMax,
      double countsPerUnit,
      double positionNoiseStdDev,
      double velocityNoiseStdDev,
      long seed) {
    if (rangeMax <= rangeMin) {
      throw new IllegalArgumentException("range");
    }
    this.rangeMin = rangeMin;
    this.rangeMax = rangeMax;
    this.range = rangeMax - rangeMin;
    this.encoder = new SimEncoder(countsPerUnit, positionNoiseStdDev, velocityNoiseStdDev, seed);
  }

  /**
   * Sets an offset applied before wrapping.
   *
   * @param offset offset in output units
   */
  public void setOffset(double offset) {
    this.offset = offset;
  }

  /**
   * Updates the encoder with mechanism position and velocity.
   *
   * @param positionUnits position in output units
   * @param velocityUnitsPerSec velocity in output units per second
   */
  public void update(double positionUnits, double velocityUnitsPerSec) {
    encoder.update(positionUnits + offset, velocityUnitsPerSec);
  }

  /**
   * Returns the wrapped absolute position.
   *
   * @return absolute position in output units
   */
  public double getAbsolutePosition() {
    return wrap(encoder.getPosition());
  }

  /**
   * Returns the measured velocity.
   *
   * @return velocity in output units per second
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  private double wrap(double value) {
    double wrapped = value - range * Math.floor((value - rangeMin) / range);
    if (wrapped >= rangeMax) {
      wrapped -= range;
    }
    return wrapped;
  }
}
