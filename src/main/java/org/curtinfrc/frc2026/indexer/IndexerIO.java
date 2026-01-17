package org.curtinfrc.frc2026.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerSecond;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double targetSpeed) {}
}
