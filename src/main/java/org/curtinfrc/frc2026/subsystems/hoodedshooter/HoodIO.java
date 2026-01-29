package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected;
    public double motorTemperature;
    public double positionRotations;
    public double absolutePositionRotations;
    public double angularVelocityRotationsPerSecond;
    public double currentAmps;
    public double appliedVolts;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVoltageV(Voltage voltage) {}

  public default void setPosition(double position) {}
}
