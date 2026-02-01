package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import edu.wpi.first.units.measure.Voltage;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean[] motorsConnected;
    public double[] motorTemperatures;
    public double appliedVolts;
    public double currentAmps;
    public double velocityMetresPerSecond;
    public double accelerationMetresPerSecondPerSecond;
    public double positionRotations;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVoltageV(Voltage voltage) {}

  public default void setVelocity(double velocity, BooleanSupplier f) {}
}
