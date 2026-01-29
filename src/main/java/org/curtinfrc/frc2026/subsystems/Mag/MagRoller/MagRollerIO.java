package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import org.littletonrobotics.junction.AutoLog;

public interface MagRollerIO {
  @AutoLog
  public static class MagRollerIOInputs {
    public double appliedVolts;
    public double currentAmps;
    public double positionRotations;
    public double angularVelocityRotationsPerMinute;
  }

  public default void setVoltage(double volts) {}

  public default void updateInputs(MagRollerIOInputs inputs) {}

  public default double getPosition() {
    return 0;
  }

  public default void setPosition(double position) {}
}
