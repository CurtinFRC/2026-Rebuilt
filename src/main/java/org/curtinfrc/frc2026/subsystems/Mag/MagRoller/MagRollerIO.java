package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import org.littletonrobotics.junction.AutoLog;

public interface MagRollerIO {
  @AutoLog
  public static class MagRollerIOInputs {
    double appliedVolts;
    double currentAmps;
    double positionRotations;
    double angularVelocityRotationsPerMinute;
    double setPoint;
  }

  public default void setVoltage(double volts) {}

  public default void updateInputs(MagRollerIOInputs inputs) {}

  public default double getPosition() {
    return 0;
  }

  public default void setPosition(double position) {}

  public default void setVelocityRPS(double targetVelocityRPS) {}
}
