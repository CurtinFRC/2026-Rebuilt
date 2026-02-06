
package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import org.curtinfrc.frc2026.sim.BallSim;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean[] motorsConnected = new boolean[4];
    public double[] motorTemperatures = new double[4];
    public double appliedVolts;
    public double currentAmps;
    public double velocityMetresPerSecond;
    public double accelerationMetresPerSecondPerSecond;
    public double positionRotations;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity, boolean shooting) {}

  public default void addSimBall(BallSim ball) {}
}
