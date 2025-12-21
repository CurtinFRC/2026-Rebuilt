package org.curtinfrc.frc2026.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.curtinfrc.frc2026.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

    PhoenixUtil.registerSignals(true, yaw, yawVelocity);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = yaw.getStatus().isOK() && yawVelocity.getStatus().isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
