package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import org.curtinfrc.frc2026.util.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodIODev implements HoodIO {
  public static final int MOTOR_ID = 16;
  public static final int ENCODER_ID = 17;
  public static final double GEAR_RATIO = 0.375;

  protected final TalonFX motor = new TalonFX(MOTOR_ID);
  private final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(ENCODER_ID) // Ties encoder with motor
                  .withFeedbackSensorSource(
                      FeedbackSensorSourceValue.FusedCANcoder)) // Ties encoder with motor
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60))
          .withSlot0(new Slot0Configs().withKP(2.4).withKI(0.0).withKD(0.1));

  protected final CANcoder encoder = new CANcoder(ENCODER_ID);
  private final CANcoderConfiguration encoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(0));

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Angle> absolutePosition = encoder.getAbsolutePosition();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

  public HoodIODev() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, voltage, current, position, absolutePosition);
    motor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, voltage, current, position, absolutePosition);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.absolutePositionRotations = absolutePosition.getValueAsDouble();
    inputs.angularVelocityRotationsPerSecond = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }
}
