package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class HoodIODev implements HoodIO {
  public static final int MOTOR_ID = 17;
  public static final int ENCODER_ID = 21;

  public static final double GEAR_RATIO = 2.67;
  public static final double ENCODER_MAGNET_OFFSET = -0.0585;
  public static final double FORWARD_LIMIT_ROTATIONS = 1.5;
  public static final double REVERSE_LIMIT_ROTATIONS = 0;
  public static final double STOWED_OUT_POSITION_THRESHOLD = 0.4;

  public static final double GRAVITY_POSITION_OFFSET = -0.08686111111;
  public static final double KP = 25.0;
  public static final double KI = 0.0;
  public static final double KD = 0.25;
  public static final double KS_STOWED = 0.60;
  public static final double KS_OUT = 0.20;
  public static final double KV = 0.15; // temp
  public static final double KA = 0.0;
  public static final double KG = 0.80;

  public static final double MM_CRUISE_VELOCITY = 2700;
  public static final double MM_ACCLERATION = 16;

  protected final TalonFX motor = new TalonFX(MOTOR_ID);
  private final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(GEAR_RATIO))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitThreshold(FORWARD_LIMIT_ROTATIONS)
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(REVERSE_LIMIT_ROTATIONS)
                  .withReverseSoftLimitEnable(true))
          .withSlot0(
              new Slot0Configs()
                  .withKP(KP)
                  .withKI(KI)
                  .withKD(KD)
                  .withKS(KS_STOWED)
                  .withKV(KV)
                  .withKA(KA)
                  .withKG(KG)
                  .withGravityArmPositionOffset(GRAVITY_POSITION_OFFSET)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withSlot1(
              new Slot1Configs()
                  .withKP(KP)
                  .withKI(KI)
                  .withKD(KD)
                  .withKS(KS_OUT)
                  .withKV(KV)
                  .withKA(KA)
                  .withKG(KG)
                  .withGravityArmPositionOffset(GRAVITY_POSITION_OFFSET)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(MM_ACCLERATION)
                  .withMotionMagicCruiseVelocity(MM_CRUISE_VELOCITY));

  protected final CANcoder encoder = new CANcoder(ENCODER_ID);
  private final CANcoderConfiguration encoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(ENCODER_MAGNET_OFFSET));

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Angle> absolutePosition = encoder.getAbsolutePosition();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();

  private final VoltageOut voltageRequest =
      new VoltageOut(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);

  public HoodIODev() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, voltage, current, position, absolutePosition);
    motor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, voltage, current, position, absolutePosition);

    PhoenixUtil.refreshAll();
    tryUntilOk(5, () -> motor.setPosition(absolutePosition.getValueAsDouble()));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        position.getStatus().isOK()
            && velocity.getStatus().isOK()
            && current.getStatus().isOK()
            && voltage.getStatus().isOK()
            && temperature.getStatus().isOK();
    inputs.motorTemperature = temperature.getValueAsDouble();
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
  public void setVoltageV(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setPosition(double position) {
    var request = positionRequest.withPosition(position);
    if (this.position.getValueAsDouble() > STOWED_OUT_POSITION_THRESHOLD) {
      request.withSlot(1);
    } else {
      request.withSlot(0);
    }
    motor.setControl(request);
  }
}
