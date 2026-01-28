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
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class HoodIODev implements HoodIO {
  public static final int MOTOR_ID = 17;
  public static final int ENCODER_ID = 21;
  public static final double GEAR_RATIO = 2.67;
  public static final double ENCODER_MAGNET_OFFSET = -0.0585;
  public static final double FORWARD_LIMIT_ROTATIONS = 1.575;
  public static final double REVERSE_LIMIT_ROTATIONS = 0;

  public static final double GRAVITY_POSITION_OFFSET = -0.08686111111;

  public static final double K_P = 20.0;
  public static final double K_I = 0.0;
  public static final double K_D = 0.0;
  public static final double K_S_STOWED = 0.0;
  public static final double K_S_OUT = 0.0;
  public static final double K_V = 0.0; // temp
  public static final double K_A = 0.0;
  public static final double K_G = 0.0;

  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2700;
  public static final double MOTION_MAGIC_ACCLERATION = 16;

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
                  .withKP(K_P)
                  .withKI(K_I)
                  .withKD(K_D)
                  .withKS(K_S_STOWED)
                  .withKV(K_V)
                  .withKA(K_A)
                  .withKG(K_G)
                  .withGravityArmPositionOffset(GRAVITY_POSITION_OFFSET)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withSlot1(
              new Slot1Configs()
                  .withKP(K_P)
                  .withKI(K_I)
                  .withKD(K_D)
                  .withKS(K_S_OUT)
                  .withKV(K_V)
                  .withKA(K_A)
                  .withKG(K_G)
                  .withGravityArmPositionOffset(GRAVITY_POSITION_OFFSET)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(MOTION_MAGIC_ACCLERATION)
                  .withMotionMagicCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY));

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

  private final VoltageOut voltageRequest =
      new VoltageOut(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);
  // .withLimitForwardMotion(true)
  // .withLimitReverseMotion(true);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);
  // .withLimitForwardMotion(true)
  // .withLimitReverseMotion(true);

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
    var request = positionRequest.withPosition(position);
    if (this.position.getValueAsDouble() > 0.75) {
      request.withSlot(1);
    } else {
      request.withSlot(0);
    }
    motor.setControl(request);
  }
}
