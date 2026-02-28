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
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class HoodIOComp implements HoodIO {
  public static final int LEADER_ID = 14;
  public static final int FOLLOWER_ID = 15;
  public static final int ENCODER_ID = 16;

  public static final double POSITION_TOLERANCE_DEGREES = 0.5;

  public static final double ENCODER_TO_MECHANISM_RATIO = 8.33;
  public static final double MOTOR_TO_SENSOR_RATIO = 3;
  public static final double FORWARD_LIMIT_ROTATIONS = 0.057;
  public static final double REVERSE_LIMIT_ROTATIONS = -0.138;
  public static final double ENCODER_MAGNET_OFFSET = 0.538;

  public static final double GRAVITY_POSITION_OFFSET = 0.0888;
  public static final double KP = 181.35;
  public static final double KI = 0;
  public static final double KD = 3.5;
  public static final double KS = 0.305;
  public static final double KV = 2.44;
  public static final double KA = 0.02;
  public static final double KG = 0.36;

  public static final double MM_CRUISE_VELOCITY = 290;
  public static final double MM_ACCLERATION =
      2
          * DCMotor.getKrakenX44Foc(1).KtNMPerAmp
          * ENCODER_TO_MECHANISM_RATIO
          * MOTOR_TO_SENSOR_RATIO
          * 60
          / 2.5;

  protected final TalonFX leaderMotor = new TalonFX(LEADER_ID);
  protected final TalonFX followerMotor = new TalonFX(FOLLOWER_ID);
  private final TalonFXConfiguration sharedConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(ENCODER_ID) // Ties encoder with motor
                  .withFeedbackSensorSource(
                      FeedbackSensorSourceValue.RotorSensor) // Ties encoder with motor
                  .withSensorToMechanismRatio(ENCODER_TO_MECHANISM_RATIO * MOTOR_TO_SENSOR_RATIO))
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
                  .withKS(KS)
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
                  .withMagnetOffset(ENCODER_MAGNET_OFFSET)
                  .withAbsoluteSensorDiscontinuityPoint(1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

  private final StatusSignal<Angle> position = leaderMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = leaderMotor.getVelocity();
  private final StatusSignal<Current> current = leaderMotor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = leaderMotor.getMotorVoltage();
  private final StatusSignal<Angle> encoderPosition = encoder.getPosition();

  private final List<StatusSignal<Temperature>> motorTemperatures =
      List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

  private final VoltageOut voltageRequest =
      new VoltageOut(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true).withIgnoreSoftwareLimits(false);

  public HoodIOComp() {
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(sharedConfig));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(sharedConfig));
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

    followerMotor.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, voltage, current, position, encoderPosition);
    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, voltage, current, position, encoderPosition);

    leaderMotor.setPosition(
        encoder.getPosition().getValueAsDouble() / ENCODER_TO_MECHANISM_RATIO - 0.138);
    // 0.0517, 0.3608, -0.3091
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorTemperatures = new double[2];
    inputs.motorsConnected = new boolean[2];
    for (int motor = 0; motor < 2; motor++) {
      inputs.motorTemperatures[motor] = motorTemperatures.get(motor).getValueAsDouble();
      inputs.motorsConnected[motor] = motorTemperatures.get(motor).getStatus().isOK();
    }
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.encoderPositionRotations = encoderPosition.getValueAsDouble();
    inputs.angularVelocityRotationsPerSecond = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    leaderMotor.setControl(positionRequest.withPosition(position));
  }
}
