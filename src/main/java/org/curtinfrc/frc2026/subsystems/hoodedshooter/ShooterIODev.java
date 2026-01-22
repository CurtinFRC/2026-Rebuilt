package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import org.curtinfrc.frc2026.util.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIODev implements ShooterIO {
  public static final int ID1 = 10;
  public static final int ID2 = 11;
  public static final int ID3 = 12;
  public static final int ID4 = 13;

  public static final double GEAR_RATIO = 1.0;
  private static final double EFFICIENCY = 0.85;
  private static final double KP = 0.05;
  private static final double KI = 0.001;
  private static final double KD = 0.003;
  private static final double KS = 0.24152;
  private static final double KV = 0.12173;
  private static final double KA = 0.015427;

  protected final TalonFX leaderMotor = new TalonFX(ID1);
  protected final TalonFX followerMotor1 = new TalonFX(ID2);
  protected final TalonFX followerMotor2 = new TalonFX(ID3);
  protected final TalonFX followerMotor3 = new TalonFX(ID4);

  private final TalonFXConfiguration sharedMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(
              new Slot0Configs().withKP(K_P).withKI(K_I).withKD(K_D).withKS(K_S).withKV(K_V));

  private final TalonFXConfiguration leftMotorConfig =
      sharedMotorConfig.withMotorOutput(
          sharedMotorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

  private final TalonFXConfiguration rightMotorConfig =
      sharedMotorConfig.withMotorOutput(
          sharedMotorConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));

  private final StatusSignal<Voltage> voltage = leaderMotor.getMotorVoltage();
  private final StatusSignal<Current> current = leaderMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> velocity = leaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> acceleration = leaderMotor.getAcceleration();

  final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  public ShooterIODev() {
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leftMotorConfig));
    tryUntilOk(5, () -> followerMotor1.getConfigurator().apply(leftMotorConfig));
    tryUntilOk(5, () -> followerMotor2.getConfigurator().apply(rightMotorConfig));
    tryUntilOk(5, () -> followerMotor3.getConfigurator().apply(rightMotorConfig));

    followerMotor1.setControl(new Follower(ID1, MotorAlignmentValue.Aligned));
    followerMotor2.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));
    followerMotor3.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, acceleration, voltage, current);
    leaderMotor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, acceleration, voltage, current);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.velocityMetresPerSecond = convertRPSToVelocity(velocity.getValueAsDouble());
    inputs.positionRotations = position.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(double velocity) {
    double rps = convertVelocityToRPS(velocity / EFFICIENCY);
    leaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void setVoltageV(Voltage voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public static double convertVelocityToRPS(double velocity) {
    return velocity / (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }

  public static double convertRPSToVelocity(double angularVelocityRotationsPerSecond) {
    return angularVelocityRotationsPerSecond * (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }
}
