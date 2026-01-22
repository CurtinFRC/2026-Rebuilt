package org.curtinfrc.frc2026.subsystems.shooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

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

public class ShooterIOComp implements ShooterIO {
  public int ID1 = 99;
  public int ID2 = 99;
  public int ID3 = 99;
  public int ID4 = 99;

  public double kGearRatio = 1.0; // unsure

  protected final TalonFX motor1 = new TalonFX(ID1);
  protected final TalonFX motor2 = new TalonFX(ID2);
  protected final TalonFX motor3 = new TalonFX(ID3);
  protected final TalonFX motor4 = new TalonFX(ID4);

  private final TalonFXConfiguration sharedMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio))
          .withSlot0(new Slot0Configs().withKP(2.4).withKI(0.0).withKD(0.1));

  private final TalonFXConfiguration leftMotorConfig =
      sharedMotorConfig.withMotorOutput(
          sharedMotorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

  private final TalonFXConfiguration rightMotorConfig =
      sharedMotorConfig.withMotorOutput(
          sharedMotorConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));

  private final StatusSignal<Voltage> voltage = motor1.getMotorVoltage();
  private final StatusSignal<Current> current = motor1.getStatorCurrent();
  private final StatusSignal<AngularVelocity> velocity = motor1.getVelocity();
  private final StatusSignal<AngularAcceleration> acceleration = motor1.getAcceleration();
  private final boolean atTargetSpeed = false; // for later implementation

  final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  public ShooterIOComp() {
    tryUntilOk(5, () -> motor1.getConfigurator().apply(leftMotorConfig));
    tryUntilOk(5, () -> motor2.getConfigurator().apply(leftMotorConfig));
    tryUntilOk(5, () -> motor3.getConfigurator().apply(rightMotorConfig));
    tryUntilOk(5, () -> motor4.getConfigurator().apply(rightMotorConfig));

    motor2.setControl(new Follower(ID1, MotorAlignmentValue.Aligned));
    motor3.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));
    motor4.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(20, velocity, acceleration, voltage, current);
    motor1.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocityRotationsPerSecond = velocity.getValueAsDouble();
    inputs.accelerationRotationsPerSecondPerSecond = acceleration.getValueAsDouble();
    inputs.atTargetSpeed = atTargetSpeed;
  }

  @Override
  public void setVoltage(double volts) {
    motor1.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setSpeed(double angularVelocityRotationsPerSecond) {
    motor1.setControl(velocityRequest.withVelocity(angularVelocityRotationsPerSecond));
  }
}
