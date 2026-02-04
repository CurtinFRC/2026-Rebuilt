package org.curtinfrc.frc2026.subsystems.hoodedshooter;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class ShooterIODev implements ShooterIO {
  public static final int ID1 = 18;
  public static final int ID2 = 19;
  public static final int ID3 = 29;

  public static final double GEAR_RATIO = 1.0;
  private static final double KP = 0.00076245;
  private static final double KI = 0.0;
  private static final double KD = 0.1;
  private static final double KS = 0.24152;
  private static final double KV = 0.12173;
  private static final double KA = 0.015427;

  protected final TalonFX leaderMotor = new TalonFX(ID1);
  protected final TalonFX followerMotor1 = new TalonFX(ID2);
  protected final TalonFX followerMotor2 = new TalonFX(ID3);

  private final TalonFXConfiguration sharedMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(100).withStatorCurrentLimit(120))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(
              new Slot0Configs().withKP(KP).withKI(KI).withKD(KD).withKS(KS).withKV(KV).withKA(KA));

  private final StatusSignal<Voltage> voltage = leaderMotor.getMotorVoltage();
  private final StatusSignal<Current> current = leaderMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> velocity = leaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> acceleration = leaderMotor.getAcceleration();
  private final StatusSignal<Angle> position = leaderMotor.getPosition();

  private final List<StatusSignal<Temperature>> motorTemperatures =
      List.of(
          leaderMotor.getDeviceTemp(),
          followerMotor1.getDeviceTemp(),
          followerMotor2.getDeviceTemp());

  final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  public ShooterIODev() {
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(sharedMotorConfig));
    tryUntilOk(5, () -> followerMotor1.getConfigurator().apply(sharedMotorConfig));
    tryUntilOk(5, () -> followerMotor2.getConfigurator().apply(sharedMotorConfig));

    followerMotor1.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));
    followerMotor2.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, acceleration, voltage, current);
    leaderMotor.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, acceleration, voltage, current);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.motorTemperatures = new double[3];
    inputs.motorsConnected = new boolean[3];
    for (int motor = 0; motor < 3; motor++) {
      inputs.motorTemperatures[motor] = motorTemperatures.get(motor).getValueAsDouble();
      inputs.motorsConnected[motor] = motorTemperatures.get(motor).getStatus().isOK();
    }

    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityMetresPerSecond = convertRPSToVelocity(velocity.getValueAsDouble());
    inputs.accelerationMetresPerSecondPerSecond =
        convertRPSToVelocity(acceleration.getValueAsDouble());
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVoltageV(Voltage voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(double velocity) {
    double rps = convertVelocityToRPS(velocity);
    leaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  public double convertVelocityToRPS(double velocity) {
    return velocity / (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }

  public double convertRPSToVelocity(double angularVelocityRotationsPerSecond) {
    return angularVelocityRotationsPerSecond * (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }
}
