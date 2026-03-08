package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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

public class ShooterIOComp implements ShooterIO {
  public static final int ID1 = 17; // BL Shooter
  public static final int ID2 = 18; // BR Shooter
  public static final int ID3 = 19; // FL Shooter
  public static final int ID4 = 20; // FR Shooter

  public static final double VELOCITY_TOLERANCE = 1;

  public static final double GEAR_RATIO = 1.0;
  private static final double KP = 0.307;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KS = 0.4;
  private static final double KV = 0.1256;
  // private static final double KV_SHOOTING = 0.2;
  private static final double KA = 0.047;

  // private final LoggedTunableNumber shooterKP =
  //     new LoggedTunableNumber("Shooter/KP", KP);
  // private final LoggedTunableNumber shooterKD =
  //     new LoggedTunableNumber("Shooter/KD", KD);
  // private final LoggedTunableNumber shooterKV =
  //     new LoggedTunableNumber("Shooter/KV", KV);
  // private final LoggedTunableNumber shooterKS =
  //     new LoggedTunableNumber("Shooter/KS", KS);
  // private final LoggedTunableNumber shooterKI =
  //     new LoggedTunableNumber("Shooter/KI", KI);

  protected final TalonFX leaderMotor = new TalonFX(ID1);
  protected final TalonFX followerMotor1 = new TalonFX(ID2);
  protected final TalonFX followerMotor2 = new TalonFX(ID3);
  protected final TalonFX followerMotor3 = new TalonFX(ID4);

  private final TalonFXConfiguration sharedMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(100).withStatorCurrentLimit(120))
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(GEAR_RATIO)
                  .withVelocityFilterTimeConstant(0.1))
          .withSlot0(
              new Slot0Configs().withKP(KP).withKI(KI).withKD(KD).withKS(KS).withKV(KV).withKA(KA))
          .withSlot1(
              new Slot1Configs().withKP(KP).withKI(KI).withKD(KD).withKS(KS).withKV(KV).withKA(KA));

  private final StatusSignal<Voltage> voltage = leaderMotor.getMotorVoltage();
  private final StatusSignal<Current> current = leaderMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> velocity = leaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> acceleration = leaderMotor.getAcceleration();
  private final StatusSignal<Angle> position = leaderMotor.getPosition();

  private final List<StatusSignal<Temperature>> motorTemperatures =
      List.of(
          leaderMotor.getDeviceTemp(),
          followerMotor1.getDeviceTemp(),
          followerMotor2.getDeviceTemp(),
          followerMotor3.getDeviceTemp());

  final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  public ShooterIOComp() {
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(sharedMotorConfig));
    tryUntilOk(5, () -> followerMotor1.getConfigurator().apply(sharedMotorConfig));
    tryUntilOk(5, () -> followerMotor2.getConfigurator().apply(sharedMotorConfig));
    tryUntilOk(5, () -> followerMotor3.getConfigurator().apply(sharedMotorConfig));

    followerMotor1.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));
    followerMotor2.setControl(new Follower(ID1, MotorAlignmentValue.Aligned));
    followerMotor3.setControl(new Follower(ID1, MotorAlignmentValue.Opposed));

    voltage.setUpdateFrequency(1000);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, acceleration, voltage, current);
    leaderMotor.optimizeBusUtilization();
    followerMotor1.optimizeBusUtilization();
    followerMotor2.optimizeBusUtilization();
    followerMotor3.optimizeBusUtilization();
    PhoenixUtil.registerSignals(false, velocity, acceleration, voltage, current);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.motorTemperatures = new double[4];
    inputs.motorsConnected = new boolean[4];
    for (int motor = 0; motor < 4; motor++) {
      inputs.motorTemperatures[motor] = motorTemperatures.get(motor).getValueAsDouble();
      inputs.motorsConnected[motor] = motorTemperatures.get(motor).getStatus().isOK();
    }

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
  public void setVelocity(double velocity, boolean shooting) {
    double rps = convertVelocityToRPS(velocity);
    var request = velocityRequest.withVelocity(rps);
    if (shooting) {
      request = request.withSlot(1);
    } else {
      request = request.withSlot(0);
    }
    leaderMotor.setControl(request);
  }

  public static double convertVelocityToRPS(double velocity) {
    return velocity / (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }

  public static double convertRPSToVelocity(double angularVelocityRotationsPerSecond) {
    return angularVelocityRotationsPerSecond * (HoodedShooter.WHEEL_DIAMETER * Math.PI);
  }
}
