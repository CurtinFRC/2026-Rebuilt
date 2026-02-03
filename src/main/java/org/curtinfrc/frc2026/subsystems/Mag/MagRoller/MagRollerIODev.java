package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class MagRollerIODev implements MagRollerIO {

  protected final TalonFX magMotor;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<AngularVelocity> angularVelocity;
  private final StatusSignal<Angle> angle;
  private final StatusSignal<Current> current;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  final PositionVoltage Indexer_PID = new PositionVoltage(0).withSlot(0);
  final VelocityVoltage Store_Vel_PID = new VelocityVoltage(0).withSlot(1);
  private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  final VelocityVoltage Motion_Magic_Store_Vel_PID = new VelocityVoltage(0);

  // indexer position PID variables
  private static final double POS_KP = 1.0;
  private static final double POS_KI = 0;
  private static final double POS_KD = 0;

  // velocity PID for all magazine rollers
  private static final double VEL_KS = 0;
  private static final double VEL_KV = 0;
  private static final double VEL_KP = 0.5;
  private static final double VEL_KI = 0;
  private static final double VEL_KD = 0;

  // Motion Magic PID for all magazine rollers
  private static final double MAGIC_VEL_KS = 0;
  private static final double MAGIC_VEL_KV = 0;
  private static final double MAGIC_VEL_KP = .4;
  private static final double MAGIC_VEL_KI = 0.24;
  private static final double MAGIC_VEL_KD = 0;
  private static final double MAGIC_VEL_ACCEL = 4;
  private static final double MAGIC_VEL_JERK = 40;

  public MagRollerIODev(int motorID, InvertedValue inverted) {

    magMotor = new TalonFX(motorID);
    voltage = magMotor.getMotorVoltage();
    angularVelocity = magMotor.getVelocity();
    angle = magMotor.getPosition();
    current = magMotor.getStatorCurrent();

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        MAGIC_VEL_ACCEL; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk =
        MAGIC_VEL_JERK; // Target jerk of 4000 rps/s/s (0.1 seconds)

    // magMotor.getConfigurator().apply(talonFXConfigs);

    tryUntilOk(
        5,
        () ->
            magMotor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs().withInverted(inverted))
                        .withCurrentLimits(currentLimits)
                        .withSlot0(new Slot0Configs().withKP(POS_KP).withKI(POS_KI).withKD(POS_KD))
                        .withSlot1(
                            new Slot1Configs()
                                .withKP(VEL_KP)
                                .withKI(VEL_KI)
                                .withKD(VEL_KD)
                                .withKS(VEL_KS)
                                .withKV(VEL_KV))
                        .withSlot2(
                            new Slot2Configs()
                                .withKP(MAGIC_VEL_KP)
                                .withKI(MAGIC_VEL_KI)
                                .withKD(MAGIC_VEL_KD)
                                .withKS(MAGIC_VEL_KS)
                                .withKV(MAGIC_VEL_KV))
                        .withMotionMagic(motionMagicConfigs)));

    // Setting update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, voltage, current, angle, angularVelocity);

    // Setting update frequency (which is slowed down) for variables which do not have an update
    // frequency
    magMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, voltage, current, angularVelocity, angle);
  }

  @Override
  public void updateInputs(MagRollerIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = angle.getValueAsDouble();
    inputs.angularVelocityRotationsPerMinute = angularVelocity.getValueAsDouble();
    inputs.setPoint = 67;
  }

  @Override
  public void setVoltage(double volts) {
    magMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    magMotor.setControl(Indexer_PID.withPosition(position));
  }

  @Override
  public double getPosition() {
    return angle.getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double targetVelocityRPS) {
    magMotor.setControl(
        Motion_Magic_Store_Vel_PID.withVelocity(targetVelocityRPS)
            .withSlot(2)
            .withFeedForward(7.7)); //
  }
}
