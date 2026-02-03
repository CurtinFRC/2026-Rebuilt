package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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

  // indexer position PID variables
  private static final double POS_KP = 1.0;
  private static final double POS_KI = 0;
  private static final double POS_KD = 0;

  private static final double VEL_KS = 0;
  private static final double VEL_KV = 0;
  private static final double VEL_KP = .4;
  private static final double VEL_KI = 0.27;
  private static final double VEL_KD = 0;

  public MagRollerIODev(int motorID, InvertedValue inverted) {

    magMotor = new TalonFX(motorID);
    voltage = magMotor.getMotorVoltage();
    angularVelocity = magMotor.getVelocity();
    angle = magMotor.getPosition();
    current = magMotor.getStatorCurrent();

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
                                .withKV(VEL_KV))));

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
        Store_Vel_PID.withVelocity(targetVelocityRPS).withSlot(1).withFeedForward(8.1)); //
  }
}
