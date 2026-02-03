package org.curtinfrc.frc2026.subsystems.Intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class IntakeIODev implements IntakeIO {
  private final TalonFX motor = new TalonFX(46);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);

  final VelocityVoltage Intake_Vel_PID = new VelocityVoltage(0).withSlot(2);

  private static final double VEL_KS = 0;
  private static final double VEL_KV = 0;
  private static final double VEL_KP = .4;
  private static final double VEL_KI = 0.27;
  private static final double VEL_KD = 0;

  public IntakeIODev() {

    tryUntilOk(
        5,
        () ->
            motor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                        .withCurrentLimits(currentLimits)
                        .withSlot2(
                            new Slot2Configs()
                                .withKP(VEL_KP)
                                .withKI(VEL_KI)
                                .withKD(VEL_KD)
                                .withKS(VEL_KS)
                                .withKV(VEL_KV))));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.AppliedVoltage = voltage.getValueAsDouble();
    inputs.CurrentAmps = current.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
    inputs.setPoint = 15;
  }

  @Override
  public void setVoltage(double Volts) {
    // voltageRequest.withOutput(Volts);
    motor.set(Volts);
    System.out.println("running");
  }

  @Override
  public void setVelocityRPS(double targetVelocityRPS) {
    motor.setControl(
        Intake_Vel_PID.withVelocity(targetVelocityRPS).withSlot(2).withFeedForward(8.1));
  }
}
