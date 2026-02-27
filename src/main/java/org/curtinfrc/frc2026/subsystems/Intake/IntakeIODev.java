package org.curtinfrc.frc2026.subsystems.Intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private static final TalonFXConfiguration config =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));

  public IntakeIODev() {

    var slot0Configs = new Slot0Configs();

    slot0Configs.kD = 0;
    slot0Configs.kI = 0;
    slot0Configs.kP = 0.01;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    motor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.frontMotorAppliedVoltage = voltage.getValueAsDouble();
    inputs.frontMotorCurrentAmps = current.getValueAsDouble();
    inputs.frontMotorAngularVelocity = velocity.getValueAsDouble();
  }

  @Override
  public void setVelocity(double Velocity) {
    m_request.withVelocity(Velocity);
  }

  @Override
  public void setVoltage(double Volts) {
    // voltageRequest.withOutput(Volts);
    motor.set(Volts);
    System.out.println("running");
  }
}
