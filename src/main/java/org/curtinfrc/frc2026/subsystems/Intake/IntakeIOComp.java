package org.curtinfrc.frc2026.subsystems.Intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOComp implements IntakeIO {
  private final TalonFX frontMotor = new TalonFX(12);
  private final TalonFX backMotor = new TalonFX(13);

  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> frontMotorVoltage = frontMotor.getMotorVoltage();
  private final StatusSignal<Current> frontMotorCurrent = frontMotor.getStatorCurrent();
  private final StatusSignal<Angle> frontMotorPosition = frontMotor.getPosition();
  private final StatusSignal<AngularVelocity> frontMotorVelocity = frontMotor.getVelocity();

  private final StatusSignal<Voltage> backMotorVoltage = backMotor.getMotorVoltage();
  private final StatusSignal<Current> backMotorCurrent = backMotor.getStatorCurrent();
  private final StatusSignal<Angle> backMotorPosition = backMotor.getPosition();
  private final StatusSignal<AngularVelocity> backMotorVelocity = backMotor.getVelocity();

  private static final TalonFXConfiguration frontMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));
  private static final TalonFXConfiguration backMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60));

  public IntakeIOComp() {

    var slot0Configs = new Slot0Configs();

    slot0Configs.kP = 0.01;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    tryUntilOk(5, () -> frontMotor.getConfigurator().apply(frontMotorConfig));
    tryUntilOk(5, () -> backMotor.getConfigurator().apply(backMotorConfig));

    frontMotor.getConfigurator().apply(slot0Configs);
    backMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.frontMotorAppliedVoltage = frontMotorVoltage.getValueAsDouble();
    inputs.frontMotorCurrentAmps = frontMotorCurrent.getValueAsDouble();
    inputs.frontMotorAngularVelocity = frontMotorVelocity.getValueAsDouble();
    inputs.frontMotorPosition = frontMotorPosition.getValueAsDouble();

    inputs.backMotorAppliedVoltage = backMotorVoltage.getValueAsDouble();
    inputs.backMotorCurrentAmps = backMotorCurrent.getValueAsDouble();
    inputs.backMotorAngularVelocity = backMotorVelocity.getValueAsDouble();
    inputs.backMotorPosition = backMotorPosition.getValueAsDouble();
  }

  @Override
  public void setVelocity(double Velocity) {
    frontMotor.setControl(m_request.withVelocity(Velocity));
    backMotor.setControl(m_request.withVelocity(Velocity));
  }

  @Override
  public void setVoltage(double Volts) {
    frontMotor.set(Volts);
    backMotor.set(Volts);
    System.out.println("running");
  }
}
