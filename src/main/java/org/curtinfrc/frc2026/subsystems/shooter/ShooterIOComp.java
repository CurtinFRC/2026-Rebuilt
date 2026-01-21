package org.curtinfrc.frc2026.subsystems.shooter;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
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

public class ShooterIOComp implements ShooterIO {
  private static final int ID = 99;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);
  private static final TalonFXConfiguration leaderConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(new Slot0Configs().withKP(0.5).withKV(0.33))
          .withCurrentLimits(currentLimits);

  private final TalonFX motor = new TalonFX(ID);

  //   private final DigitalInput frontSensor = new DigitalInput(4);
  //   private final DigitalInput backSensor = new DigitalInput(2);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  public ShooterIOComp() {
    tryUntilOk(5, () -> motor.getConfigurator().apply(leaderConfig));
    BaseStatusSignal.setUpdateFrequencyForAll(20, velocity, voltage, current, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, current, position);

    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.angularVelocityRotationsPerMinute = position.getValueAsDouble();
    // inputs.frontSensor = frontSensor.get();
    // inputs.backSensor = backSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  public void setVelocity(double speedRotationsPerMinute) {
    motor.setControl(velocityRequest.withVelocity(speedRotationsPerMinute));
  }
}
