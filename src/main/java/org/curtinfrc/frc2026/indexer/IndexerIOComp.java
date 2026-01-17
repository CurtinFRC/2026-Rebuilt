package org.curtinfrc.frc2026.indexer;

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

public class IndexerIOComp implements IndexerIO {
  private static final int ID = 0;

  private final TalonFX motor = new TalonFX(ID);

  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  public IndexerIOComp() {
    tryUntilOk(
        5,
        () ->
            motor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                        .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(30)
                                .withStatorCurrentLimit(60))));
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, voltage, current, position);
    motor.optimizeBusUtilization();

    var slot0Configs = new Slot0Configs().withKP(2.4).withKI(0.0).withKD(0.1);

    var pidConfig = new TalonFXConfiguration();
    pidConfig.Slot0 = slot0Configs;

    tryUntilOk(5, () -> motor.getConfigurator().apply(pidConfig));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.angularVelocityRotationsPerSecond = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.setControl(velocityRequest.withVelocity(speed));
  }
}
