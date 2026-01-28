package org.curtinfrc.frc2026.subsystems.MagRoller;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.curtinfrc.frc2026.util.PhoenixUtil;

public class MagRollerIODev implements MagRollerIO {

  private final TalonFX magMotor;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<AngularVelocity> angularVelocity;
  private final StatusSignal<Angle> angle;
  private final StatusSignal<Current> current;
  private static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public MagRollerIODev(int motorID, InvertedValue inverted) {
    magMotor = new TalonFX(motorID);
    voltage = magMotor.getMotorVoltage();
    angularVelocity = magMotor.getVelocity();
    angle = magMotor.getPosition();
    current = magMotor.getStatorCurrent();

    tryUntilOk(
        5,
        () ->
            magMotor
                .getConfigurator()
                .apply(
                    new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs().withInverted(inverted))
                        .withCurrentLimits(currentLimits)));

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
  }

  @Override
  public void setVoltage(double volts) {
    magMotor.set(volts);
  }
}
