package org.curtinfrc.frc2026.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim extends IndexerIOComp {
  private static final double kSimDt = 0.001;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public IndexerIOSim() {
    super();

    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    motorSimModel =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, 0.001, kGearRatio), motorType);

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(kSimDt);
  }

  public void updateSim() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(kSimDt);

    motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));
    motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    super.updateInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.setControl(new VelocityVoltage(speed));
  }
}
