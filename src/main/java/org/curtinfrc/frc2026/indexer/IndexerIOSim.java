package org.curtinfrc.frc2026.indexer;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private static final int ID = 0;
  private static final double kGearRatio = 10.0;

  private final TalonFX motor = new TalonFX(ID);
  private final TalonFXSimState motorSim;

  private final DCMotorSim motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio),
          DCMotor.getKrakenX60Foc(1));

  public IndexerIOSim() {
    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = motorSim.getMotorVoltageMeasure();
    motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    motorSimModel.update(0.001);
    motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));

    motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));

    inputs.appliedVolts = motorVoltage.in(Volts);
    inputs.currentAmps = motorSim.getSupplyCurrent();
    inputs.positionRotations = motorSimModel.getAngularPosition().times(kGearRatio).in(Rotation);
    inputs.angularVelocityRotationsPerSecond =
        motorSimModel.getAngularVelocity().times(kGearRatio).in(RotationsPerSecond);
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
