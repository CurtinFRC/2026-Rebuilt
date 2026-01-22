package org.curtinfrc.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim extends ShooterIOComp {
  private static final double kSimDt = 0.001;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public ShooterIOSim() {
    super();

    motorSim = motor1.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);

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

    motorSim.setRawRotorPosition(motorSimModel.getAngularPosition());
    motorSim.setRotorVelocity(motorSimModel.getAngularVelocity());
  }
}
