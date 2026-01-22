package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim extends ShooterIODev {
  private static final double SIMULATION_DELTA_TIME = 0.001;
  private static final double SHOOTER_J = 0.0035;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(4);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public ShooterIOSim() {
    super();

    motorSim = leaderMotor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, SHOOTER_J, GEAR_RATIO), motorType);

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(SIMULATION_DELTA_TIME);
  }

  public void updateSim() {

    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(SIMULATION_DELTA_TIME);

    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().in(Rotations));
    motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().in(RotationsPerSecond));
  }
}
