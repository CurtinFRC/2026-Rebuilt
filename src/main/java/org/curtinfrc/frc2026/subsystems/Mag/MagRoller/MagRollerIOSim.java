package org.curtinfrc.frc2026.subsystems.Mag.MagRoller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MagRollerIOSim extends MagRollerIODev {
  private static final double DT = 0.001;
  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public MagRollerIOSim(double GEAR_RATIO) {

    super(0, InvertedValue.CounterClockwise_Positive);

    motorSim = magMotor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, 0.0001, GEAR_RATIO), motorType);
    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(DT);
  }

  public void updateSim() {
    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(DT);

    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
    motorSim.setRotorVelocity(motorSimModel.getAngularVelocityRPM());
  }
}

// Gear ratios: intake roller 3:1, mid roller 9.6:1, 3:1
