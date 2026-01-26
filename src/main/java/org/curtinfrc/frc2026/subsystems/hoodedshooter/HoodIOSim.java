package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIODev {
  private static final double D_T = 0.001;
  private static final double HOOD_JKG = 0.00816;

  private final TalonFXSimState motorSim;
  private final CANcoderSimState encoderSim;
  private final DCMotor motorType = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  public HoodIOSim() {
    super();

    motorSim = motor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX44);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, HOOD_JKG, GEAR_RATIO), motorType);
    encoderSim = encoder.getSimState();

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(D_T);
  }

  public void updateSim() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(D_T);

    double motorRotations = motorSimModel.getAngularPositionRotations() * GEAR_RATIO;
    double motorRPS = motorSimModel.getAngularVelocityRPM() * GEAR_RATIO / 60;

    motorSim.setRawRotorPosition(motorRotations);
    motorSim.setRotorVelocity(motorRPS);

    encoderSim.setRawPosition(motorRotations / GEAR_RATIO);
    encoderSim.setVelocity(motorRPS / GEAR_RATIO);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
