package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.ArmSim;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.HardStopConfig;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.MechanismSimParams;

public class HoodIOSim extends HoodIOComp {
  private static final double DT = 0.001;
  private static final double HOOD_JKG = 0.00816;
  private static final double HOOD_MIN_POSITION_ROTATIONS = 0;
  private static final double HOOD_MAX_POSITION_ROTATIONS = 1.5;
  private static final double HOOD_CG_RADIUS_METERS = 0.220259;

  private final TalonFXSimState motorSim;
  private final CANcoderSimState encoderSim;
  private final DCMotor motorType = DCMotor.getKrakenX44Foc(1);
  private final ArmSim motorSimModel;
  private final Notifier simNotifier;

  public HoodIOSim() {
    super();

    motorSim = leaderMotor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX44);
    motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    double hoodMassKg = HOOD_JKG / (HOOD_CG_RADIUS_METERS * HOOD_CG_RADIUS_METERS);
    MechanismSimParams params =
        MechanismSimParams.rotary(motorType, GEAR_RATIO, HOOD_JKG)
            .withHardStopConfig(
                new HardStopConfig(
                    Units.rotationsToRadians(HOOD_MIN_POSITION_ROTATIONS),
                    Units.rotationsToRadians(HOOD_MAX_POSITION_ROTATIONS),
                    0.0,
                    0.0,
                    0.0,
                    1.0))
            .build();
    motorSimModel =
        new ArmSim(
            params,
            hoodMassKg,
            HOOD_CG_RADIUS_METERS,
            9.80665,
            Units.rotationsToRadians(GRAVITY_POSITION_OFFSET));

    encoderSim = encoder.getSimState();
    encoderSim.Orientation = ChassisReference.Clockwise_Positive;

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(DT);
  }

  public void updateSim() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVolts = motorSim.getMotorVoltageMeasure().in(Volts);
    motorSimModel.setInputVoltage(motorVolts);
    motorSimModel.update(DT);

    double motorRotations =
        Units.radiansToRotations(motorSimModel.getPositionRad()) * GEAR_RATIO
            - GRAVITY_POSITION_OFFSET;
    double motorRPS = Units.radiansToRotations(motorSimModel.getVelocityRadPerSec()) * GEAR_RATIO;

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
