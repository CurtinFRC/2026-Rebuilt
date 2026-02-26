package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.sim.BallSim;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim extends ShooterIODev {
  private static final double DT = 0.001;
  private static final double SHOOTER_JKG = 0.0035;

  private final TalonFXSimState motorSim;
  private final DCMotor motorType = DCMotor.getKrakenX60Foc(3);
  private final DCMotorSim motorSimModel;
  private final Notifier simNotifier;

  private final List<BallSim> simBalls = new ArrayList<>();

  public ShooterIOSim() {
    super();

    motorSim = leaderMotor.getSimState();
    motorSim.setMotorType(MotorType.KrakenX60);
    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorType, SHOOTER_JKG, GEAR_RATIO), motorType);

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

    for (int ball = 0; ball < simBalls.size(); ball++) {
      Pose3d ballPose = simBalls.get(ball).update(0.02);
      Logger.recordOutput("Balls/Ball " + String.valueOf(ball), ballPose);
    }
  }

  @Override
  public void addSimBall(BallSim ball) {
    simBalls.add(ball);
  }
}
