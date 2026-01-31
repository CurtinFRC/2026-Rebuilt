package org.curtinfrc.frc2026.subsystems.Mag;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.curtinfrc.frc2026.subsystems.Mag.MagRoller.*;

public class Mag {

  private MagRoller intakeMagRoller;
  private MagRoller middleMagRoller;
  private MagRoller indexerMagRoller;

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Mag(MagRollerIO roller1, MagRollerIO roller2, MagRollerIO roller3) {
    intakeMagRoller = new MagRoller(roller1, "intakeMagRoller");
    middleMagRoller = new MagRoller(roller2, "midMagRoller");
    indexerMagRoller = new MagRoller(roller3, "indexerMagRoller");
  }

  public Command spinIndexer(double volts) {
    return this.indexerMagRoller.runMotor(volts);
  }

  public Command moveAll(double volts) {
    return Commands.parallel(
        indexerMagRoller.runMotor(volts),
        middleMagRoller.runMotor(volts),
        intakeMagRoller.runMotor(volts));
  }

  public Command store(double volts) {
    return Commands.parallel(middleMagRoller.runMotor(volts), intakeMagRoller.runMotor(volts));
  }

  public Command stop() {
    return Commands.parallel(
        intakeMagRoller.stop(), middleMagRoller.stop(), indexerMagRoller.stop());
  }

  public Command holdIndexerCommand() {
    return indexerMagRoller.stayAtCurrentPosition();
  }

  public Command runAtVelocity_RPS_PID(double velocityRPS) {
    return Commands.parallel(
        intakeMagRoller.runAtVelocityPID(velocityRPS),
        middleMagRoller.runAtVelocityPID(velocityRPS),
        indexerMagRoller.runAtVelocityPID(velocityRPS));
  }
}
