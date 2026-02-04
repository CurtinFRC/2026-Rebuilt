package org.curtinfrc.frc2026.subsystems.Mag;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.curtinfrc.frc2026.subsystems.Mag.MagRoller.*;

public class Mag extends SubsystemBase {

  private MagRoller intakeMagRoller;
  private MagRoller middleMagRoller;
  private MagRoller indexerMagRoller;

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Mag(MagRollerIO roller1, MagRollerIO roller2, MagRollerIO roller3) {
    intakeMagRoller = new MagRoller(roller1);
    middleMagRoller = new MagRoller(roller2);
    indexerMagRoller = new MagRoller(roller3);
  }

  public void initDefaultCommand() {
    setDefaultCommand(stop());
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
}
