package org.curtinfrc.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.curtinfrc.frc2026.subsystems.MagRoller.*;

public class Mag extends SubsystemBase {
  // private MagRoller magRoller1;
  // private MagRoller magRoller2;
  // private MagRoller Indexer;

  private MagRoller intakeMagRoller;
  private MagRoller middleMagRoller;
  private MagRoller indexerMagRoller;

  public Mag(MagRollerIO roller1, MagRollerIO roller2, MagRollerIO roller3) {
    intakeMagRoller = new MagRoller(roller1);
    middleMagRoller = new MagRoller(roller2);
    indexerMagRoller = new MagRoller(roller3);
    // this.magRoller1 = roller1;
    // this.magRoller2 = roller2;
    // this.Indexer = roller3;
  }

  public Command spinIndexer(double volts) {
    return this.indexerMagRoller.runMotor(volts);
  }

  public Command moveAll(double volts) {
    return run(
        () -> {
          this.intakeMagRoller.runMotor(volts);
          this.middleMagRoller.runMotor(volts);
          this.indexerMagRoller.runMotor(volts);
        });
  }

  public Command store(double volts) {
    return run(
        () -> {
          this.intakeMagRoller.runMotor(volts);
          this.middleMagRoller.runMotor(volts);
        });
  }

  public Command stop() {
    return run(
        () -> {
          this.intakeMagRoller.stop();
          this.middleMagRoller.stop();
          this.indexerMagRoller.stop();
        });
  }
}
