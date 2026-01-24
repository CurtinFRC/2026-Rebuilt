package org.curtinfrc.frc2026.subsystems;

import org.curtinfrc.frc2026.subsystems.MagRoller.MagRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mag extends SubsystemBase {
    private MagRoller magRoller1;
    private MagRoller magRoller2;
    private MagRoller Indexer;
    

    public Mag (MagRoller roller1, MagRoller roller2, MagRoller roller3) {
        this.magRoller1 = roller1;
        this.magRoller2 = roller2;
        this.Indexer = roller3;
    }

   public Command spinIndexer(double volts) {
    return this.Indexer.runMotor(volts).withTimeout(3);
  }
}

