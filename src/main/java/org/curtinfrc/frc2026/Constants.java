package org.curtinfrc.frc2026;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  // Mag port numbers for motors
  public static final int intakeMagRollerMotorID = 22;
  public static final int middleMagRollerMotorID = 20;
  public static final int indexerMagRollerMotorID = 15;

  public static final RobotType robotType = RobotType.DEV;

  public static final Mode getMode() {
    return switch (robotType) {
      case COMP, DEV -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM -> Mode.SIM;
    };
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY;
  }

  public static enum RobotType {
    COMP,
    DEV,
    SIM;
  }

  public static void main(String... args) {
    if (robotType == RobotType.SIM) {
      System.out.println("Error invalid robot type selected for deploy: SIM");
      System.exit(1);
    }
  }
}
