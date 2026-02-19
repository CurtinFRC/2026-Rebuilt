package org.curtinfrc.frc2026.util.Repulsor.State;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public final class SimMatchDriver {
  private static double matchTimeSec = 0.0;
  private static boolean runAuto = false;

  public static void simInit(boolean runAuto) {
    SimMatchDriver.runAuto = runAuto;
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setGameSpecificMessage("R");

    DriverStationSim.setAutonomous(runAuto);
    // DriverStationSim.setEnabled(true);

    matchTimeSec = 0.0;
    DriverStationSim.setMatchTime(matchTimeSec);

    DriverStationSim.notifyNewData();
  }

  public static void simPeriodic(double dt) {
    if (DriverStationSim.getEnabled() == false) {
      return;
    }

    matchTimeSec += dt;
    DriverStationSim.setMatchTime(matchTimeSec);

    if (matchTimeSec >= 20.0 && runAuto) {
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
    }

    DriverStationSim.notifyNewData();
  }
}
