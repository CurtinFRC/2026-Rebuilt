package org.curtinfrc.frc2026.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GameState extends VirtualSubsystem {
  public static final double TELEOP_GAME_LENGTH = 140.0;
  public static final double TRANSITION_PERIOD_LENGTH = 10.0;
  public static final double MATCH_SHIFT_LENGTH = 25.0;

  public static DriverStation.Alliance alliance;
  public static DriverStation.Alliance inactiveFirst; // alliance that won auto

  private static Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private static Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  public static void updateAlliance() {
    noAllianceAlert.set(false);
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    if (alliance == null && readAlliance.isPresent()) {
      alliance = readAlliance.get();
    } else {
      noAllianceAlert.set(true);
    }
  }

  public static void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() == 0 && !(gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R')) {
      noGameDataAlert.set(true);
    }
    noGameDataAlert.set(false);

    if (inactiveFirst == null) {
      inactiveFirst = (gameData == "B") ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red;
    }
  }

  // Returns the number of game periods that has passed starting from auto as -1
  public static int getGamePeriodNumber() {
    double gameTime = DriverStation.getMatchTime();
    if (DriverStation.isFMSAttached()) {
      gameTime = TELEOP_GAME_LENGTH - gameTime;
    }

    int gamePeriodNumber;
    if (!DriverStation.isTeleopEnabled()) { // auto
      gamePeriodNumber = -1;
    } else {
      if (gameTime <= 10) {
        gamePeriodNumber = 0;
      } else {
        gamePeriodNumber =
            (int) Math.ceil((gameTime - TRANSITION_PERIOD_LENGTH) / MATCH_SHIFT_LENGTH);
      }
    }

    return gamePeriodNumber;
  }

  public static boolean isHubActive() {
    int shiftDiscriminant = (inactiveFirst == alliance) ? 1 : 0;

    int gamePeriodNumber = getGamePeriodNumber();
    Logger.recordOutput("periodNumber", gamePeriodNumber);
    Logger.recordOutput("shiftDiscriminant", shiftDiscriminant);
    boolean isActive = false;
    if (gamePeriodNumber == 0) {
      isActive = true;
    } else {
      isActive = !(gamePeriodNumber % 2 == shiftDiscriminant);
    }
    return isActive;
  }

  public static boolean isHubInactive() {
    return !isHubActive();
  }

  @Override
  public void periodic() {
    updateGameData();
    updateAlliance();
  }
}
