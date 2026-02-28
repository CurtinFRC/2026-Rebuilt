package org.curtinfrc.frc2026.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GameState {
  public static final double TELEOP_GAME_LENGTH = 140.0;
  public static final double AUTONOMOUS_PERIOD_LENGTH = 20.0;
  public static final double TRANSITION_PERIOD_LENGTH = 10.0;
  public static final double MATCH_SHIFT_LENGTH = 25.0;

  public static Optional<DriverStation.Alliance> alliance = Optional.empty();
  public static Optional<DriverStation.Alliance> inactiveFirst = Optional.empty();

  private static Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private static Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  public static final Trigger activeShift = new Trigger(GameState::isHubActive);

  public static void updateAlliance() {
    noAllianceAlert.set(false);
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    if (alliance.isEmpty() && readAlliance.isPresent()) {
      alliance = readAlliance;
    } else {
      noAllianceAlert.set(true);
    }
  }

  public static void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    noGameDataAlert.set(true);
    if (gameData.length() > 0) {
      if (gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R') {
        noGameDataAlert.set(false);
      }
    }

    if (inactiveFirst.isEmpty() && !noGameDataAlert.get()) {
      inactiveFirst =
          Optional.of((gameData == "B") ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);
    }
  }

  public static double getMatchTime() {
    double gameTime = DriverStation.getMatchTime();
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.isTeleopEnabled()) {
        gameTime = TELEOP_GAME_LENGTH - gameTime;
      } else {
        gameTime = AUTONOMOUS_PERIOD_LENGTH - gameTime;
      }
    }
    return gameTime;
  }

  // Returns the number of game periods that has passed starting from auto as -1
  public static int getGamePeriodNumber() {
    double gameTime = getMatchTime();

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
    if (DriverStation.isAutonomous()) {
      return true;
    }

    if (!(inactiveFirst.isPresent() && alliance.isPresent())) {
      return false;
    }

    int shiftDiscriminant = (inactiveFirst.get() == alliance.get()) ? 1 : 0;
    int gamePeriodNumber = getGamePeriodNumber();
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

  public static double getRemainingShiftTime() {
    double shiftEndTime;
    double gameTime = getMatchTime();
    double shiftNumber = getGamePeriodNumber();
    if (shiftNumber == -1) {
      shiftEndTime = AUTONOMOUS_PERIOD_LENGTH;
    } else if (shiftNumber == 0) {
      shiftEndTime = TRANSITION_PERIOD_LENGTH;
    } else {
      shiftEndTime = shiftNumber * MATCH_SHIFT_LENGTH + TRANSITION_PERIOD_LENGTH;
      shiftEndTime = (shiftNumber < 5) ? shiftEndTime : shiftEndTime + 5; // checking for endgame
    }

    return shiftEndTime - gameTime;
  }

  public static void periodic() {
    updateGameData();
    updateAlliance();

    Logger.recordOutput("GameState/gameState", GameState.isHubActive());
    Logger.recordOutput("GameState/remainingShiftTime", GameState.getRemainingShiftTime());
  }
}
