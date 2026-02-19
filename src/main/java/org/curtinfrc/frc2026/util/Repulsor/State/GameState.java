/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */
package org.curtinfrc.frc2026.util.Repulsor.State;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class GameState extends StaticState {
  private final double TELEOP_GAME_LENGTH = 140.0;
  private final double AUTONOMOUS_PERIOD_LENGTH = 20.0;
  private final double TRANSITION_PERIOD_LENGTH = 10.0;
  private final double MATCH_SHIFT_LENGTH = 25.0;

  private Optional<DriverStation.Alliance> alliance = Optional.empty();
  private Optional<DriverStation.Alliance> inactiveFirst = Optional.empty();

  private Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  private void updateAlliance() {
    noAllianceAlert.set(false);
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    if (alliance.isEmpty() && readAlliance.isPresent()) {
      alliance = readAlliance;
      noAllianceAlert.set(false);
    } else {
      noAllianceAlert.set(true);
    }
  }

  private void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    noGameDataAlert.set(true);
    if (gameData.length() > 0) {
      if (gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R') {
        noGameDataAlert.set(false);
      }
    }

    if (inactiveFirst.isEmpty() && !noGameDataAlert.get()) {
      inactiveFirst =
          Optional.of(
              ("B".equals(gameData)) ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);
    }
  }

  private double getMatchTime() {
    double gameTime = DriverStation.getMatchTime();
    if (DriverStation.isFMSAttached()) {
      gameTime = TELEOP_GAME_LENGTH - gameTime;
    }
    return gameTime;
  }

  private int getGamePeriodNumber() {
    double gameTime = getMatchTime();

    int gamePeriodNumber;
    if (!DriverStation.isTeleopEnabled()) {
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

  public boolean isHubActive() {
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

  public boolean isHubInactive() {
    return !isHubActive();
  }

  public double getRemainingShiftTime() {
    double shiftEndTime;
    double gameTime = getMatchTime();
    double shiftNumber = getGamePeriodNumber();
    if (shiftNumber == -1) {
      shiftEndTime = AUTONOMOUS_PERIOD_LENGTH;
    } else if (shiftNumber == 0) {
      shiftEndTime = TRANSITION_PERIOD_LENGTH;
    } else {
      shiftEndTime = shiftNumber * MATCH_SHIFT_LENGTH + TRANSITION_PERIOD_LENGTH;
      shiftEndTime = (shiftNumber < 5) ? shiftEndTime : shiftEndTime + 5;
    }

    return shiftEndTime - gameTime;
  }

  @Override
  public void update(double dt) {
    updateGameData();
    updateAlliance();
  }
}
