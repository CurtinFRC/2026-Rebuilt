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

package org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.Queue;

public class ExtraPathingBounceListener {
  private final double bounceDistanceThreshold;
  private final int bounceHistoryLimit;
  private final Queue<Pose2d> recentGoals = new LinkedList<>();
  private boolean isBouncing;

  public ExtraPathingBounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
    this.bounceDistanceThreshold = bounceDistanceThreshold;
    this.bounceHistoryLimit = bounceHistoryLimit;
  }

  public void update(Pose2d currentGoal) {
    recentGoals.add(currentGoal);
    if (recentGoals.size() > bounceHistoryLimit) {
      recentGoals.poll();
    }

    isBouncing = checkBouncing();
  }

  private boolean checkBouncing() {
    if (recentGoals.size() < bounceHistoryLimit) return false;

    int similarCount = 0;
    Pose2d[] goals = recentGoals.toArray(new Pose2d[0]);
    for (int i = 0; i < goals.length - 1; i++) {
      for (int j = i + 1; j < goals.length; j++) {
        if (goals[i].getTranslation().getDistance(goals[j].getTranslation())
            < bounceDistanceThreshold) {
          similarCount++;
        }
      }
    }

    int totalPairs = (goals.length * (goals.length - 1)) / 2;
    return similarCount >= (totalPairs * 0.6);
  }

  public void clearHistory() {
    recentGoals.clear();
    isBouncing = false;
  }

  public boolean isBouncing() {
    return isBouncing;
  }
}
