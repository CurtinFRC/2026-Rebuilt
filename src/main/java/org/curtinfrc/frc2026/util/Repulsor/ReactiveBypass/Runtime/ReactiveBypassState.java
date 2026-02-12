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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

final class ReactiveBypassState {
  Pose2d latchedSubgoal = null;
  Translation2d latchedAtPosition = Translation2d.kZero;
  double timeSinceLatchS = 1e9;
  double timeSinceEvalS = 1e9;
  double timeSinceSideSwitchS = 1e9;
  int preferredSide = 0;
  double lastOcc = 0.0;
  Double lastChosenCost = null;
  double sideConfidence = 0.0;
  boolean pinnedMode = false;
  double pinnedTimeS = 0.0;
  double blockedAccumS = 0.0;
  double pinnedCooldownS = 0.0;
  Rotation2d pinnedHeading = Rotation2d.kZero;
  int consecutiveBypassFailures = 0;

  void reset() {
    latchedSubgoal = null;
    latchedAtPosition = Translation2d.kZero;
    timeSinceLatchS = 1e9;
    timeSinceEvalS = 1e9;
    timeSinceSideSwitchS = 1e9;
    preferredSide = 0;
    lastOcc = 0.0;
    lastChosenCost = null;
    sideConfidence = 0.0;
    pinnedMode = false;
    pinnedTimeS = 0.0;
    blockedAccumS = 0.0;
    pinnedCooldownS = 0.0;
    pinnedHeading = Rotation2d.kZero;
    consecutiveBypassFailures = 0;
  }
}
