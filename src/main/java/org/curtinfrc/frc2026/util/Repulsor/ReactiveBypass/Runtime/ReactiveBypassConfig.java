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

import org.curtinfrc.frc2026.util.Repulsor.Constants;

public class ReactiveBypassConfig {
  public double fieldLen = Constants.FIELD_LENGTH;
  public double fieldWid = Constants.FIELD_WIDTH;
  public double inflationMeters = 0.10;
  public double triggerAheadMeters = 1.2;
  public double triggerWidthMeters = 0.7;
  public int corridorSamples = 15;
  public double occHigh = 0.2079111216823982;
  public double occLow = 0.08402471544016843;
  public double lateralMeters = 0.50;
  public double lateralMaxMeters = 1.90;
  public double minLateralMeters = 0.55;
  public double forwardMeters = 1.4219386889821075;
  public double forwardMaxMeters = 2.50;
  public double holdMinSeconds = 0.45;
  public double recalcSeconds = 0.18;
  public double subgoalMaxSeconds = 2.0;
  public double minHeadingErrToTriggerDeg = 6.0;
  public double releaseAheadMeters = 1.35;
  public double angleCostWeight = 0.6538189117352257;
  public double curvatureWeight = 0.5598688771152425;
  public double wallPenaltyGain = 0.55;
  public double occCostGain = 2;
  public double sideSwitchPenalty = 0.35;
  public double headingDeadbandDeg = 4.0;
  public double minProgressMeters = 0.08;
  public double probeOccLegStep = 0.25;
  public double sideBiasProbeAngleDeg = 10.0;
  public double sideStickSeconds = 0.70;
  public double subgoalSlewRateMps = 2.0;
  public double subgoalJitterMeters = 0.04;
  public double arcAngleDeg = 18.0;
  public double arcRMin = 0.9;
  public double arcRMax = 1.8;
  public int arcRadialSteps = 2;
  public int arcAngularStepsPerSide = 2;
  public double relatchImproveFrac = 0.12;
  public double zzzMaxYawDeltaDeg = 22.0;
  public double zzzPenalty = 0.40;
  public double vibWindowS = 0.6;
  public double vibMinDisp = 0.10;
  public int vibMaxDirFlips = 8;
  public double escapeForward = 1.60;
  public double escapeLateral = 1.30;
  public double escapeHoldS = 0.60;
  public double escapeOccBoost = 0.25;
  public double progressCostWeight = 0.05;
  public double minForwardProgressMeters = 0.50;
  public double nearGoalDistMeters = 3.0;
  public double nearGoalForwardScale = 0.60;
  public double nearGoalLateralScale = 0.70;
  public double stuckMinForwardProgress = 0.15;
  public double stuckOccMin = 0.30;
  public double stuckLookbackFrac = 0.7;
  public double sideSwitchStuckPenaltyScale = 0.25;
  public double pinnedOccMin = 0.40;
  public double pinnedMinTimeSeconds = 0.35;
  public double pinnedMaxTimeSeconds = 3.0;
  public double pinnedCooldownSeconds = 0.70;
  public double cornerWallThresh = 3.0;
  public double cornerEscapeLatBoost = 1.8;
  public double cornerEscapeFwdBoost = 1.4;
  public double cornerEscapeBlend = 0.65;
  public double cornerRewardGain = 0.6;
}
