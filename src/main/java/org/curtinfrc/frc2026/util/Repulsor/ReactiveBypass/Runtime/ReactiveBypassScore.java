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

final class ReactiveBypassScore {
  final Pose2d wp;
  final boolean okLeg1;
  final boolean okLeg2;
  final double pathLen;
  final double angleCost;
  final double curvatureCost;
  final double wallPenalty;
  final double occPathCost;
  final double switchPenalty;
  final double zzzPenalty;
  final double progressCost;
  final double localOccCost;
  final double cornerReward;
  final double totalCost;

  ReactiveBypassScore(
      Pose2d wp,
      boolean okLeg1,
      boolean okLeg2,
      double pathLen,
      double angleCost,
      double curvatureCost,
      double wallPenalty,
      double occPathCost,
      double switchPenalty,
      double zzzPenalty,
      double progressCost,
      double localOccCost,
      double cornerReward,
      double totalCost) {
    this.wp = wp;
    this.okLeg1 = okLeg1;
    this.okLeg2 = okLeg2;
    this.pathLen = pathLen;
    this.angleCost = angleCost;
    this.curvatureCost = curvatureCost;
    this.wallPenalty = wallPenalty;
    this.occPathCost = occPathCost;
    this.switchPenalty = switchPenalty;
    this.zzzPenalty = zzzPenalty;
    this.progressCost = progressCost;
    this.localOccCost = localOccCost;
    this.cornerReward = cornerReward;
    this.totalCost = totalCost;
  }
}
