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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveCollectSecondaryRankers;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;

public final class PredictiveSecondaryCollectApi implements PredictiveCollectSecondaryRankers.Api {
  private final PredictiveFieldStateOps ops;

  public PredictiveSecondaryCollectApi(PredictiveFieldStateOps ops) {
    this.ops = ops;
  }

  @Override
  public SpatialDyn cachedDyn() {
    return ops.cachedDyn();
  }

  @Override
  public void setCollectContext(
      Translation2d ourPos, double ourSpeedCap, int goalUnits, double cellM) {
    ops.lastOurPosForCollect = ourPos;
    ops.lastOurCapForCollect =
        ourSpeedCap > 0.0 ? ourSpeedCap : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;
    ops.lastGoalUnitsForCollect = Math.max(1, goalUnits);
    ops.lastCellMForCollect = Math.max(0.10, cellM);
  }

  @Override
  public void sweepDepletedMarks() {
    ops.sweepDepletedMarks();
  }

  @Override
  public Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    return ops.buildCollectCandidates(gridPoints, dyn);
  }

  @Override
  public double dynamicMinUnits(double totalEvidence) {
    return ops.dynamicMinUnits(totalEvidence);
  }

  @Override
  public int dynamicMinCount(double totalEvidence) {
    return ops.dynamicMinCount(totalEvidence);
  }

  @Override
  public double minEvidence(double totalEvidence) {
    return ops.minEvidence(totalEvidence);
  }

  @Override
  public ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
    return ops.buildResourceRegions(dyn, maxRegions);
  }

  @Override
  public IntentAggCont enemyIntentToRegions(ResourceRegions regs) {
    return ops.enemyIntentToRegions(ops.enemyMap, regs);
  }

  @Override
  public IntentAggCont allyIntentToRegions(ResourceRegions regs) {
    return ops.allyIntentToRegions(ops.allyMap, regs);
  }

  @Override
  public double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
    return ops.estimateTravelTime(a, b, speed);
  }

  @Override
  public CollectEval evalCollectPoint(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d p,
      int goalUnits,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {
    return ops.evalCollectPoint(
        ourPos, ourSpeedCap, p, goalUnits, cellM, dyn, enemyIntent, allyIntent);
  }

  @Override
  public double allyRadialDensity(Translation2d p, double sigma) {
    return PredictiveFieldStateOps.radialDensity(ops.allyMap, p, sigma);
  }

  @Override
  public double enemyRadialDensity(Translation2d p, double sigma) {
    return PredictiveFieldStateOps.radialDensity(ops.enemyMap, p, sigma);
  }

  @Override
  public double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
    return ops.regionBanditBonus(dyn, p, now);
  }

  @Override
  public void addDepletedMark(
      Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {
    ops.addDepletedMark(p, radiusM, strength, ttlS, merge);
  }

  @Override
  public void addDepletedRing(Translation2d p, double r0, double r1, double strength, double ttlS) {
    ops.addDepletedRing(p, r0, r1, strength, ttlS);
  }

  @Override
  public void setLastReturnedCollect(Translation2d p, double nowS) {
    ops.lastReturnedCollect = p;
    ops.lastReturnedCollectTs = nowS;
  }
}
