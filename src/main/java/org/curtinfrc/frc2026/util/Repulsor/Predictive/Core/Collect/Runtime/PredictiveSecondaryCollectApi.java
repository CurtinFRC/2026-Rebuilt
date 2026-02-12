package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveCollectSecondaryRankers;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.SpatialDyn;

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
