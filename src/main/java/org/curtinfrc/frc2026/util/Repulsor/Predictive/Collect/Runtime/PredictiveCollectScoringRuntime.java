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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Collect.Runtime;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.Track;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;

public final class PredictiveCollectScoringRuntime {
  private PredictiveCollectScoringRuntime() {}

  public static CollectEval evalCollectPoint(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double cap,
      Translation2d p,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {

    CollectEval e = new CollectEval();
    e.p = p != null ? p : new Translation2d();
    if (p == null || dyn == null) {
      e.score = -1e18;
      return e;
    }

    double rCore = PredictiveFieldStateOps.coreRadiusFor(cellM);
    double rJitter = PredictiveFieldStateOps.jitterRadiusFor(cellM);

    e.eta = ops.estimateTravelTime(ourPos, p, cap);

    e.units = dyn.valueAt(p);
    e.count = dyn.countResourcesWithin(p, 0.70);
    e.evidence = dyn.evidenceMassWithin(p, PredictiveFieldStateOps.EVIDENCE_R);

    e.depleted = ops.depletedPenaltySoft(p);
    e.localAvoid = dyn.localAvoidPenalty(p, PredictiveFieldStateOps.COLLECT_LOCAL_AVOID_R);

    e.activity =
        Math.min(
            PredictiveFieldStateOps.ACTIVITY_CAP,
            PredictiveFieldStateOps.COLLECT_ACTIVITY_ALLY_W
                    * PredictiveFieldStateOps.radialDensity(
                        ops.allyMap, p, PredictiveFieldStateOps.COLLECT_ACTIVITY_SIGMA)
                + PredictiveFieldStateOps.COLLECT_ACTIVITY_ENEMY_W
                    * PredictiveFieldStateOps.radialDensity(
                        ops.enemyMap, p, PredictiveFieldStateOps.COLLECT_ACTIVITY_SIGMA)
                + PredictiveFieldStateOps.COLLECT_ACTIVITY_DYN_W
                    * dyn.otherDensity(p, PredictiveFieldStateOps.COLLECT_ACTIVITY_SIGMA));

    double ei = enemyIntent != null ? enemyIntent.intentAt(p) : 0.0;
    double ai = allyIntent != null ? allyIntent.intentAt(p) : 0.0;
    e.enemyIntent = ei;
    e.allyIntent = ai;

    e.enemyPressure = ops.radialPressure(ops.enemyMap, p, e.eta, 0.0, 0);
    e.allyCongestion = ops.radialCongestion(ops.allyMap, p, e.eta, 0.0, 0);

    e.overlap = reservationOverlapPenalty(ops, ops.allyMap, p, e.eta);

    Translation2d nn = dyn.nearestResourceTo(p, rCore);
    e.coreDist = nn != null ? nn.getDistance(p) : Double.POSITIVE_INFINITY;
    e.coreCount = dyn.countResourcesWithin(p, rCore);

    e.robustPenalty = ops.pickupRobustPenalty(dyn, p, rCore, rJitter);

    double sat =
        1.0 - Math.exp(-PredictiveFieldStateOps.COLLECT_VALUE_SAT_K * Math.max(0.0, e.units));
    e.value = sat * Math.max(1, goal) * PredictiveFieldStateOps.COLLECT_VALUE_GAIN;

    double near = Math.exp(-PredictiveFieldStateOps.COLLECT_NEAR_DECAY * e.eta);

    double valueNorm = normalizeValue(ops, dyn, e.value);

    e.score =
        valueNorm
            - e.eta * PredictiveFieldStateOps.COLLECT_ETA_COST
            + near * PredictiveFieldStateOps.COLLECT_NEAR_BONUS
            - e.enemyPressure * PredictiveFieldStateOps.COLLECT_ENEMY_PRESS_COST
            - e.allyCongestion * PredictiveFieldStateOps.COLLECT_ALLY_CONGEST_COST
            - e.enemyIntent * PredictiveFieldStateOps.COLLECT_ENEMY_INTENT_COST
            - e.allyIntent * PredictiveFieldStateOps.COLLECT_ALLY_INTENT_COST
            - e.localAvoid
            - e.activity
            - e.depleted * PredictiveFieldStateOps.DEPLETED_PEN_W
            - e.overlap
            - e.robustPenalty;

    return e;
  }

  public static double normalizeValue(PredictiveFieldStateOps ops, SpatialDyn dyn, double value) {
    double tot = dyn != null ? dyn.totalEvidence() : 0.0;
    double denom = 0.65 + 0.35 * Math.log(1.0 + Math.max(0.0, tot)) + 0.20 * Math.max(0.0, tot);
    return value / Math.max(0.65, denom);
  }

  public static double reservationOverlapPenalty(
      PredictiveFieldStateOps ops,
      HashMap<Integer, Track> allies,
      Translation2d p,
      double ourEtaS) {
    if (allies.isEmpty() || p == null) return 0.0;
    double best = Double.POSITIVE_INFINITY;
    for (Track r : allies.values()) {
      Translation2d pred = PredictiveFieldStateOps.predictAt(r, ourEtaS);
      double d = pred.getDistance(p);
      if (d < best) best = d;
    }
    if (best <= 1e-6) return PredictiveFieldStateOps.RES_OVERLAP_GAIN * 1.25;
    if (best >= PredictiveFieldStateOps.RES_OVERLAP_R) return 0.0;
    double x =
        (PredictiveFieldStateOps.RES_OVERLAP_R - best)
            / Math.max(1e-6, PredictiveFieldStateOps.RES_OVERLAP_R);
    return PredictiveFieldStateOps.RES_OVERLAP_GAIN * (0.35 + 0.95 * x * x);
  }

  public static boolean shouldEscapeCurrentCollect(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      SpatialDyn dyn,
      double totalEv,
      double minUnits,
      int minCount,
      double cellM) {

    if (ops.currentCollectTarget == null || ourPos == null || dyn == null) return true;

    double rCore = PredictiveFieldStateOps.coreRadiusFor(cellM);
    double footprintMinUnits =
        Math.max(0.025, Math.min(PredictiveFieldStateOps.COLLECT_FINE_MIN_UNITS, minUnits * 0.80));
    Rotation2d heading =
        ops.currentCollectHeading != null ? ops.currentCollectHeading : new Rotation2d();
    if (!ops.footprintOk(dyn, ops.currentCollectTarget, heading, rCore, footprintMinUnits))
      return true;

    CollectEval cur =
        evalCollectPoint(
            ops,
            ourPos,
            ops.lastOurCapForCollect > 0.0
                ? ops.lastOurCapForCollect
                : PredictiveFieldStateOps.DEFAULT_OUR_SPEED,
            ops.currentCollectTarget,
            Math.max(1, ops.lastGoalUnitsForCollect),
            cellM,
            dyn,
            null,
            null);

    if (cur.units < Math.max(0.02, minUnits * 0.70)) return true;
    if (cur.count < Math.max(1, minCount - 1)) return true;
    if (cur.depleted > 0.90) return true;
    if (cur.evidence < minEvidence(ops, totalEv) * 0.75) return true;

    double now = Timer.getFPGATimestamp();
    double dist = ourPos.getDistance(ops.currentCollectTarget);

    if (!Double.isFinite(ops.collectProgressLastDist)) {
      ops.collectProgressLastDist = dist;
      ops.collectProgressLastTs = now;
      return false;
    }

    if (dist + 1e-9
        < ops.collectProgressLastDist - PredictiveFieldStateOps.COLLECT_PROGRESS_MIN_DROP_M) {
      ops.collectProgressLastDist = dist;
      ops.collectProgressLastTs = now;
      return false;
    }

    if (now - ops.collectProgressLastTs > PredictiveFieldStateOps.COLLECT_PROGRESS_WINDOW_S
        && dist > PredictiveFieldStateOps.COLLECT_ARRIVE_R + 0.20) {
      return true;
    }

    return false;
  }

  public static double collectCommitWindow(PredictiveFieldStateOps ops, double etaCurrent) {
    double x = clamp01(ops, (etaCurrent - 0.20) / 1.25);
    return lerp(
        ops,
        PredictiveFieldStateOps.COLLECT_COMMIT_MIN_S,
        PredictiveFieldStateOps.COLLECT_COMMIT_MAX_S,
        x);
  }

  public static double minEvidence(PredictiveFieldStateOps ops, double totalEvidence) {
    double x = clamp01(ops, totalEvidence / 6.0);
    return lerp(
        ops,
        PredictiveFieldStateOps.EVIDENCE_MIN_BASE,
        PredictiveFieldStateOps.EVIDENCE_MIN_MAX,
        x);
  }

  public static double dynamicMinUnits(PredictiveFieldStateOps ops, double totalEvidence) {
    double x = clamp01(ops, totalEvidence / 7.5);
    return lerp(ops, 0.05, 0.15, x);
  }

  public static int dynamicMinCount(PredictiveFieldStateOps ops, double totalEvidence) {
    double x = clamp01(ops, totalEvidence / 7.5);
    return (int) Math.round(lerp(ops, 1.0, 3.0, x));
  }

  public static void recordRegionAttempt(
      PredictiveFieldStateOps ops, SpatialDyn dyn, Translation2d p, double now, boolean success) {
    if (dyn == null || p == null) return;
    ops.penaltyTracker.recordRegionAttempt(p, now, success);
  }

  public static double regionBanditBonus(
      PredictiveFieldStateOps ops, SpatialDyn dyn, Translation2d p, double now) {
    if (dyn == null || p == null) return 0.0;
    return ops.penaltyTracker.regionBanditBonus(p, now);
  }

  public static double lerp(PredictiveFieldStateOps ops, double a, double b, double t) {
    double x = Math.max(0.0, Math.min(1.0, t));
    return a + (b - a) * x;
  }

  public static double clamp01(PredictiveFieldStateOps ops, double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  public static double wallDistance(PredictiveFieldStateOps ops, Translation2d p) {
    if (p == null) return 0.0;
    double dx = Math.min(p.getX(), Constants.FIELD_LENGTH - p.getX());
    double dy = Math.min(p.getY(), Constants.FIELD_WIDTH - p.getY());
    return Math.min(dx, dy);
  }

  public static boolean isInvalidFuelBand(PredictiveFieldStateOps ops, Translation2d p) {
    if (p == null) return false;
    double x = p.getX();
    return PredictiveFieldStateOps.X_LEFT_BAND.within(x)
        || PredictiveFieldStateOps.X_RIGHT_BAND.within(x);
  }

  public static boolean defaultCollectResourcePositionFilter(
      PredictiveFieldStateOps ops, Translation2d p) {
    return !isInvalidFuelBand(ops, p);
  }
}
