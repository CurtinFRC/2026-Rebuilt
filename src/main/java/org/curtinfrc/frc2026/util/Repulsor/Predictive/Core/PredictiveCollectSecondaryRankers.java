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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.PointCandidate;
import org.littletonrobotics.junction.Logger;

public final class PredictiveCollectSecondaryRankers {
  public interface Api {
    SpatialDyn cachedDyn();

    void setCollectContext(Translation2d ourPos, double ourSpeedCap, int goalUnits, double cellM);

    void sweepDepletedMarks();

    Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn);

    double dynamicMinUnits(double totalEvidence);

    int dynamicMinCount(double totalEvidence);

    double minEvidence(double totalEvidence);

    ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions);

    IntentAggCont enemyIntentToRegions(ResourceRegions regs);

    IntentAggCont allyIntentToRegions(ResourceRegions regs);

    double estimateTravelTime(Translation2d a, Translation2d b, double speed);

    CollectEval evalCollectPoint(
        Translation2d ourPos,
        double ourSpeedCap,
        Translation2d p,
        int goalUnits,
        double cellM,
        SpatialDyn dyn,
        IntentAggCont enemyIntent,
        IntentAggCont allyIntent);

    double allyRadialDensity(Translation2d p, double sigma);

    double enemyRadialDensity(Translation2d p, double sigma);

    double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now);

    void addDepletedMark(
        Translation2d p, double radiusM, double strength, double ttlS, boolean merge);

    void addDepletedRing(Translation2d p, double r0, double r1, double strength, double ttlS);

    void setLastReturnedCollect(Translation2d p, double nowS);
  }

  private static final double COLLECT_VALUE_GAIN = 10.10;
  private static final double COLLECT_ETA_COST = 1.05;
  private static final double COLLECT_ENEMY_PRESS_COST = 1.15;
  private static final double COLLECT_ALLY_CONGEST_COST = 0.95;
  private static final double COLLECT_ENEMY_INTENT_COST = 0.75;
  private static final double COLLECT_ALLY_INTENT_COST = 0.55;
  private static final double COLLECT_VALUE_SAT_K = 0.75;
  private static final double COLLECT_ACTIVITY_SIGMA = 1.05;
  private static final double COLLECT_ACTIVITY_ALLY_W = 0.80;
  private static final double COLLECT_ACTIVITY_ENEMY_W = 0.55;
  private static final double COLLECT_ACTIVITY_DYN_W = 0.60;
  private static final double COLLECT_REGION_SAMPLES_W = 2.70;
  private static final double COLLECT_CELL_M = 0.10;
  private static final double COLLECT_NEAR_BONUS = 0.85;
  private static final double COLLECT_NEAR_DECAY = 1.1;
  private static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.12;
  private static final double COLLECT_FINE_MIN_UNITS = 0.08;
  private static final double DEPLETED_TTL_S = 3.25;
  private static final double DEPLETED_PEN_W = 1.75;
  private static final double EVIDENCE_R = 0.85;
  private static final double ACTIVITY_CAP = 1.05;
  private static final int ENEMY_REGIONS_MAX = 24;

  private PredictiveCollectSecondaryRankers() {}

  static PointCandidate rankCollectHierarchical(
      Api api,
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {

    if (ourPos == null) return null;

    SpatialDyn dyn = api.cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    api.setCollectContext(ourPos, ourSpeedCap, goalUnits, cellM);

    double cap = ourSpeedCap > 0.0 ? ourSpeedCap : 3.5;
    int goal = Math.max(1, goalUnits);

    api.sweepDepletedMarks();

    Translation2d[] targets = api.buildCollectCandidates(points, dyn);
    if (targets.length == 0) return null;

    double totalEv = dyn.totalEvidence();
    double minUnits = api.dynamicMinUnits(totalEv);
    int minCount = api.dynamicMinCount(totalEv);

    ResourceRegions enemyRegions = api.buildResourceRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = api.enemyIntentToRegions(enemyRegions);
    IntentAggCont allyIntent = api.allyIntentToRegions(enemyRegions);

    int topK = Math.max(1, Math.min(coarseTopK, Math.max(2, targets.length)));
    int rg = Math.max(2, refineGrid);

    double half = Math.max(0.05, cellM * 0.5);
    double step = (2.0 * half) / (rg - 1);

    int[] bestIdx = new int[topK];
    double[] bestScore = new double[topK];
    for (int i = 0; i < topK; i++) {
      bestIdx[i] = -1;
      bestScore[i] = -1e18;
    }

    int fallbackIdx = -1;
    double fallbackUnits = -1e18;

    double minEv = api.minEvidence(totalEv);

    for (int i = 0; i < targets.length; i++) {
      Translation2d cpt = targets[i];
      if (cpt == null) continue;

      double regionUnitsAny = dyn.valueInSquare(cpt, half);
      if (regionUnitsAny > fallbackUnits) {
        fallbackUnits = regionUnitsAny;
        fallbackIdx = i;
      }

      double ev = dyn.evidenceMassWithin(cpt, EVIDENCE_R);
      if (ev < minEv && regionUnitsAny < Math.max(0.03, minUnits * 0.70)) continue;

      double eta = api.estimateTravelTime(ourPos, cpt, cap);
      double near = Math.exp(-COLLECT_NEAR_DECAY * eta);

      if (regionUnitsAny < Math.min(COLLECT_COARSE_MIN_REGION_UNITS, minUnits * 0.90)) continue;

      double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, regionUnitsAny));
      double value = sat * goal * COLLECT_VALUE_GAIN;

      CollectEval e =
          api.evalCollectPoint(ourPos, cap, cpt, goal, cellM, dyn, enemyIntent, allyIntent);
      double activity =
          Math.min(
              ACTIVITY_CAP,
              COLLECT_ACTIVITY_ALLY_W * api.allyRadialDensity(cpt, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * api.enemyRadialDensity(cpt, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(cpt, COLLECT_ACTIVITY_SIGMA));

      double score =
          value * COLLECT_REGION_SAMPLES_W
              - eta * COLLECT_ETA_COST
              + near * COLLECT_NEAR_BONUS
              - e.enemyPressure * COLLECT_ENEMY_PRESS_COST
              - e.allyCongestion * COLLECT_ALLY_CONGEST_COST
              - e.enemyIntent * COLLECT_ENEMY_INTENT_COST
              - e.allyIntent * COLLECT_ALLY_INTENT_COST
              - activity
              - e.depleted * DEPLETED_PEN_W
              + api.regionBanditBonus(dyn, cpt, Timer.getFPGATimestamp());

      if (ev < minEv) score -= 2.00;

      int insertAt = -1;
      double worst = 1e18;
      int worstK = -1;
      for (int k = 0; k < topK; k++) {
        if (bestIdx[k] < 0) {
          insertAt = k;
          break;
        }
        if (bestScore[k] < worst) {
          worst = bestScore[k];
          worstK = k;
        }
      }
      if (insertAt < 0) {
        if (score > worst && worstK >= 0) insertAt = worstK;
      }
      if (insertAt >= 0) {
        bestIdx[insertAt] = i;
        bestScore[insertAt] = score;
      }
    }

    boolean anyCoarse = false;
    for (int k = 0; k < topK; k++) if (bestIdx[k] >= 0) anyCoarse = true;

    if (!anyCoarse && fallbackIdx >= 0) {
      bestIdx[0] = fallbackIdx;
      bestScore[0] = -1e12;
    }

    final int MAX_TRIES = 10;

    double[] candScore = new double[MAX_TRIES];
    Translation2d[] candPt = new Translation2d[MAX_TRIES];
    CollectEval[] candEval = new CollectEval[MAX_TRIES];
    int candN = 0;

    for (int k = 0; k < topK; k++) {
      int idx = bestIdx[k];
      if (idx < 0) continue;

      Translation2d center = targets[idx];
      if (center == null) continue;

      for (int ix = 0; ix < rg; ix++) {
        double ox = -half + ix * step;
        for (int iy = 0; iy < rg; iy++) {
          double oy = -half + iy * step;
          Translation2d p = new Translation2d(center.getX() + ox, center.getY() + oy);

          CollectEval e =
              api.evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);
          e.banditBonus = api.regionBanditBonus(dyn, p, Timer.getFPGATimestamp());
          e.score += e.banditBonus;

          if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
          if (e.evidence < minEv) e.score -= 2.25;

          int insert = -1;
          if (candN < MAX_TRIES) {
            insert = candN++;
          } else {
            int worstI = 0;
            double worstS = candScore[0];
            for (int t = 1; t < candN; t++) {
              if (candScore[t] < worstS) {
                worstS = candScore[t];
                worstI = t;
              }
            }
            if (e.score > worstS) insert = worstI;
          }

          if (insert >= 0) {
            candScore[insert] = e.score;
            candPt[insert] = p;
            candEval[insert] = e;
          }
        }
      }
    }

    if (candN == 0) return null;

    int[] order = new int[candN];
    for (int i = 0; i < candN; i++) order[i] = i;

    for (int i = 0; i < candN - 1; i++) {
      int best = i;
      double bestS = candScore[order[i]];
      for (int j = i + 1; j < candN; j++) {
        double s = candScore[order[j]];
        if (s > bestS) {
          best = j;
          bestS = s;
        }
      }
      int tmp = order[i];
      order[i] = order[best];
      order[best] = tmp;
    }

    double now = Timer.getFPGATimestamp();

    for (int oi = 0; oi < candN; oi++) {
      int ci = order[oi];
      Translation2d p = candPt[ci];
      CollectEval e0 = candEval[ci];
      if (p == null || e0 == null) continue;

      Translation2d snapped = dyn.centroidResourcesWithin(p, 0.85, 0.20);
      Translation2d use = p;
      CollectEval eUse = e0;

      if (snapped != null) {
        CollectEval es =
            api.evalCollectPoint(ourPos, cap, snapped, goal, cellM, dyn, enemyIntent, allyIntent);
        es.banditBonus = api.regionBanditBonus(dyn, snapped, now);
        es.score += es.banditBonus;

        if (es.units < minUnits * 0.55 || es.count < Math.max(1, minCount - 1)) es.score -= 2.75;
        if (es.evidence < minEv) es.score -= 2.25;

        if (es.score > e0.score + 1e-9) {
          use = snapped;
          eUse = es;
        }
      }

      if (eUse.units >= Math.max(0.02, minUnits * 0.70)
          && eUse.count >= Math.max(1, minCount - 1)) {
        api.setLastReturnedCollect(use, now);

        Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(use, new Rotation2d()));

        return new PointCandidate(
            use,
            new Rotation2d(),
            eUse.eta,
            eUse.value,
            eUse.enemyPressure,
            eUse.allyCongestion,
            eUse.enemyIntent,
            eUse.allyIntent,
            eUse.score);
      }

      api.addDepletedMark(use, 0.70, 1.20, DEPLETED_TTL_S, false);
      api.addDepletedRing(use, 0.35, 0.95, 0.75, DEPLETED_TTL_S);
    }

    return null;
  }

  static Translation2d bestCollectHotspot(Api api, Translation2d[] points, double cellM) {
    if (points == null || points.length == 0) return null;
    SpatialDyn dyn = api.cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    double half = Math.max(0.05, cellM * 0.5);
    Translation2d best = null;
    double bestU = 0.0;

    for (Translation2d p : points) {
      if (p == null) continue;
      double u = dyn.valueInSquare(p, half);
      if (u > bestU) {
        bestU = u;
        best = p;
      }
    }

    double min = Math.max(0.02, COLLECT_FINE_MIN_UNITS * 0.5);
    return bestU >= min ? best : null;
  }

  static PointCandidate rankCollectPoints(
      Api api,
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      int goalUnits,
      int limit) {

    if (ourPos == null) return null;

    SpatialDyn dyn = api.cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    api.setCollectContext(ourPos, ourSpeedCap, goalUnits, COLLECT_CELL_M);

    api.sweepDepletedMarks();

    Translation2d[] targets = api.buildCollectCandidates(points, dyn);
    if (targets == null || targets.length == 0) return null;

    double totalEv = dyn.totalEvidence();
    double minUnits = api.dynamicMinUnits(totalEv);
    int minCount = api.dynamicMinCount(totalEv);

    ResourceRegions enemyRegions = api.buildResourceRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = api.enemyIntentToRegions(enemyRegions);
    IntentAggCont allyIntent = api.allyIntentToRegions(enemyRegions);

    double cap = ourSpeedCap > 0.0 ? ourSpeedCap : 3.5;
    int goal = Math.max(1, goalUnits);

    int maxCheck = limit > 0 ? Math.min(Math.max(1, limit), targets.length) : targets.length;

    double[] etas = new double[targets.length];
    int[] order = new int[targets.length];
    for (int i = 0; i < targets.length; i++) {
      order[i] = i;
      Translation2d t = targets[i];
      etas[i] = t != null ? api.estimateTravelTime(ourPos, t, cap) : Double.POSITIVE_INFINITY;
    }

    sortIdxByKey(etas, order);

    Translation2d bestP = null;
    CollectEval bestE = null;

    double minEv = api.minEvidence(totalEv);

    for (int oi = 0; oi < order.length && oi < maxCheck; oi++) {
      int i = order[oi];
      Translation2d p = targets[i];
      if (p == null) continue;

      CollectEval e =
          api.evalCollectPoint(ourPos, cap, p, goal, COLLECT_CELL_M, dyn, enemyIntent, allyIntent);
      e.banditBonus = api.regionBanditBonus(dyn, p, Timer.getFPGATimestamp());
      e.score += e.banditBonus;

      if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
      if (e.evidence < minEv) e.score -= 2.25;

      if (bestE == null || e.score > bestE.score + 1e-9) {
        bestE = e;
        bestP = p;
      }
    }

    if (bestP == null || bestE == null) return null;

    if (bestE.units < Math.max(0.02, minUnits * 0.70) || bestE.count < Math.max(1, minCount - 1)) {
      api.addDepletedMark(bestP, 0.70, 1.15, DEPLETED_TTL_S, false);
      api.addDepletedRing(bestP, 0.35, 0.95, 0.75, DEPLETED_TTL_S);
      return null;
    }

    api.setLastReturnedCollect(bestP, Timer.getFPGATimestamp());

    Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(bestP, new Rotation2d()));

    return new PointCandidate(
        bestP,
        new Rotation2d(),
        bestE.eta,
        bestE.value,
        bestE.enemyPressure,
        bestE.allyCongestion,
        bestE.enemyIntent,
        bestE.allyIntent,
        bestE.score);
  }

  private static void sortIdxByKey(double[] key, int[] idx) {
    quickSortIdx(key, idx, 0, idx.length - 1);
  }

  private static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    int i = lo;
    int j = hi;
    double p = key[idx[(lo + hi) >>> 1]];
    while (i <= j) {
      while (key[idx[i]] < p) i++;
      while (key[idx[j]] > p) j--;
      if (i <= j) {
        int t = idx[i];
        idx[i] = idx[j];
        idx[j] = t;
        i++;
        j--;
      }
    }
    if (lo < j) quickSortIdx(key, idx, lo, j);
    if (i < hi) quickSortIdx(key, idx, i, hi);
  }
}
