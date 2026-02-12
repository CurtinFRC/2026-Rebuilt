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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAgg;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.Track;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.SpatialDyn;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;

public final class PredictiveFieldStateTrackingRuntime {
  private PredictiveFieldStateTrackingRuntime() {}

  public static void updateAlly(
      PredictiveFieldStateOps ops,
      int id,
      Translation2d pos,
      Translation2d velHint,
      Double speedCap) {
    if (pos == null) return;

    double now = Timer.getFPGATimestamp();
    Track t =
        ops.allyMap.getOrDefault(
            id,
            new Track(
                pos,
                new Translation2d(),
                speedCap != null ? speedCap : PredictiveFieldStateOps.DEFAULT_ALLY_SPEED,
                now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw);
    double dtMeas =
        Math.min(
            PredictiveFieldStateOps.MAX_MEAS_DT, Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap =
        speedCap != null ? Math.max(0.1, speedCap) : PredictiveFieldStateOps.DEFAULT_ALLY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.VEL_EMA, dt);
    double aPos = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.POS_EMA, dt);

    Translation2d vEma = PredictiveFieldStateOps.lerpVec(t.vel, vClamped, aVel);
    t.vel = PredictiveFieldStateOps.clampDeltaV(t.vel, vEma, PredictiveFieldStateOps.ACC_LIMIT, dt);

    t.pos = PredictiveFieldStateOps.lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    ops.allyMap.put(id, t);
  }

  public static void updateEnemy(
      PredictiveFieldStateOps ops,
      int id,
      Translation2d pos,
      Translation2d velHint,
      Double speedCap) {
    if (pos == null) return;

    double now = Timer.getFPGATimestamp();
    Track t =
        ops.enemyMap.getOrDefault(
            id,
            new Track(
                pos,
                new Translation2d(),
                speedCap != null ? speedCap : PredictiveFieldStateOps.DEFAULT_ENEMY_SPEED,
                now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw);
    double dtMeas =
        Math.min(
            PredictiveFieldStateOps.MAX_MEAS_DT, Math.max(PredictiveFieldStateOps.MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap =
        speedCap != null ? Math.max(0.1, speedCap) : PredictiveFieldStateOps.DEFAULT_ENEMY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.VEL_EMA, dt);
    double aPos = PredictiveFieldStateOps.emaAlpha(PredictiveFieldStateOps.POS_EMA, dt);

    Translation2d vEma = PredictiveFieldStateOps.lerpVec(t.vel, vClamped, aVel);
    t.vel = PredictiveFieldStateOps.clampDeltaV(t.vel, vEma, PredictiveFieldStateOps.ACC_LIMIT, dt);

    t.pos = PredictiveFieldStateOps.lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    ops.enemyMap.put(id, t);
  }

  public static void clearStale(PredictiveFieldStateOps ops, double maxAgeS) {
    double now = Timer.getFPGATimestamp();
    ops.allyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
    ops.enemyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
  }

  public static List<Candidate> rank(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    Objects.requireNonNull(ourPos);
    List<GameElement> elems = new ArrayList<>();
    for (GameElement e : ops.worldElements) {
      if (e.getAlliance() == ops.ourAlliance
          && e.getCategory() == cat
          && e.getRelatedPoint().isPresent()
          && !e.isAtCapacity()) {
        elems.add(e);
      }
    }
    if (elems.isEmpty()) return List.of();

    List<Translation2d> targets = new ArrayList<>();
    List<RepulsorSetpoint> sps = new ArrayList<>();
    for (GameElement e : elems) {
      Translation2d p =
          new Translation2d(e.getModel().getPosition().getX(), e.getModel().getPosition().getY());
      targets.add(p);
      sps.add(e.getRelatedPoint().get());
    }

    IntentAgg allyAgg = ops.softIntentAgg(ops.allyMap, targets);
    IntentAgg enemyAgg = ops.softIntentAgg(ops.enemyMap, targets);

    double now = Timer.getFPGATimestamp();
    List<Candidate> out = new ArrayList<>();
    double cap = ourSpeedCap > 0 ? ourSpeedCap : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;

    for (int i = 0; i < targets.size(); i++) {
      Translation2d t = targets.get(i);
      RepulsorSetpoint sp = sps.get(i);

      double ourEta = ops.estimateTravelTime(ourPos, t, cap);
      double enemyEta = PredictiveFieldStateOps.minEtaToTarget(ops.enemyMap, t);
      double allyEta = PredictiveFieldStateOps.minEtaToTarget(ops.allyMap, t);

      double pressure =
          ops.radialPressure(ops.enemyMap, t, ourEta, enemyAgg.intent[i], enemyAgg.count);
      double congestion =
          ops.radialCongestion(ops.allyMap, t, ourEta, allyAgg.intent[i], allyAgg.count);
      double distBias = ourPos.getDistance(t) * PredictiveFieldStateOps.DIST_COST;

      double capacityFrac = 0.0;
      GameElement e = elems.get(i);
      if (e.getMaxContained() > 0) {
        capacityFrac =
            1.0
                - Math.min(
                    1.0, (double) e.getContainedCount() / Math.max(1.0, e.getMaxContained()));
      }

      double heading = ops.headingAffinity(ourPos, t, ops.allyMap, ops.enemyMap);

      double score =
          (enemyEta - ourEta) * PredictiveFieldStateOps.ADV_GAIN
              - congestion * PredictiveFieldStateOps.CONGEST_COST
              - pressure * PredictiveFieldStateOps.PRESSURE_GAIN
              - distBias
              + capacityFrac * PredictiveFieldStateOps.CAPACITY_GAIN
              + heading * PredictiveFieldStateOps.HEADING_GAIN;

      if (ops.lastChosen != null
          && sp.equals(ops.lastChosen)
          && now - ops.lastChosenTs < PredictiveFieldStateOps.HYST_PERSIST_S) {
        score += PredictiveFieldStateOps.HYST_BONUS;
      }

      out.add(new Candidate(sp, t, ourEta, enemyEta, allyEta, congestion, pressure, score));
    }

    out.sort(Comparator.comparingDouble((Candidate c) -> -c.score));
    if (!out.isEmpty()) {
      ops.lastChosen = out.get(0).setpoint;
      ops.lastChosenTs = now;
    }
    if (limit > 0 && out.size() > limit) return new ArrayList<>(out.subList(0, limit));
    return out;
  }

  public static void sortIdxByKey(double[] key, int[] idx) {
    quickSortIdx(key, idx, 0, idx.length - 1);
  }

  public static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    while (lo < hi) {
      int i = lo;
      int j = hi;
      double pivot = key[idx[(lo + hi) >>> 1]];

      while (i <= j) {
        while (key[idx[i]] < pivot) i++;
        while (key[idx[j]] > pivot) j--;
        if (i <= j) {
          int tmp = idx[i];
          idx[i] = idx[j];
          idx[j] = tmp;
          i++;
          j--;
        }
      }

      if (j - lo < hi - i) {
        if (lo < j) quickSortIdx(key, idx, lo, j);
        lo = i;
      } else {
        if (i < hi) quickSortIdx(key, idx, i, hi);
        hi = j;
      }
    }
  }

  public static List<RepulsorSetpoint> rankSetpoints(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    List<Candidate> c = rank(ops, ourPos, ourSpeedCap, cat, limit);
    List<RepulsorSetpoint> out = new ArrayList<>();
    for (Candidate k : c) out.add(k.setpoint);
    return out;
  }

  public static int resourceObservationCount(PredictiveFieldStateOps ops) {
    SpatialDyn d = ops.cachedDyn();
    if (d == null) return 0;
    return d.resources.size();
  }

  public static Translation2d snapToCollectCentroid(
      PredictiveFieldStateOps ops, Translation2d seed, double r, double minMass) {
    if (seed == null) return null;
    SpatialDyn dyn = ops.cachedDyn();
    if (dyn == null) return seed;

    Translation2d c = dyn.centroidResourcesWithin(seed, Math.max(0.05, r), Math.max(0.0, minMass));
    if (c == null) return seed;

    Translation2d ourPos = ops.lastOurPosForCollect != null ? ops.lastOurPosForCollect : seed;
    double cap =
        ops.lastOurCapForCollect > 0.0
            ? ops.lastOurCapForCollect
            : PredictiveFieldStateOps.DEFAULT_OUR_SPEED;
    int goal = Math.max(1, ops.lastGoalUnitsForCollect);
    double cellM = Math.max(0.10, ops.lastCellMForCollect);

    var a = ops.evalCollectPoint(ourPos, cap, seed, goal, cellM, dyn, null, null);
    var b = ops.evalCollectPoint(ourPos, cap, c, goal, cellM, dyn, null, null);

    if (b.score > a.score + 1e-9) return c;
    return seed;
  }

  public static Translation2d nearestCollectResource(
      PredictiveFieldStateOps ops, Translation2d p, double maxDist) {
    if (p == null) return null;
    SpatialDyn d = ops.cachedDyn();
    if (d == null) return null;
    return d.nearestResourceTo(p, Math.max(0.01, maxDist));
  }
}
