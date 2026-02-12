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
package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;

public final class PredictiveCollectCandidateBuilder {
  private static final double COLLECT_AGE_DECAY = 0.75;
  private static final double COLLECT_SPREAD_SCORE_R = 0.85;
  private static final double COLLECT_SPREAD_MIN = 0.30;
  private static final double COLLECT_SPREAD_MAX = 0.65;

  private static final int COLLECT_GRID_TOPK = 420;
  private static final int COLLECT_RESOURCE_SEEDS_MAX = 64;
  private static final double COLLECT_CAND_GATE_R = 0.60;
  private static final int COLLECT_CLUSTER_MAX = 72;
  private static final int COLLECT_GRID_FALLBACK_MAX = 1400;

  private PredictiveCollectCandidateBuilder() {}

  public static Translation2d[] peakFinder(Translation2d[] gridPoints, SpatialDyn dyn, int topN) {
    if (gridPoints == null || gridPoints.length == 0 || dyn == null) return new Translation2d[0];

    int K = Math.max(1, topN);
    Translation2d[] bestP = new Translation2d[K];
    double[] bestV = new double[K];
    for (int i = 0; i < K; i++) bestV[i] = -1e18;

    for (int i = 0; i < gridPoints.length; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;

      Translation2d near = dyn.nearestResourceTo(p, 0.60);
      if (near == null) continue;

      double v = dyn.evidenceMassWithin(p, 0.85);
      if (v <= 1e-9) continue;

      int worstI = 0;
      double worst = bestV[0];
      for (int k = 1; k < K; k++) {
        if (bestV[k] < worst) {
          worst = bestV[k];
          worstI = k;
        }
      }
      if (v > worst) {
        bestV[worstI] = v;
        bestP[worstI] = p;
      }
    }

    ArrayList<Translation2d> tmp = new ArrayList<>(K);
    for (Translation2d p : bestP) if (p != null) tmp.add(p);

    tmp = dedupPoints(tmp, adaptiveDupSkip(dyn));
    return tmp.toArray(new Translation2d[0]);
  }

  public static Translation2d[] buildCollectCandidates(
      Translation2d[] gridPoints, SpatialDyn dyn, int peakFinderTopN) {
    Translation2d[] clusters = buildResourceClustersMulti(dyn, COLLECT_CLUSTER_MAX);

    ArrayList<Translation2d> peaks = new ArrayList<>(64);
    if (dyn != null && dyn.resources != null && !dyn.resources.isEmpty() && gridPoints != null) {
      Translation2d[] peakPts = peakFinder(gridPoints, dyn, peakFinderTopN);
      for (Translation2d p : peakPts) if (p != null) peaks.add(p);
    }

    ArrayList<Translation2d> out = new ArrayList<>(256);
    for (Translation2d c : clusters) if (c != null) out.add(c);
    for (Translation2d p : peaks) if (p != null) out.add(p);

    if (dyn != null
        && dyn.resources != null
        && !dyn.resources.isEmpty()
        && dyn.specs != null
        && !dyn.specs.isEmpty()) {
      Translation2d[] bestRes = new Translation2d[Math.max(1, COLLECT_RESOURCE_SEEDS_MAX)];
      double[] bestW = new double[bestRes.length];
      for (int i = 0; i < bestW.length; i++) bestW[i] = -1e18;

      for (DynamicObject o : dyn.resources) {
        if (o == null || o.pos == null || o.type == null) continue;
        ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double ageW =
            PredictiveFieldStateRuntime.error()
                ? 1.0
                : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        double w = Math.max(0.0, s.unitValue) * ageW;

        int worstI = 0;
        double worst = bestW[0];
        for (int k = 1; k < bestW.length; k++) {
          if (bestW[k] < worst) {
            worst = bestW[k];
            worstI = k;
          }
        }
        if (w > worst) {
          bestW[worstI] = w;
          bestRes[worstI] = o.pos;
        }
      }

      for (Translation2d p : bestRes) if (p != null) out.add(p);
    }

    if (gridPoints == null || gridPoints.length == 0) {
      return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
          .toArray(new Translation2d[0]);
    }

    int nGrid = 0;
    for (Translation2d p : gridPoints) if (p != null) nGrid++;
    if (nGrid == 0) {
      return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
          .toArray(new Translation2d[0]);
    }

    int K = Math.min(COLLECT_GRID_TOPK, Math.min(nGrid, COLLECT_GRID_FALLBACK_MAX));
    Translation2d[] bestP = new Translation2d[K];
    Translation2d[] bestN = new Translation2d[K];
    double[] bestS = new double[K];
    for (int i = 0; i < K; i++) bestS[i] = -1e18;

    double gateR = COLLECT_CAND_GATE_R;

    for (int i = 0; i < gridPoints.length; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;
      if (dyn == null) continue;

      Translation2d near = dyn.nearestResourceTo(p, gateR);
      if (near == null) continue;

      double s = dyn.evidenceMassWithin(p, 0.85);
      if (s <= 1e-9) continue;

      int worstI = 0;
      double worst = bestS[0];
      for (int k = 1; k < K; k++) {
        if (bestS[k] < worst) {
          worst = bestS[k];
          worstI = k;
        }
      }
      if (s > worst) {
        bestS[worstI] = s;
        bestP[worstI] = p;
        bestN[worstI] = near;
      }
    }

    for (int i = 0; i < K; i++) {
      Translation2d p = bestP[i];
      if (p != null) out.add(p);
      Translation2d q = bestN[i];
      if (q != null) out.add(q);
    }

    return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
        .toArray(new Translation2d[0]);
  }

  public static ArrayList<Translation2d> dedupPoints(ArrayList<Translation2d> in, double dupSkipM) {
    if (in == null || in.isEmpty()) return new ArrayList<>();
    double d2 = dupSkipM * dupSkipM;
    ArrayList<Translation2d> out = new ArrayList<>(in.size());
    for (int i = 0; i < in.size(); i++) {
      Translation2d p = in.get(i);
      if (p == null) continue;
      boolean dup = false;
      for (int j = 0; j < out.size(); j++) {
        Translation2d q = out.get(j);
        double dx = q.getX() - p.getX();
        double dy = q.getY() - p.getY();
        if (dx * dx + dy * dy <= d2) {
          dup = true;
          break;
        }
      }
      if (!dup) out.add(p);
    }
    return out;
  }

  public static ArrayList<Translation2d> spreadCollectPoints(
      ArrayList<Translation2d> in, SpatialDyn dyn) {
    if (in == null || in.isEmpty()) return new ArrayList<>();
    double sep = adaptiveCollectSeparation(dyn);
    double d2 = sep * sep;
    int n = in.size();
    double[] score = new double[n];
    for (int i = 0; i < n; i++) {
      Translation2d p = in.get(i);
      score[i] =
          (dyn != null && p != null) ? dyn.evidenceMassWithin(p, COLLECT_SPREAD_SCORE_R) : 0.0;
    }

    boolean[] blocked = new boolean[n];
    ArrayList<Translation2d> out = new ArrayList<>(n);

    for (; ; ) {
      int bestI = -1;
      double best = -1e18;
      for (int i = 0; i < n; i++) {
        if (blocked[i]) continue;
        if (score[i] > best) {
          best = score[i];
          bestI = i;
        }
      }
      if (bestI < 0) break;
      Translation2d p = in.get(bestI);
      if (p != null) out.add(p);
      blocked[bestI] = true;
      if (p == null) continue;
      for (int j = 0; j < n; j++) {
        if (blocked[j]) continue;
        Translation2d q = in.get(j);
        if (q == null) {
          blocked[j] = true;
          continue;
        }
        double dx = q.getX() - p.getX();
        double dy = q.getY() - p.getY();
        if (dx * dx + dy * dy <= d2) blocked[j] = true;
      }
    }
    return out;
  }

  public static double adaptiveDupSkip(SpatialDyn dyn) {
    double total = dyn != null ? dyn.totalEvidence() : 0.0;
    double x = clamp01(total / 6.0);
    return lerp(0.18, 0.42, x);
  }

  public static double adaptiveCollectSeparation(SpatialDyn dyn) {
    double total = dyn != null ? dyn.totalEvidence() : 0.0;
    double x = clamp01(total / 6.0);
    return lerp(COLLECT_SPREAD_MIN, COLLECT_SPREAD_MAX, x);
  }

  public static Translation2d[] buildResourceClustersMulti(SpatialDyn dyn, int maxClusters) {
    if (dyn == null || dyn.resources.isEmpty() || dyn.specs.isEmpty()) return new Translation2d[0];

    double coarseBin = Math.max(0.25, Math.min(0.35, 0.30));
    double inv = 1.0 / Math.max(1e-6, coarseBin);

    HashMap<Long, org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ClusterAcc> coarse =
        new HashMap<>(256);

    for (DynamicObject o : dyn.resources) {
      if (o == null || o.pos == null || o.type == null) continue;
      ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
      if (s == null) continue;

      int cx = (int) Math.floor(o.pos.getX() * inv);
      int cy = (int) Math.floor(o.pos.getY() * inv);
      long k = SpatialDyn.key(cx, cy);

      org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ClusterAcc a = coarse.get(k);
      if (a == null) {
        a = new org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ClusterAcc();
        coarse.put(k, a);
      }

      double ageW =
          PredictiveFieldStateRuntime.error()
              ? 1.0
              : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
      double w = Math.max(0.0, s.unitValue) * ageW;

      a.sx += o.pos.getX() * w;
      a.sy += o.pos.getY() * w;
      a.sw += w;
      a.n++;
    }

    if (coarse.isEmpty()) return new Translation2d[0];

    ArrayList<long[]> scored = new ArrayList<>(coarse.size());
    for (var e : coarse.entrySet()) {
      org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ClusterAcc a = e.getValue();
      if (a == null || a.sw <= 1e-9) continue;
      scored.add(new long[] {e.getKey(), Double.doubleToLongBits(a.sw)});
    }

    if (scored.isEmpty()) return new Translation2d[0];

    scored.sort(
        (u, v) -> {
          double a = Double.longBitsToDouble(u[1]);
          double b = Double.longBitsToDouble(v[1]);
          return Double.compare(b, a);
        });

    int m = Math.min(maxClusters, scored.size());
    ArrayList<Translation2d> out = new ArrayList<>(m);

    for (int i = 0; i < m; i++) {
      long key = scored.get(i)[0];
      org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ClusterAcc a = coarse.get(key);
      if (a == null || a.sw <= 1e-9) continue;

      Translation2d c = new Translation2d(a.sx / a.sw, a.sy / a.sw);

      Translation2d refined = meanShiftRefine(dyn, c, 0.75, 3);
      if (refined != null) out.add(refined);
    }

    out = dedupPoints(out, 0.28);
    return out.toArray(new Translation2d[0]);
  }

  public static Translation2d meanShiftRefine(
      SpatialDyn dyn, Translation2d seed, double r, int iters) {
    if (dyn == null || seed == null) return null;
    Translation2d c = seed;
    double rr2 = r * r;

    for (int it = 0; it < Math.max(1, iters); it++) {
      double sx = 0.0;
      double sy = 0.0;
      double sw = 0.0;

      for (DynamicObject o : dyn.resources) {
        if (o == null || o.pos == null || o.type == null) continue;
        ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double dx = o.pos.getX() - c.getX();
        double dy = o.pos.getY() - c.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 > rr2) continue;

        double ageW =
            PredictiveFieldStateRuntime.error()
                ? 1.0
                : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        double w = Math.max(0.0, s.unitValue) * ageW;

        sx += o.pos.getX() * w;
        sy += o.pos.getY() * w;
        sw += w;
      }

      if (sw <= 1e-9) break;

      Translation2d nc = new Translation2d(sx / sw, sy / sw);
      if (nc.getDistance(c) <= 0.02) {
        c = nc;
        break;
      }
      c = nc;
    }

    return c;
  }

  private static double lerp(double a, double b, double t) {
    double x = Math.max(0.0, Math.min(1.0, t));
    return a + (b - a) * x;
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }
}
