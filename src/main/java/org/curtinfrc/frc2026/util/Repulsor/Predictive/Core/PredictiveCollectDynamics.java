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

import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAgg;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.Track;

final class PredictiveCollectDynamics {
  private static final double ETA_FLOOR = 0.05;
  private static final int PATH_SAMPLES = 7;
  private static final double PATH_COST_W = 0.60;
  private static final double KERNEL_SIGMA = 0.95;
  private static final double SOFTMAX_TEMP = 1.35;

  private PredictiveCollectDynamics() {}

  static double etaPath(Translation2d a, Translation2d b, double speed) {
    double d = a.getDistance(b);
    return Math.max(ETA_FLOOR, d / Math.max(0.1, speed));
  }

  static double estimateTravelTime(
      Translation2d a,
      Translation2d b,
      double speed,
      SpatialDyn dyn,
      HashMap<Integer, Track> allyMap,
      HashMap<Integer, Track> enemyMap) {
    double base = etaPath(a, b, speed);
    if (base <= ETA_FLOOR + 1e-9) return base;

    if (dyn == null) return base;

    double seg = a.getDistance(b);
    if (seg < 1e-3) return base;

    double inv = 1.0 / Math.max(1.0, (double) (PATH_SAMPLES - 1));
    double cost = 0.0;

    for (int i = 0; i < PATH_SAMPLES; i++) {
      double t = i * inv;
      Translation2d p = new Translation2d(lerp(a.getX(), b.getX(), t), lerp(a.getY(), b.getY(), t));

      double c =
          0.85 * radialDensity(allyMap, p, 1.00)
              + 0.95 * radialDensity(enemyMap, p, 1.00)
              + 0.70 * dyn.otherDensity(p, 1.00);

      cost += c;
    }

    cost /= Math.max(1.0, PATH_SAMPLES);
    double mult = 1.0 + PATH_COST_W * Math.min(1.35, cost);
    return Math.max(ETA_FLOOR, base * mult);
  }

  static double minEtaToTarget(HashMap<Integer, Track> map, Translation2d t) {
    double best = Double.POSITIVE_INFINITY;
    for (Track r : map.values()) {
      double s = Math.max(0.1, r.speedCap);
      double e = etaPath(r.pos, t, s);
      if (e < best) best = e;
    }
    return best;
  }

  static double dotNorm(Translation2d a, Translation2d b) {
    double an = a.getNorm();
    double bn = b.getNorm();
    if (an < 1e-6 || bn < 1e-6) return 0.0;
    return (a.getX() * b.getX() + a.getY() * b.getY()) / (an * bn);
  }

  static double radialKernel(double dist) {
    double s2 = KERNEL_SIGMA * KERNEL_SIGMA;
    return Math.exp(-0.5 * (dist * dist) / Math.max(1e-6, s2));
  }

  static double densityKernel(double d, double sigma) {
    double s2 = sigma * sigma;
    return Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
  }

  static double radialDensity(HashMap<Integer, Track> map, Translation2d target, double sigma) {
    if (map.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : map.values()) {
      double d = r.pos.getDistance(target);
      agg += densityKernel(d, sigma);
    }
    return agg / Math.max(1.0, map.size());
  }

  static Translation2d predictAt(Track r, double horizonS) {
    if (r == null) return new Translation2d();
    double h = Math.max(0.0, horizonS);
    if (r.vel == null || r.vel.getNorm() < 1e-9) return r.pos;
    return r.pos.plus(r.vel.times(h));
  }

  static double radialPressure(
      HashMap<Integer, Track> enemies,
      Translation2d target,
      double ourEtaS,
      double reservationRadius,
      double intentMass,
      int count) {
    if (enemies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : enemies.values()) {
      Translation2d p = predictAt(r, ourEtaS);
      double d = p.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - reservationRadius));
      agg += k;
    }
    double base = agg / Math.max(1.0, enemies.size());
    double intent = count > 0 ? (intentMass / Math.max(1.0, count)) : 0.0;
    return base * (1.0 + 0.85 * intent);
  }

  static double radialCongestion(
      HashMap<Integer, Track> allies,
      Translation2d target,
      double ourEtaS,
      double reservationRadius,
      double intentMass,
      int count) {
    if (allies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : allies.values()) {
      Translation2d p = predictAt(r, ourEtaS);
      double d = p.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - reservationRadius));
      agg += k;
    }
    double base = agg / Math.max(1.0, allies.size());
    double intent = count > 0 ? (intentMass / Math.max(1.0, count)) : 0.0;
    return base * (1.0 + 0.75 * intent);
  }

  static double headingAffinity(
      Translation2d ourPos,
      Translation2d target,
      HashMap<Integer, Track> allies,
      HashMap<Integer, Track> enemies) {
    Translation2d toTarget = target.minus(ourPos);
    if (toTarget.getNorm() < 1e-6) return 0.0;
    double allyAlign = 0.0;
    int na = 0;
    for (Track a : allies.values()) {
      Translation2d to = target.minus(a.pos);
      if (to.getNorm() < 1e-6 || a.vel.getNorm() < 1e-6) continue;
      double c = dotNorm(to, a.vel);
      allyAlign += Math.max(0.0, c);
      na++;
    }
    if (na > 0) allyAlign /= na;
    double enemyMis = 0.0;
    int ne = 0;
    for (Track e : enemies.values()) {
      Translation2d to = target.minus(e.pos);
      if (to.getNorm() < 1e-6 || e.vel.getNorm() < 1e-6) continue;
      double c = dotNorm(to, e.vel);
      enemyMis += Math.max(0.0, 1.0 - c);
      ne++;
    }
    if (ne > 0) enemyMis /= ne;
    return 0.6 * allyAlign + 0.4 * enemyMis;
  }

  static IntentAgg softIntentAgg(HashMap<Integer, Track> map, List<Translation2d> targets) {
    int m = targets.size();
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAgg(accum, 0);

    double[] logits = new double[m];
    double[] probs = new double[m];

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = targets.get(i);
        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double logit = -(eta * invT) + 0.35 * align;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double e = Math.exp(logits[i] - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        double w = probs[i] * inv;
        accum[i] += w;
      }
    }

    return new IntentAgg(accum, n);
  }

  static ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
    if (dyn == null) return new ResourceRegions(new Translation2d[0], new double[0]);
    Translation2d[] c =
        PredictiveCollectCandidateBuilder.buildResourceClustersMulti(dyn, Math.max(4, maxRegions));
    double[] m = new double[c.length];
    for (int i = 0; i < c.length; i++) {
      Translation2d p = c[i];
      m[i] = p != null ? dyn.evidenceMassWithin(p, 0.85) : 0.0;
    }
    return new ResourceRegions(c, m);
  }

  static IntentAggCont enemyIntentToRegions(
      HashMap<Integer, Track> map, ResourceRegions regs, double sigma) {
    int m = regs != null && regs.centers != null ? regs.centers.length : 0;
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAggCont(regs.centers, accum, 0, sigma);

    double[] logits = new double[m];
    double[] probs = new double[m];

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = regs.centers[i];
        if (t == null) {
          logits[i] = -1e18;
          continue;
        }

        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double mass = regs.mass != null && i < regs.mass.length ? regs.mass[i] : 0.0;
        double massTerm = Math.log(1.0 + Math.max(0.0, mass));

        double logit = -(eta * invT) + 0.35 * align + 0.22 * massTerm;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double li = logits[i];
        if (li <= -1e17) {
          probs[i] = 0.0;
          continue;
        }
        double e = Math.exp(li - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        accum[i] += probs[i] * inv;
      }
    }

    return new IntentAggCont(regs.centers, accum, n, sigma);
  }

  static IntentAggCont allyIntentToRegions(
      HashMap<Integer, Track> map, ResourceRegions regs, double sigma) {
    int m = regs != null && regs.centers != null ? regs.centers.length : 0;
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAggCont(regs.centers, accum, 0, sigma);

    double[] logits = new double[m];
    double[] probs = new double[m];

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = regs.centers[i];
        if (t == null) {
          logits[i] = -1e18;
          continue;
        }

        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double mass = regs.mass != null && i < regs.mass.length ? regs.mass[i] : 0.0;
        double massTerm = Math.log(1.0 + Math.max(0.0, mass));

        double logit = -(eta * invT) + 0.35 * align + 0.18 * massTerm;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double li = logits[i];
        if (li <= -1e17) {
          probs[i] = 0.0;
          continue;
        }
        double e = Math.exp(li - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        accum[i] += probs[i] * inv;
      }
    }

    return new IntentAggCont(regs.centers, accum, n, sigma);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
